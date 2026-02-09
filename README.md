# LoRa Mesh Network

Una red mesh inalambrica basada en LoRa para comunicacion entre multiples nodos con capacidades de reenvio automatico, deteccion de caidas mediante maquina de estados, localizacion GPS y visualizacion en tiempo real.

## Descripcion del Proyecto

Este proyecto implementa una **red mesh LoRa** utilizando modulos Heltec WiFi LoRa 32 V3 (ESP32-S3) con chips SX1262. Los nodos pueden:

- Transmitir su estado periodicamente (con datos de sensores y GPS)
- Recibir mensajes de otros nodos
- Reenviar automaticamente mensajes recibidos (store & forward)
- Evitar bucles infinitos mediante deteccion de paquetes duplicados
- Prevenir colisiones mediante mecanismos de desincronizacion temporal
- Mostrar informacion en pantalla OLED integrada
- Detectar caidas mediante una maquina de estados (FSM) con el MPU6050
- Obtener posicion GPS en tiempo real con el modulo NEO-6M
- Ejecutar deteccion de caidas en paralelo usando FreeRTOS (dual-core)

## Hardware Requerido

### Componentes Principales
- **Heltec WiFi LoRa 32 V3** (ESP32-S3 + SX1262)
  - Microcontrolador: ESP32-S3 (dual-core)
  - Radio LoRa: SX1262
  - Pantalla OLED: SSD1306 (128x64)
  - Banda de frecuencia: 868 MHz (EU868)

### Sensores
- **MPU6050**: Acelerometro y giroscopio de 6 ejes + sensor de temperatura
  - Conectado via I2C en bus secundario (Wire1): SDA: 4, SCL: 5
  - Utilizado para la deteccion de caidas (FSM)

- **GPS NEO-6M**: Receptor GPS para localizacion
  - Conectado via UART (Serial2): TX: 2, RX: 3
  - Velocidad: 9600 baud
  - Proporciona latitud, longitud, altitud, hora y numero de satelites

### Pines Utilizados

| Componente | Pin | Funcion |
|------------|-----|---------|
| **LoRa SX1262** | | |
| | 9 | SCK (SPI Clock) |
| | 10 | MOSI (SPI Data Out) |
| | 11 | MISO (SPI Data In) |
| | 8 | NSS/CS (Chip Select) |
| | 12 | RST (Reset) |
| | 13 | BUSY (Busy Signal) |
| | 14 | DIO1 (Interrupt) |
| **OLED Display** | | |
| | 17 | SDA (I2C Data) |
| | 18 | SCL (I2C Clock) |
| | 21 | RST (Reset) |
| | 36 | Vext (Power Control) |
| **MPU6050** | | |
| | 4 | SDA (I2C Data - Wire1) |
| | 5 | SCL (I2C Clock - Wire1) |
| **GPS NEO-6M** | | |
| | 2 | TX (UART Serial2 RX) |
| | 3 | RX (UART Serial2 TX) |

## Arquitectura del Sistema

### Arquitectura Dual-Core (FreeRTOS)

El sistema aprovecha los dos nucleos del ESP32-S3 para ejecutar tareas en paralelo:

```
┌─────────────────────────────────────────────────────────────┐
│                     ESP32-S3 Dual Core                      │
├──────────────────────────┬──────────────────────────────────┤
│       NUCLEO 0           │          NUCLEO 1                │
│                          │                                  │
│  ┌────────────────────┐  │  ┌────────────────────────────┐  │
│  │ fallDetectionTask  │  │  │         loop()             │  │
│  │                    │  │  │                            │  │
│  │ - Lee MPU6050      │  │  │ - Actualiza GPS            │  │
│  │ - Procesa FSM      │  │  │ - Lee MPU6050 (datos)      │  │
│  │ - Detecta caidas   │  │  │ - TX estado propio         │  │
│  │ - Muestreo 20Hz    │  │  │ - Procesa RX               │  │
│  │                    │  │  │ - Store & Forward           │  │
│  └────────────────────┘  │  └────────────────────────────┘  │
│                          │                                  │
│     Prioridad: 2         │        Prioridad: 1              │
└──────────────────────────┴──────────────────────────────────┘
                    │
            ┌───────────────┐
            │  I2C Mutex    │
            │ (SemaphoreHandle_t) │
            │ Protege acceso │
            │ compartido al  │
            │ bus I2C        │
            └───────────────┘
```

### Modulos del Sistema

```
┌─────────────────────────────────────────────────────┐
│                    main.cpp                         │
│  - Coordina todos los modulos                      │
│  - Loop principal (Nucleo 1)                       │
│  - Tarea deteccion caidas (Nucleo 0, FreeRTOS)     │
│  - Gestion de GPS y espera de fix                  │
│  - Mutex I2C para acceso compartido                │
└──────────┬───────────────────────────┬──────────────┘
           │                           │
    ┌──────┼──────────┬────────────────┤
    │      │          │                │
    ▼      ▼          ▼                ▼
┌────────┐┌────────┐┌─────────┐┌──────────────┐
│  LoRa  ││Display ││ MPU6050 ││  GPS Driver  │
│ Config ││        ││  (FSM)  ││  (NEO-6M)    │
└───┬────┘└────────┘└─────────┘└──────────────┘
    │
 ┌──┴──┐
 │     │
 ▼     ▼
┌────┐┌────┐
│ TX ││ RX │
└────┘└────┘
```

### Descripcion de Componentes

#### 1. **lora_config** (`lora_config.h/cpp`)
- Configuracion centralizada del modulo LoRa (SX1262)
- Gestion de interrupciones unificada (TX/RX) via callback `onRadioIrq()`
- Cambio de modo (RX <-> TX) con `switchToReceiveMode()` / `switchToTransmitMode()`
- Mecanismos anti-colision (jitter, offset por ID, separacion temporal)
- Protocolo binario de comunicacion (14 bytes)
- Funciones de separacion temporal (`canSendOwnTx()`, `canForward()`)
- Buffer circular de paquetes vistos (anti-loop)

#### 2. **transmitter** (`transmitter.h/cpp`)
- Envio de estado del nodo en formato binario (14 bytes)
- Gestion de numeros de secuencia incrementales
- Obtencion de coordenadas GPS reales del modulo NEO-6M
- Inclusion de estado de caida (`fallDetected`) en cada paquete
- Modo SOS conmutable (`toggleSOS()`)
- Marcado preventivo de paquetes propios como vistos antes de enviar

#### 3. **receiver** (`receiver.h/cpp`)
- Recepcion de paquetes binarios (14 bytes)
- Sistema anti-loop (buffer circular de paquetes vistos)
- Deteccion inmediata de paquetes propios (ignorados sin procesar)
- Reenvio automatico (store & forward) con decremento de TTL
- Verificacion de separacion temporal antes de reenviar
- Visualizacion de paquetes recibidos en display OLED

#### 4. **display** (`display.h/cpp`)
- Control de pantalla OLED SSD1306 (128x64, I2C en 0x3C)
- Gestion de alimentacion Vext (pin 36)
- Funciones de texto normal (ArialMT_Plain_10) y grande (ArialMT_Plain_16)
- Limpieza de pantalla

#### 5. **mpu6050** (`mpu6050.h/cpp`)
- Lectura de acelerometro (X, Y, Z) y giroscopio (X, Y, Z)
- Sensor de temperatura
- Calculo de angulos (roll, pitch)
- **Maquina de estados (FSM) de deteccion de caidas** con 4 estados:
  - `STATE_NORMAL`: Esperando inicio de caida libre
  - `STATE_FREE_FALL`: Detectando duracion de caida libre (80-300ms)
  - `STATE_FREE_FALL_CONFIRMED`: Ventana temporal para detectar impacto (100ms-1s)
  - `STATE_IMPACT_DETECTED`: Caida confirmada, notificar y resetear
- Umbrales configurables: caida libre (<2.45 m/s2), impacto (>26.46 m/s2)
- Escaneo de dispositivos I2C para depuracion

#### 6. **gps_driver** (`gps_driver.h/cpp`)
- Driver para el modulo GPS NEO-6M via UART (Serial2)
- Lectura de latitud, longitud, altitud
- Lectura de hora (UTC), numero de satelites, HDOP
- Deteccion de fix valido (`hasFix()`) y edad de datos (`getAge()`)
- Estadisticas de diagnostico (chars procesados, checksums fallidos)

## Protocolo de Comunicacion

### Formato de Paquete Binario (14 bytes)

```
┌─────┬──────────┬─────┬──────────┬──────────┬─────┬──────┐
│ src │   seq    │ ttl │   lat    │   lon    │ sos │ fall │
│(1B) │  (2B)    │(1B) │  (4B)    │  (4B)    │(1B) │ (1B) │
└─────┴──────────┴─────┴──────────┴──────────┴─────┴──────┘
  0     1-2       3     4-7       8-11       12     13
```

**Campos:**
- `src` (1 byte): ID del nodo origen (1-254)
- `seq` (2 bytes): Numero de secuencia (big-endian, uint16)
- `ttl` (1 byte): Time To Live (saltos restantes, por defecto 3)
- `lat` (4 bytes): Latitud (float, IEEE 754). Valor `-1.0` indica sin fix GPS
- `lon` (4 bytes): Longitud (float, IEEE 754). Valor `-1.0` indica sin fix GPS
- `sos` (1 byte): Estado del nodo (0=OK, 1=SOS)
- `fall` (1 byte): Estado de caida (0=sin caida, 1=caida detectada por FSM)

### IDs Especiales
- `MY_ID`: ID del nodo actual (configurable en `lora_config.h`)
- `GATEWAY_ID`: ID del nodo gateway (tipicamente 1)
- `BROADCAST_ID`: 255 (broadcast a todos los nodos)

## Configuracion LoRa

### Parametros de Radio

```cpp
Frecuencia:       868.0 MHz (EU868)
Bandwidth:        125 kHz
Spreading Factor: 12 (SF12 - maximo alcance)
Coding Rate:      4/5
Sync Word:        0x12
CRC:              Habilitado
Potencia:         14 dBm (~25 mW)
```

### Modos de Operacion

#### Modo Gateway (`isGatewayOnly = true`)
- Solo recibe mensajes
- No transmite su estado
- No reenvia mensajes
- Util para nodos centrales de recoleccion

#### Modo Mesh (`isGatewayOnly = false`)
- Transmite su estado periodicamente
- Recibe mensajes de otros nodos
- Reenvia mensajes automaticamente
- Participa activamente en la red mesh

## Maquina de Estados de Deteccion de Caidas (FSM)

El sistema implementa una maquina de estados de 3 fases para detectar caidas reales (caida libre seguida de impacto), reduciendo falsos positivos:

```
┌──────────────────┐
│   STATE_NORMAL   │◄──────────────────────────────────────┐
│ Esperando        │                                       │
│ free-fall        │                                       │
└────────┬─────────┘                                       │
         │ accel_mag < 2.45 m/s2                           │
         ▼                                                 │
┌──────────────────┐                                       │
│ STATE_FREE_FALL  │── duracion < 80ms ──> NORMAL ─────────┤
│ Midiendo duracion│── duracion > 300ms ─> NORMAL ─────────┤
│ de caida libre   │                                       │
└────────┬─────────┘                                       │
         │ accel_mag >= 2.45 m/s2                          │
         │ Y duracion 80-300ms                             │
         ▼                                                 │
┌────────────────────────┐                                 │
│ STATE_FREE_FALL_       │── timeout > 1000ms ─> NORMAL ───┤
│ CONFIRMED              │                                 │
│ Ventana para impacto   │                                 │
│ (100ms - 1000ms)       │                                 │
└────────┬───────────────┘                                 │
         │ accel_mag > 26.46 m/s2                          │
         │ Y tiempo en ventana                             │
         ▼                                                 │
┌──────────────────┐                                       │
│ STATE_IMPACT_    │                                       │
│ DETECTED         │── notificar caida ──> NORMAL ─────────┘
│ Caida confirmada │
└──────────────────┘
```

### Parametros de la FSM

| Parametro | Valor | Descripcion |
|-----------|-------|-------------|
| `FREE_FALL_THRESHOLD` | 2.45 m/s2 | Umbral de magnitud para detectar caida libre |
| `IMPACT_THRESHOLD` | 26.46 m/s2 | Umbral de magnitud para detectar impacto |
| `FREE_FALL_MIN_DURATION_MS` | 80 ms | Duracion minima de caida libre para confirmar |
| `FREE_FALL_MAX_DURATION_MS` | 300 ms | Duracion maxima de caida libre antes de reset |
| `IMPACT_WINDOW_MIN_MS` | 100 ms | Ventana minima para detectar impacto tras caida libre |
| `IMPACT_WINDOW_MAX_MS` | 1000 ms | Ventana maxima para detectar impacto tras caida libre |

### Ejecucion en Paralelo

La deteccion de caidas corre en una tarea FreeRTOS independiente en el **Nucleo 0** del ESP32-S3:

- **Frecuencia de muestreo**: 20 Hz (cada 50ms)
- **Prioridad**: 2 (alta)
- **Stack**: 4096 bytes
- **Comunicacion**: Variable volatil `fallDetected` compartida con el loop principal
- **Proteccion I2C**: Mutex (`SemaphoreHandle_t`) para acceso seguro al bus I2C compartido con el display

## Mecanismos Anti-Colision

El sistema implementa multiples estrategias para evitar colisiones:

### 1. Desincronizacion Inicial por ID
```cpp
Offset inicial = (MY_ID - 1) * OFFSET_PER_ID_MS
```
Cada nodo espera un tiempo inicial diferente basado en su ID antes de la primera transmision.

### 2. Jitter Aleatorio
```cpp
Jitter = random(0, MAX_JITTER_MS + 1)
```
Se aplica un delay aleatorio antes y despues de cada transmision para evitar sincronizacion.

### 3. Separacion Temporal
- **Despues de TX propio**: Espera `SEPARATION_AFTER_OWN_TX_MS` antes de permitir reenvio
- **Despues de reenvio**: Espera `SEPARATION_AFTER_FORWARD_MS` antes de permitir TX propio

### 4. Intervalo de Transmision
```cpp
TX_INTERVAL_MS = 10000  // 10 segundos (configurable)
```

## Sistema Anti-Loop

Para evitar bucles infinitos en la red mesh:

### Buffer Circular de Paquetes Vistos
- Almacena los ultimos `MAX_SEEN_PACKETS` (10) paquetes
- Cada entrada contiene: `(src, seq)`
- Verificacion antes de reenviar

### Deteccion de Paquetes Propios
- Los nodos ignoran **inmediatamente** sus propios paquetes (sin procesar nada mas)
- Los paquetes se marcan como vistos **antes** de transmitir (prevencion proactiva)

### Time To Live (TTL)
- Cada paquete tiene un TTL inicial de 3
- Se decrementa en cada reenvio
- Cuando TTL = 0, el paquete no se reenvia

## Flujo de Operacion

### Secuencia de Inicio (`setup()`)

```
1. Inicializar Serial (115200 baud)
2. Crear mutex I2C (xSemaphoreCreateMutex)
3. Inicializar bus I2C principal (Wire, 400kHz)
4. Inicializar generador aleatorio (semilla basada en ID + tiempo)
5. Inicializar pantalla OLED
6. Inicializar sensor MPU6050 (bus Wire1, 100kHz)
7. Inicializar GPS NEO-6M (Serial2, 9600 baud)
8. [Opcional] Esperar fix GPS valido (si WAIT_FOR_GPS_FIX = true)
9. Crear tarea FreeRTOS de deteccion de caidas (Nucleo 0, prioridad 2)
10. Inicializar radio LoRa en modo mesh (recepcion)
```

### Loop Principal (`loop()` - Nucleo 1)

```
┌─────────────────────────────────────┐
│ 1. Actualizar datos GPS             │
│    gps.update()                     │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│ 2. Leer sensores MPU6050            │
│    (protegido con mutex I2C)        │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│ 3. Es tiempo de TX propio?          │
│    - Verificar intervalo/offset     │
│    - Verificar separacion temporal  │
│    - Verificar radio lista          │
│    - Aplicar jitter pre-TX          │
│    - Enviar sendMyStatus()          │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│ 4. TX completado?                   │
│    - Aplicar jitter post-TX         │
│    - Volver a modo RX               │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│ 5. Escuchar y reenviar              │
│    listenAndForward()               │
│    - Procesar paquetes recibidos    │
│    - Verificar anti-loop            │
│    - Reenviar si TTL > 0            │
└─────────────────────────────────────┘
```

### Procesamiento de Recepcion

```
Paquete recibido
    │
    ├─ Es paquete propio? ──SI──> Ignorar inmediatamente
    │                             (sin deserializar resto)
    │
    └─ NO
        │
        ├─ Ya visto (src,seq)? ──SI──> Ignorar
        │
        └─ NO
            │
            ├─ Marcar como visto
            │
            ├─ Mostrar en display OLED
            │
            ├─ TTL > 0? ──NO──> Fin
            │
            └─ SI
                │
                ├─ Puede reenviar? (separacion temporal)
                │   └─ NO ──> Volver a RX
                │
                └─ SI ──> Decrementar TTL ──> Reenviar
```

## Instalacion y Configuracion

### Requisitos Previos

1. **PlatformIO** instalado (recomendado) o Arduino IDE
2. **Librerias necesarias** (se instalan automaticamente con PlatformIO):
   - RadioLib (v7.1.1+)
   - Heltec ESP32 Display (v0.1.0+)
   - Adafruit MPU6050 (v2.2.6+)
   - Adafruit Unified Sensor (v1.1.9+)
   - TinyGPSPlus (mikalhart)

### Configuracion del Proyecto

1. **Clonar/Descargar el proyecto**
   ```bash
   cd Lora-Mesh
   ```

2. **Configurar el ID del nodo**
   
   Editar `include/lora_config.h`:
   ```cpp
   #define MY_ID 1  // Cambiar segun el nodo (1, 2, 3, ...)
   ```

3. **Configurar modo de operacion**
   
   Editar `src/main.cpp`:
   ```cpp
   const bool isGatewayOnly = false;  // true = solo gateway, false = mesh
   ```

4. **Configurar espera de GPS**
   
   Editar `src/main.cpp`:
   ```cpp
   const bool WAIT_FOR_GPS_FIX = true;  // true = espera fix GPS antes de iniciar
   ```

5. **Ajustar intervalo de transmision** (opcional)
   
   Editar `src/main.cpp`:
   ```cpp
   const unsigned long TX_INTERVAL_MS = 10000;  // 10 segundos
   ```

### Compilacion y Carga

#### Con PlatformIO (Recomendado)

```bash
# Compilar
pio run

# Subir al dispositivo
pio run --target upload

# Monitorear serial
pio device monitor
```

#### Con Arduino IDE

1. Instalar el soporte para ESP32
2. Instalar las librerias necesarias desde el Library Manager
3. Seleccionar: **Heltec WiFi LoRa 32 V3**
4. Compilar y subir

## Uso

### Inicializacion

Al encender el dispositivo:

1. Se inicializa la pantalla OLED
2. Se inicializa el sensor MPU6050 (bus I2C secundario)
3. Se inicializa el modulo GPS NEO-6M (UART)
4. Se espera fix GPS valido (si `WAIT_FOR_GPS_FIX = true`)
   - Muestra en pantalla: satelites visibles, chars procesados, HDOP
5. Se lanza la tarea de deteccion de caidas en Nucleo 0 (FreeRTOS)
6. Se configura el modulo LoRa en modo recepcion

### Operacion Normal

- **Nodo Mesh**: Transmite su estado cada `TX_INTERVAL_MS` milisegundos
- **Recepcion**: Escucha continuamente en modo RX
- **Reenvio**: Reenvia automaticamente paquetes validos (store & forward)
- **GPS**: Actualiza posicion continuamente en cada iteracion del loop
- **Deteccion de caidas**: Corre en paralelo a 20Hz en Nucleo 0
- **Display**: Muestra informacion de paquetes recibidos y alertas de caida

### Monitor Serial

El puerto serial (115200 baud) muestra:

```
Inicializando nodo mesh...
[MESH] Nodo ID: 1 - Generador aleatorio inicializado
Display inicializado
MPU6050 inicializado
GPS NEO-6M inicializado. Esperando fix...
[GPS] Esperando senal GPS...
[GPS] Satelites: 5, Chars: 1234, HDOP: 150
[GPS] Fix obtenido! Lat: 40.416800, Lon: -3.703800
[FALL] Tarea de deteccion de caidas iniciada (FSM)
Tarea de deteccion de caidas iniciada en nucleo 0
[MESH] Radio inicializada - Modo RECEPCION
Modo: NODO MESH (envia estado propio + reenvia mensajes)

[MAIN] Enviando estado propio...
[MESH-TX] Enviado: src=1, dst=1, seq=0, ttl=3, state=0, lat=40.416800, lon=-3.703800, st=0
[MESH-RX] src=2, seq=5, ttl=2, rssi=-85.3 dBm
          lat=41.385064, lon=2.173404, state=0
[MESH] Reenviando paquete (nuevo ttl=1)...

========================================
    CAIDA CONFIRMADA (FSM)
========================================
TIPO: IMPACTO
----------------------------------------
Valor detectado: 28.50
Umbral superado: 26.46
----------------------------------------
Valores actuales:
  Accel Mag: 28.50 m/s2
  Roll:      15.30 deg
  Pitch:     -5.20 deg
  Gyro Mag:  1.50 rad/s
========================================
```

## Parametros Configurables

### En `lora_config.h`

| Parametro | Valor por Defecto | Descripcion |
|-----------|-------------------|-------------|
| `MY_ID` | 1 | ID unico del nodo |
| `GATEWAY_ID` | 1 | ID del nodo gateway |
| `BROADCAST_ID` | 255 | ID de broadcast |
| `PACKET_SIZE` | 14 | Tamano del paquete (bytes) |
| `MAX_JITTER_MS` | 300 | Jitter maximo (ms) |
| `OFFSET_PER_ID_MS` | 1000 | Offset por ID (ms) |
| `SEPARATION_AFTER_FORWARD_MS` | 500 | Separacion despues de reenvio (ms) |
| `SEPARATION_AFTER_OWN_TX_MS` | 500 | Separacion despues de TX propio (ms) |
| `MAX_SEEN_PACKETS` | 10 | Tamano del buffer anti-loop |

### En `main.cpp`

| Parametro | Valor por Defecto | Descripcion |
|-----------|-------------------|-------------|
| `isGatewayOnly` | false | Modo gateway o mesh |
| `TX_INTERVAL_MS` | 10000 | Intervalo de transmision (ms) |
| `WAIT_FOR_GPS_FIX` | true | Esperar fix GPS antes de iniciar |

### En `gps_driver.h`

| Parametro | Valor por Defecto | Descripcion |
|-----------|-------------------|-------------|
| `GPS_TX_PIN` | 2 | Pin TX del GPS (conectado a RX del ESP32) |
| `GPS_RX_PIN` | 3 | Pin RX del GPS (conectado a TX del ESP32) |
| `GPS_BAUD` | 9600 | Velocidad UART del GPS |
| `GPS_NO_FIX_VALUE` | -1.0 | Valor enviado cuando no hay fix GPS |

### En `mpu6050.cpp` (Umbrales de deteccion de caidas)

| Parametro | Valor por Defecto | Descripcion |
|-----------|-------------------|-------------|
| `FREE_FALL_THRESHOLD` | 2.45 m/s2 | Umbral para detectar caida libre |
| `IMPACT_THRESHOLD` | 26.46 m/s2 | Umbral para detectar impacto |
| `FREE_FALL_MIN_DURATION_MS` | 80 ms | Duracion minima de caida libre |
| `FREE_FALL_MAX_DURATION_MS` | 300 ms | Duracion maxima de caida libre |
| `IMPACT_WINDOW_MIN_MS` | 100 ms | Ventana minima para impacto |
| `IMPACT_WINDOW_MAX_MS` | 1000 ms | Ventana maxima para impacto |

## Estructura del Proyecto

```
Lora-Mesh/
├── include/                  # Archivos de cabecera
│   ├── display.h             # Control de pantalla OLED
│   ├── gps_driver.h          # Driver GPS NEO-6M
│   ├── lora_config.h         # Configuracion central LoRa y protocolo mesh
│   ├── mpu6050.h             # Sensor MPU6050 y FSM de deteccion de caidas
│   ├── receiver.h            # Recepcion y reenvio de paquetes
│   └── transmitter.h         # Transmision de paquetes
├── src/                      # Codigo fuente
│   ├── main.cpp              # Loop principal, FreeRTOS, GPS, coordinacion
│   └── modules/              # Implementaciones
│       ├── display.cpp       # Pantalla OLED SSD1306
│       ├── gps_driver.cpp    # GPS NEO-6M via Serial2
│       ├── lora_config.cpp   # Radio SX1262 y logica mesh
│       ├── mpu6050.cpp       # Sensor y maquina de estados
│       ├── receiver.cpp      # Recepcion y store & forward
│       └── transmitter.cpp   # Transmision y protocolo binario
├── lib/                      # Librerias personalizadas
├── test/                     # Tests unitarios
├── platformio.ini            # Configuracion PlatformIO
├── UnderstandingC++.md       # Notas de aprendizaje C++
└── README.md                 # Este archivo
```

## Solucion de Problemas

### La radio no inicializa
- Verificar conexiones SPI
- Comprobar que los pines esten correctamente definidos
- Revisar que el chip SX1262 este alimentado

### No se reciben paquetes
- Verificar que todos los nodos usen los mismos parametros LoRa
- Comprobar que `SYNC_WORD` sea igual en todos los nodos (0x12)
- Verificar que la frecuencia sea 868.0 MHz (EU868)

### Colisiones frecuentes
- Aumentar `TX_INTERVAL_MS`
- Aumentar `MAX_JITTER_MS`
- Aumentar `OFFSET_PER_ID_MS`

### Bucles infinitos
- Verificar que `MAX_SEEN_PACKETS` sea suficiente
- Comprobar que el TTL inicial sea razonable (3-5)
- Asegurar que los nodos tengan IDs unicos

### Display no funciona
- Verificar conexion I2C (SDA: 17, SCL: 18)
- Comprobar que Vext (pin 36) este configurado correctamente
- Revisar direccion I2C (0x3C)

### GPS no obtiene fix
- Verificar conexiones UART (TX: pin 2, RX: pin 3)
- Asegurar antena GPS con vista al cielo
- Comprobar velocidad UART (9600 baud)
- Primera adquisicion puede tardar varios minutos (cold start)

### MPU6050 no detecta caidas
- Verificar conexion I2C secundaria (SDA: 4, SCL: 5)
- Comprobar direccion I2C (0x68 o 0x69)
- Ajustar umbrales `FREE_FALL_THRESHOLD` e `IMPACT_THRESHOLD` segun necesidad
- Revisar que la tarea FreeRTOS este corriendo (`[FALL] Tarea de deteccion de caidas iniciada`)

### Conflictos I2C
- El display (Wire) y MPU6050 (Wire1) usan buses I2C separados
- El mutex `i2cMutex` protege acceso compartido al MPU6050 entre nucleo 0 y nucleo 1
- Si hay bloqueos, verificar que `xSemaphoreGive()` se llame siempre despues de `xSemaphoreTake()`

## Referencias y Recursos

- [RadioLib Documentation](https://github.com/jgromes/RadioLib)
- [Heltec WiFi LoRa 32 V3 Datasheet](https://heltec.org/project/wifi-lora-32-v3/)
- [SX1262 Datasheet](https://www.semtech.com/products/wireless-rf/lora-transceivers/sx1262)
- [TinyGPS++ Library](http://arduiniana.org/libraries/tinygpsplus/)
- [Adafruit MPU6050 Library](https://github.com/adafruit/Adafruit_MPU6050)
- [FreeRTOS ESP32 Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html)
- [LoRaWAN Regional Parameters](https://lora-alliance.org/)

## Autor

**Oscar** - TFG Telecomunicaciones - 2025

## Licencia

Este proyecto es parte de un Trabajo de Fin de Grado (TFG) en Telecomunicaciones.

---

## Notas para el Desarrollo

### Conceptos Clave

- **Store & Forward**: Los nodos almacenan y reenvian mensajes automaticamente
- **TTL (Time To Live)**: Limita el numero de saltos para evitar bucles infinitos
- **Anti-Loop**: Buffer circular de deteccion de paquetes duplicados
- **Jitter**: Variacion aleatoria en tiempos para evitar sincronizacion
- **Separacion Temporal**: Tiempo minimo entre diferentes tipos de transmision
- **FSM (Finite State Machine)**: Maquina de estados para deteccion fiable de caidas
- **FreeRTOS**: Sistema operativo en tiempo real para ejecucion multi-tarea en dual-core
- **Mutex I2C**: Semaforo para proteger acceso concurrente al bus I2C

### Mejoras Futuras

- [ ] Implementar encriptacion de paquetes (AES-128)
- [ ] Anadir sistema de routing mas inteligente (tablas de rutas)
- [ ] Implementar QoS (Quality of Service)
- [ ] Anadir soporte para multiples gateways
- [ ] Implementar sleep mode para ahorro de energia
- [ ] Anadir logging persistente (SD card o SPIFFS)
- [ ] Implementar OTA (Over-The-Air) updates
- [ ] Dashboard web para visualizacion de la red (via gateway WiFi)
- [ ] Mejorar protocolo con campo de destino (unicast/multicast)
- [ ] Anadir confirmacion de recepcion (ACK) para mensajes criticos
