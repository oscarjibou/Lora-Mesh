# 📡 LoRa Mesh Network

Una red mesh inalámbrica basada en LoRa para comunicación entre múltiples nodos con capacidades de reenvío automático, detección de sensores y visualización en tiempo real.

## 🎯 Descripción del Proyecto

Este proyecto implementa una **red mesh LoRa** utilizando módulos Heltec WiFi LoRa 32 V3 (ESP32-S3) con chips SX1262. Los nodos pueden:

- ✅ Transmitir su estado periódicamente (con datos de sensores)
- ✅ Recibir mensajes de otros nodos
- ✅ Reenviar automáticamente mensajes recibidos (store & forward)
- ✅ Evitar bucles infinitos mediante detección de paquetes duplicados
- ✅ Prevenir colisiones mediante mecanismos de desincronización temporal
- ✅ Mostrar información en pantalla OLED integrada
- ✅ Leer datos de sensores MPU6050 (acelerómetro, giroscopio, temperatura)

## 🛠️ Hardware Requerido

### Componentes Principales
- **Heltec WiFi LoRa 32 V3** (ESP32-S3 + SX1262)
  - Microcontrolador: ESP32-S3
  - Radio LoRa: SX1262
  - Pantalla OLED: SSD1306 (128x64)
  - Banda de frecuencia: 868 MHz (EU868)

### Sensores (Opcional)
- **MPU6050**: Acelerómetro y giroscopio de 6 ejes + sensor de temperatura
  - Conectado vía I2C en pines dedicados (SDA: 4, SCL: 5)

### Pines Utilizados

| Componente | Pin | Función |
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
| | 4 | SDA (I2C Data) |
| | 5 | SCL (I2C Clock) |

## 🏗️ Arquitectura del Sistema

### Módulos Principales

```
┌─────────────────────────────────────────┐
│           main.cpp                      │
│  - Coordina todos los módulos           │
│  - Loop principal de ejecución          │
│  - Gestión de temporización             │
└──────────────┬──────────────────────────┘
               │
    ┌──────────┼──────────┐
    │          │          │
    ▼          ▼          ▼
┌─────────┐ ┌─────────┐ ┌─────────┐
│ LoRa    │ │ Display │ │ MPU6050 │
│ Config  │ │         │ │         │
└────┬────┘ └─────────┘ └─────────┘
     │
  ┌──┴──┐
  │     │
  ▼     ▼
┌────┐ ┌────┐
│ TX │ │ RX │
└────┘ └────┘
```

### Componentes del Sistema

#### 1. **lora_config** (`lora_config.h/cpp`)
- Configuración centralizada del módulo LoRa
- Gestión de interrupciones (TX/RX)
- Cambio de modo (RX ↔ TX)
- Mecanismos anti-colisión
- Protocolo binario de comunicación

#### 2. **transmitter** (`transmitter.h/cpp`)
- Envío de estado del nodo en formato binario
- Gestión de números de secuencia
- Protocolo de paquetes de 14 bytes

#### 3. **receiver** (`receiver.h/cpp`)
- Recepción de paquetes binarios
- Sistema anti-loop (detección de duplicados)
- Reenvío automático (store & forward)
- Procesamiento de TTL (Time To Live)

#### 4. **display** (`display.h/cpp`)
- Control de pantalla OLED SSD1306
- Visualización de información en tiempo real
- Texto normal y grande

#### 5. **mpu6050** (`mpu6050.h/cpp`)
- Lectura de acelerómetro (X, Y, Z)
- Lectura de giroscopio (X, Y, Z)
- Sensor de temperatura
- Cálculo de ángulos (roll, pitch)

## 📦 Protocolo de Comunicación

### Formato de Paquete Binario (14 bytes)

```
┌─────┬─────┬─────┬─────┬──────────┬──────────┬─────┐
│ src │ seq │ seq │ ttl │   lat    │   lon    │state│
│(1B) │(1B) │(1B) │(1B) │  (4B)    │  (4B)    │(1B) │
└─────┴─────┴─────┴─────┴──────────┴──────────┴─────┘
  0     1     2     3     4-7       8-11      12
```

**Campos:**
- `src` (1 byte): ID del nodo origen (1-254)
- `seq` (2 bytes): Número de secuencia (big-endian)
- `ttl` (1 byte): Time To Live (saltos restantes)
- `lat` (4 bytes): Latitud (float, IEEE 754)
- `lon` (4 bytes): Longitud (float, IEEE 754)
- `state` (1 byte): Estado del nodo (0=OK, 1=SOS)

### IDs Especiales
- `MY_ID`: ID del nodo actual (configurable en `lora_config.h`)
- `GATEWAY_ID`: ID del nodo gateway (típicamente 1)
- `BROADCAST_ID`: 255 (broadcast a todos los nodos)

## ⚙️ Configuración LoRa

### Parámetros de Radio

```cpp
Frecuencia:     868.0 MHz (EU868)
Bandwidth:      125 kHz
Spreading Factor: 12 (SF12 - máximo alcance)
Coding Rate:    4/5
Sync Word:      0x12
CRC:            Habilitado
Potencia:       14 dBm (~25 mW)
```

### Modos de Operación

#### Modo Gateway (`isGatewayOnly = true`)
- Solo recibe mensajes
- No transmite su estado
- No reenvía mensajes
- Útil para nodos centrales de recolección

#### Modo Mesh (`isGatewayOnly = false`)
- Transmite su estado periódicamente
- Recibe mensajes de otros nodos
- Reenvía mensajes automáticamente
- Participa activamente en la red mesh

## 🔄 Mecanismos Anti-Colisión

El sistema implementa múltiples estrategias para evitar colisiones:

### 1. **Desincronización Inicial por ID**
```cpp
Offset inicial = (MY_ID - 1) × OFFSET_PER_ID_MS
```
Cada nodo espera un tiempo inicial diferente basado en su ID antes de la primera transmisión.

### 2. **Jitter Aleatorio**
```cpp
Jitter = random(0, MAX_JITTER_MS + 1)
```
Se aplica un delay aleatorio antes de cada transmisión para evitar sincronización.

### 3. **Separación Temporal**
- **Después de TX propio**: Espera `SEPARATION_AFTER_OWN_TX_MS` antes de permitir reenvío
- **Después de reenvío**: Espera `SEPARATION_AFTER_FORWARD_MS` antes de permitir TX propio

### 4. **Intervalo de Transmisión**
```cpp
TX_INTERVAL_MS = 10000  // 10 segundos (configurable)
```

## 🛡️ Sistema Anti-Loop

Para evitar bucles infinitos en la red mesh:

### Buffer Circular de Paquetes Vistos
- Almacena los últimos `MAX_SEEN_PACKETS` (10) paquetes
- Cada entrada contiene: `(src, seq)`
- Verificación antes de reenviar

### Detección de Paquetes Propios
- Los nodos ignoran inmediatamente sus propios paquetes
- Se marcan como vistos antes de transmitir

### Time To Live (TTL)
- Cada paquete tiene un TTL inicial (típicamente 3)
- Se decrementa en cada reenvío
- Cuando TTL = 0, el paquete no se reenvía

## 📊 Flujo de Operación

### Loop Principal (`main.cpp`)

```
┌─────────────────────────────────────┐
│ 1. Leer sensores (MPU6050)         │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│ 2. ¿Es tiempo de TX propio?         │
│    - Verificar intervalo            │
│    - Verificar separación temporal  │
│    - Aplicar jitter                 │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│ 3. ¿TX completado?                  │
│    - Aplicar jitter post-TX         │
│    - Volver a modo RX               │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│ 4. Escuchar y reenviar              │
│    - Procesar paquetes recibidos    │
│    - Verificar anti-loop            │
│    - Reenviar si TTL > 0            │
└─────────────────────────────────────┘
```

### Procesamiento de Recepción

```
Paquete recibido
    │
    ├─ ¿Es paquete propio? ──SÍ──> Ignorar
    │                              └─> Marcar como visto
    │
    └─ NO
        │
        ├─ ¿Ya visto? ──SÍ──> Ignorar
        │
        └─ NO
            │
            ├─ Marcar como visto
            │
            ├─ Mostrar en display
            │
            ├─ ¿TTL > 0? ──NO──> Fin
            │
            └─ SÍ
                │
                ├─ ¿Puede reenviar? (separación temporal)
                │
                └─ SÍ ──> Decrementar TTL ──> Reenviar
```

## 🚀 Instalación y Configuración

### Requisitos Previos

1. **PlatformIO** instalado (recomendado) o Arduino IDE
2. **Librerías necesarias** (se instalan automáticamente con PlatformIO):
   - RadioLib (v7.1.1+)
   - Heltec ESP32 Display
   - Adafruit MPU6050
   - Adafruit Unified Sensor

### Configuración del Proyecto

1. **Clonar/Descargar el proyecto**
   ```bash
   cd Lora-Mesh
   ```

2. **Configurar el ID del nodo**
   
   Editar `include/lora_config.h`:
   ```cpp
   #define MY_ID 1  // Cambiar según el nodo (1, 2, 3, ...)
   ```

3. **Configurar modo de operación**
   
   Editar `src/main.cpp`:
   ```cpp
   const bool isGatewayOnly = false;  // true = solo gateway, false = mesh
   ```

4. **Ajustar intervalo de transmisión** (opcional)
   
   Editar `src/main.cpp`:
   ```cpp
   const unsigned long TX_INTERVAL_MS = 10000;  // 10 segundos
   ```

### Compilación y Carga

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
2. Instalar las librerías necesarias desde el Library Manager
3. Seleccionar: **Heltec WiFi LoRa 32 V3**
4. Compilar y subir

## 📝 Uso

### Inicialización

Al encender el dispositivo:

1. Se inicializa la pantalla OLED
2. Se inicializa el sensor MPU6050
3. Se configura el módulo LoRa en modo recepción
4. Se muestra información en el display

### Operación Normal

- **Nodo Mesh**: Transmite su estado cada `TX_INTERVAL_MS` segundos
- **Recepción**: Escucha continuamente en modo RX
- **Reenvío**: Reenvía automáticamente paquetes válidos
- **Display**: Muestra información de paquetes recibidos

### Monitor Serial

El puerto serial (115200 baud) muestra:

```
[MESH] Nodo ID: 1 - Generador aleatorio inicializado
Display inicializado
MPU6050 inicializado
[MESH] Radio inicializada - Modo RECEPCIÓN
Modo: NODO MESH (envía estado propio + reenvía mensajes)

[MESH-TX] Enviado: src=1, dst=1, seq=0, ttl=3, state=0, lat=40.416800, lon=-3.703800
[MESH-RX] src=2, seq=5, ttl=2, rssi=-85.3 dBm
[MESH] Reenviando paquete (nuevo ttl=1)...
```

## 🔧 Parámetros Configurables

### En `lora_config.h`

| Parámetro | Valor por Defecto | Descripción |
|-----------|-------------------|-------------|
| `MY_ID` | 1 | ID único del nodo |
| `GATEWAY_ID` | 1 | ID del nodo gateway |
| `MAX_JITTER_MS` | 300 | Jitter máximo (ms) |
| `OFFSET_PER_ID_MS` | 1000 | Offset por ID (ms) |
| `SEPARATION_AFTER_FORWARD_MS` | 500 | Separación después de reenvío (ms) |
| `SEPARATION_AFTER_OWN_TX_MS` | 500 | Separación después de TX propio (ms) |
| `MAX_SEEN_PACKETS` | 10 | Tamaño del buffer anti-loop |
| `PACKET_SIZE` | 14 | Tamaño del paquete (bytes) |

### En `main.cpp`

| Parámetro | Valor por Defecto | Descripción |
|-----------|-------------------|-------------|
| `isGatewayOnly` | false | Modo gateway o mesh |
| `TX_INTERVAL_MS` | 10000 | Intervalo de transmisión (ms) |

## 📁 Estructura del Proyecto

```
Lora-Mesh/
├── include/              # Archivos de cabecera
│   ├── display.h
│   ├── lora_config.h     # Configuración central LoRa
│   ├── mpu6050.h
│   ├── receiver.h
│   └── transmitter.h
├── src/                  # Código fuente
│   ├── main.cpp          # Loop principal
│   └── modules/          # Implementaciones
│       ├── display.cpp
│       ├── lora_config.cpp
│       ├── mpu6050.cpp
│       ├── receiver.cpp
│       └── transmitter.cpp
├── lib/                  # Librerías personalizadas
├── test/                 # Tests (si aplica)
├── platformio.ini        # Configuración PlatformIO
└── README.md             # Este archivo
```

## 🐛 Solución de Problemas

### La radio no inicializa
- Verificar conexiones SPI
- Comprobar que los pines estén correctamente definidos
- Revisar que el chip SX1262 esté alimentado

### No se reciben paquetes
- Verificar que todos los nodos usen los mismos parámetros LoRa
- Comprobar que `SYNC_WORD` sea igual en todos los nodos
- Verificar que la frecuencia sea 868.0 MHz (EU868)

### Colisiones frecuentes
- Aumentar `TX_INTERVAL_MS`
- Aumentar `MAX_JITTER_MS`
- Aumentar `OFFSET_PER_ID_MS`

### Bucles infinitos
- Verificar que `MAX_SEEN_PACKETS` sea suficiente
- Comprobar que el TTL inicial sea razonable (3-5)
- Asegurar que los nodos tengan IDs únicos

### Display no funciona
- Verificar conexión I2C
- Comprobar que Vext esté configurado correctamente
- Revisar dirección I2C (0x3C)

## 📚 Referencias y Recursos

- [RadioLib Documentation](https://github.com/jgromes/RadioLib)
- [Heltec WiFi LoRa 32 V3 Datasheet](https://heltec.org/project/wifi-lora-32-v3/)
- [SX1262 Datasheet](https://www.semtech.com/products/wireless-rf/lora-transceivers/sx1262)
- [LoRaWAN Regional Parameters](https://lora-alliance.org/)

## 👤 Autor

**Oscar** - Mayo 2025

## 📄 Licencia

Este proyecto es parte de un Trabajo de Fin de Grado (TFG) en Telecomunicaciones.

---

## 🎓 Notas para el Desarrollo

### Conceptos Clave

- **Store & Forward**: Los nodos almacenan y reenvían mensajes automáticamente
- **TTL (Time To Live)**: Limita el número de saltos para evitar bucles infinitos
- **Anti-Loop**: Sistema de detección de paquetes duplicados
- **Jitter**: Variación aleatoria en tiempos para evitar sincronización
- **Separación Temporal**: Tiempo mínimo entre diferentes tipos de transmisión

### Mejoras Futuras

- [ ] Implementar encriptación de paquetes
- [ ] Añadir sistema de routing más inteligente
- [ ] Implementar QoS (Quality of Service)
- [ ] Añadir soporte para múltiples gateways
- [ ] Implementar sleep mode para ahorro de energía
- [ ] Añadir logging persistente
- [ ] Implementar OTA (Over-The-Air) updates

---

**¡Disfruta construyendo tu red mesh LoRa!** 🚀📡

