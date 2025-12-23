#include <Arduino.h>
#include <transmitter.h>
#include <receiver.h>
#include <display.h>
#include <lora_config.h>
#include <Wire.h>
#include <mpu6050.h>

// Configuración del modo del nodo
// true = Solo gateway (solo recibe, no envía ni reenvía)
// false = Nodo mesh completo (envía su estado Y reenvía mensajes)
const bool isGatewayOnly = false;

// Configuración del intervalo de transmisión (milisegundos)
// Ajusta este valor para cambiar cuánto tiempo pasa escuchando antes de enviar
// 10000 = 10 segundos (recomendado para más tiempo escuchando)
// 15000 = 15 segundos
// 30000 = 30 segundos
const unsigned long TX_INTERVAL_MS = 10000; // 10 segundos

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  Wire.begin(SDA_OLED, SCL_OLED);
  Wire.setClock(400000);  // Frecuencia del display

  Serial.println("Inicializando nodo mesh...");

  // Inicializar generador aleatorio con semilla basada en MY_ID y tiempo
  // Esto asegura que cada nodo tenga una secuencia aleatoria diferente
  randomSeed(analogRead(0) + MY_ID * 1000 + millis());
  
  Serial.printf("[MESH] Nodo ID: %d - Generador aleatorio inicializado\n", MY_ID);

  // Inicializar la pantalla OLED
  initDisplay();

  Serial.println("Display inicializado");

  // Inicializar el sensor MPU6050
  initMPU6050();
  Serial.println("MPU6050 inicializado");

  // Inicializar radio en modo mesh (soporta TX y RX)
  meshRadioSetup();

  if (isGatewayOnly)
  {
    Serial.println("Modo: GATEWAY (solo recepción)");
  }
  else
  {
    Serial.println("Modo: NODO MESH (envía estado propio + reenvía mensajes)");
    
    // Calcular offset inicial basado en MY_ID para desincronizar nodos
    unsigned long initialOffset = (MY_ID - 1) * OFFSET_PER_ID_MS;
    if (initialOffset > 0)
    {
      Serial.printf("[MESH] Offset inicial configurado: %lu ms (ID %d)\n", initialOffset, MY_ID);
      Serial.println("[MESH] La primera transmisión se retrasará este tiempo para desincronizar nodos");
    }
  }
}

void loop()
{
  static unsigned long lastTxTime = 0;
  static unsigned long systemStartTime = 0; // Tiempo de inicio del sistema
  static bool firstTxDone = false; // Para aplicar offset solo a la primera TX
  unsigned long currentTime = millis();

  // Obtener eventos de sensores
  MPU6050Data data;
  if (!readMPU6050(&data)) {
    Serial.println("Error al leer el sensor MPU6050");
    delay(500);
    return;
  }


  // Inicializar tiempo de inicio en la primera ejecución
  if (systemStartTime == 0)
  {
    systemStartTime = currentTime;
  }

  // === 1. Si es nodo mesh (no gateway), enviar estado propio periódicamente ===
  if (!isGatewayOnly)
  {
    unsigned long timeSinceStart = currentTime - systemStartTime;
    unsigned long timeSinceLastTx = currentTime - lastTxTime;
    unsigned long initialOffset = (MY_ID - 1) * OFFSET_PER_ID_MS;
    
    // Verificar si es tiempo de transmitir
    bool timeToTx;
    if (!firstTxDone)
    {
      // Primera TX: esperar solo el offset inicial (no el intervalo completo)
      // Esto hace que los nodos se desincronicen desde el inicio
      timeToTx = (timeSinceStart >= initialOffset);
      
      if (initialOffset > 0 && timeSinceStart < initialOffset)
      {
        // Aún no ha pasado el offset inicial, mostrar mensaje una vez
        static bool offsetMsgShown = false;
        if (!offsetMsgShown)
        {
          Serial.printf("[MESH] Esperando offset inicial: %lu ms antes de primera TX (ID %d)\n", initialOffset, MY_ID);
          offsetMsgShown = true;
        }
      }
    }
    else
    {
      // Transmisiones siguientes: usar el intervalo normal
      timeToTx = (timeSinceLastTx >= TX_INTERVAL_MS);
    }
    
    // Enviar estado propio cuando sea tiempo Y la radio esté lista Y haya separación temporal
    if (timeToTx && isRadioReadyForTX() && canSendOwnTx())
    {
      // Aplicar jitter aleatorio antes de transmitir (evita sincronización)
      unsigned long jitter = random(0, MAX_JITTER_MS + 1);
      
      if (jitter > 0)
      {
        Serial.printf("[MAIN] Aplicando jitter: %lu ms antes de TX\n", jitter);
        delay(jitter);
        // Actualizar tiempo después del jitter
        currentTime = millis();
      }
      
      Serial.println("[MAIN] Enviando estado propio...");
      displayText("Enviando estado...", 0, 0);

      // Mostrar datos del acelerómetro
      Serial.println("\n--- DATOS DEL SENSOR MPU6050 ---");
      Serial.println("Acelerómetro (m/s²):");
      Serial.printf("  X: %.2f\n", data.accel.acceleration.x);
      Serial.printf("  Y: %.2f\n", data.accel.acceleration.y);
      Serial.printf("  Z: %.2f\n", data.accel.acceleration.z);
      Serial.println("Giroscopio (rad/s):");
      Serial.printf("  X: %.2f\n", data.gyro.gyro.x);
      Serial.printf("  Y: %.2f\n", data.gyro.gyro.y);
      Serial.printf("  Z: %.2f\n", data.gyro.gyro.z);
      Serial.println("Temperatura (°C):");
      Serial.printf("  %.2f\n", data.temp.temperature);

      // Enviar usando protocolo binario (18 bytes)
      sendMyStatus();
      
      // === MEJORA 3: Actualizar rastreo de TX propio ===
      setLastTxType(1); // 1 = TX propio
      
      lastTxTime = currentTime;
      firstTxDone = true; // Marcar que ya se hizo la primera transmisión
    }
    else if (timeToTx && !canSendOwnTx())
    {
      // Esperar separación temporal antes de poder enviar
      unsigned long timeSinceLastTx = currentTime - lastAnyTxTime;
      unsigned long requiredSeparation = (lastTxType == 2) ? SEPARATION_AFTER_FORWARD_MS : SEPARATION_AFTER_OWN_TX_MS;
      if (timeSinceLastTx < requiredSeparation)
      {
        static unsigned long lastSepMsg = 0;
        if (currentTime - lastSepMsg > 2000) // Mostrar mensaje cada 2 segundos
        {
          Serial.printf("[MESH] Esperando separación temporal: %lu ms restantes\n", 
                        requiredSeparation - timeSinceLastTx);
          lastSepMsg = currentTime;
        }
      }
    }
  }

  // === 2. Procesar finalización de transmisión ===
  // Si terminó de transmitir (ya sea estado propio o reenvío), añadir separación y volver a RX
  if (txDone)
  {
    txDone = false;
    
    // === MEJORA 3: Añadir jitter después de TX para separación temporal ===
    unsigned long jitter = random(0, MAX_JITTER_MS + 1);
    if (jitter > 0)
    {
      Serial.printf("[MESH] Aplicando jitter post-TX: %lu ms\n", jitter);
      delay(jitter);
      currentTime = millis(); // Actualizar tiempo después del jitter
    }
    
    Serial.println("[MAIN] Transmisión completada - volviendo a modo RX");
    switchToReceiveMode();
  }

  // === 3. Escuchar y reenviar mensajes (mientras está en RX) ===
  // Esta función maneja recepción y reenvío automático
  listenAndForward();

  // Pequeño delay para evitar saturar el procesador
  delay(10);
}
