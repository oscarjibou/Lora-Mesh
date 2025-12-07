/*  WiFi LoRa 32 V3  —  Transmisor LoRa SX1262 (modo asíncrono)  */
/*  Oscar • mayo 2025                                          */

#include "transmitter.h"
#include "lora_config.h"
#include "receiver.h" // Para markPacketAsSeen()

// Nota: txDone ahora está definido en lora_config.cpp y es compartido

// --- Variables globales para el protocolo binario ---
static uint16_t mySeqNumber = 0;
static uint8_t sosMode = 0; // 0 = OK, 1 = SOS

// Inicializa el módulo transmisor LoRa
void transmitterSetup()
{
    // 1) Arrancar SPI en los pines físicos correctos
    spiLoRa.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);

    // 2) Inicializar el SX1262 directamente en nuestra banda
    int16_t state = radio.begin(868.0F); // EU868
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.printf("begin() falló: %d\n", state);
        while (true)
            delay(1);
    }

    // 3) Parámetros que casan con el receptor
    radio.setBandwidth(125.0F);
    radio.setSpreadingFactor(12); // usamos SF12 para mayor alcance con menor tasa de datos, o SF7 para mayor tasa de datos y menor alcance
    radio.setCodingRate(5);
    radio.setSyncWord(0x12);
    radio.setCRC(true);
    radio.setOutputPower(14); // 14 dBm (~25 mW)

    // 4) Registrar interrupción de TX-done (ahora usa callback unificado)
    radio.setDio1Action(onRadioIrq);

    Serial.println("Transmisor listo.");
}

// Envía un mensaje por el transmisor LoRa
bool sendMessage(String message)
{
    int16_t st = radio.startTransmit(message); // NO bloquea
    if (st == RADIOLIB_ERR_NONE)
    {
        Serial.printf("[TX] --> \"%s\"\n", message.c_str());
        return true;
    }
    else
    {
        Serial.printf("startTransmit() error %d\n", st);
        return false;
    }
}

// Procesa la transmisión (debe llamarse periódicamente)
void transmitterLoop()
{
    // ¿Terminó la radio de emitir?
    if (txDone)
    {
        txDone = false;
        Serial.println("[TX] Paquete enviado correctamente.");
        
        // Volver a modo recepción para seguir escuchando (modo mesh)
        switchToReceiveMode();
    }
}

// [NUEVO] Envía el estado del nodo en protocolo binario (14 bytes)
void sendMyStatus()
{
    // Crear buffer binario de 14 bytes
    uint8_t buffer[PACKET_SIZE];

    // Simular datos (en producción, obtener de sensores reales)
    float latitude = 39.492032;
    float longitude = -0.374011;

    // Serializar cada campo en orden (big-endian para consistencia)
    buffer[0] = MY_ID;      // src (1 byte)
    // buffer[1] = GATEWAY_ID; // dst (1 byte)

    // seq (2 bytes)
    buffer[1] = (mySeqNumber >> 8) & 0xFF;
    buffer[2] = mySeqNumber & 0xFF;
    uint16_t currentSeq = mySeqNumber; // Guardar número de secuencia antes de incrementar
    mySeqNumber++;

    buffer[3] = 3; // ttl (1 byte) - mínimo 3 saltos

    // lat (4 bytes) - copiar bytes como float
    memcpy(&buffer[4], &latitude, 4);

    // lon (4 bytes)
    memcpy(&buffer[8], &longitude, 4);

    buffer[12] = sosMode; // state (1 byte): 0=OK, 1=SOS

    // === MEJORA 2: MARCAR PAQUETE PROPIO COMO VISTO ANTES DE ENVIAR ===
    // Así si nos llega de vuelta por reenvío, lo ignoraremos inmediatamente
    markPacketAsSeen(MY_ID, currentSeq);

    // Preparar para TX (actualizar estado esperado)
    switchToTransmitMode();
    
    // Enviar por LoRa
    int16_t st = radio.startTransmit(buffer, PACKET_SIZE);
    if (st == RADIOLIB_ERR_NONE)
    {
        Serial.printf("[MESH-TX] Enviado: src=%d, dst=%d, seq=%d, ttl=3, state=%d\n",
                      MY_ID, GATEWAY_ID, currentSeq, sosMode);
    }
    else
    {
        Serial.printf("[MESH-TX] Error: %d\n", st);
        // Si falló, volver a RX
        switchToReceiveMode();
    }
}

// [NUEVO] Alternar modo SOS
void toggleSOS()
{
    sosMode = (sosMode == 0) ? 1 : 0;
    Serial.printf("[MESH] Modo SOS: %s\n", sosMode ? "ACTIVADO" : "DESACTIVADO");
}
