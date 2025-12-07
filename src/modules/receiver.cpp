/*  WiFi LoRa 32 V3  —  Receptor LoRa SX1262 (RadioLib)  */
/*  Oscar • mayo 2025                                 */

#include "receiver.h"
#include "display.h"
#include "lora_config.h"

// Nota: paqueteRecibido ahora está definido en lora_config.cpp y es compartido

// --- Variables globales para el mesh anti-loop ---
SeenPacket seenPackets[MAX_SEEN_PACKETS];
uint8_t seenPacketsIndex = 0;

// [NUEVO] Verificar si un paquete ya ha sido visto
bool hasSeenPacket(uint8_t src, uint16_t seq)
{
    for (int i = 0; i < MAX_SEEN_PACKETS; i++)
    {
        if (seenPackets[i].src == src && seenPackets[i].seq == seq)
        {
            return true;
        }
    }
    return false;
}

// [NUEVO] Registrar un paquete como visto
void markPacketAsSeen(uint8_t src, uint16_t seq)
{
    seenPackets[seenPacketsIndex].src = src;
    seenPackets[seenPacketsIndex].seq = seq;
    seenPacketsIndex = (seenPacketsIndex + 1) % MAX_SEEN_PACKETS;
}

void receiverSetup()
{
    // 1) Inicializamos SPI en los pines correctos
    spiLoRa.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);

    // 2) Arrancamos la radio directamente en nuestra banda
    int16_t state = radio.begin(868.0F); // 868 MHz (EU868)
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.printf("Fallo en begin(): %d\n", state);
        while (true)
            delay(1);
    }

    // 3) Parámetros a juego con tu transmisor
    radio.setBandwidth(125.0F);   // kHz
    radio.setSpreadingFactor(12); // SF7
    radio.setCodingRate(5);       // 4/5
    radio.setSyncWord(0x12);      // mismo que tu TX
    radio.setCRC(true);

    // 4) Registramos la interrupción de RX-done (ahora usa callback unificado)
    radio.setDio1Action(onRadioIrq);

    // 5) Comenzamos a escuchar (modo continuo)
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println("Receptor listo—esperando paquetes…");
    }
    else
    {
        Serial.printf("startReceive() error %d\n", state);
    }

    // Inicializar buffer de paquetes vistos
    memset(seenPackets, 0, sizeof(seenPackets));
}

void receiverLoop()
{
    if (paqueteRecibido)
    {
        paqueteRecibido = false; // limpiar flag

        String payload;
        int16_t state = radio.readData(payload); // leer ya decodificado
        if (state == RADIOLIB_ERR_NONE)
        {
            float rssi = radio.getRSSI();
            float snr = radio.getSNR();
            // Serial.printf("[RX] %s  |  RSSI %.1f dBm  SNR %.1f dB\n",
            //               payload.c_str(), rssi, snr);
            Serial.println(payload);
            displayText(payload.c_str(), 0, 0); // mostrar en pantalla
            displayText((String(rssi, 1) + " dBm").c_str(), 0, 15);
            displayText((String(snr, 1) + " dB").c_str(), 0, 30);
            delay(1000); // mostrar 1 segundo
            clearDisplay();
        }
        else
        {
            Serial.printf("readData() error %d\n", state);
        }

        // Volvemos al modo recepción
        switchToReceiveMode();
    }
}

// [NUEVO] Escucha, procesa y reenvía paquetes binarios (mesh)
void listenAndForward()
{
    if (paqueteRecibido)
    {
        paqueteRecibido = false;

        // Leer buffer binario de exactamente PACKET_SIZE bytes
        uint8_t buffer[PACKET_SIZE];
        int16_t state = radio.readData(buffer, PACKET_SIZE);

        if (state == RADIOLIB_ERR_NONE)
        {
            // Deserializar solo los campos esenciales primero
            uint8_t src = buffer[0];
            uint16_t seq = (buffer[1] << 8) | buffer[2];

            // === MEJORA 1: IGNORAR PAQUETES PROPIOS INMEDIATAMENTE ===
            // Verificar ANTES de procesar nada (deserializar, mostrar, etc.)
            if (src == MY_ID)
            {
                // Marcar como visto y salir sin procesar nada más
                markPacketAsSeen(src, seq);
                Serial.printf("[MESH] Paquete propio detectado (src=%d, seq=%d). Ignorado inmediatamente.\n", src, seq);
                switchToReceiveMode();
                return;
            }

            // Solo si NO es nuestro paquete, continuar procesando...
            float rssi = radio.getRSSI();

            // Deserializar resto de campos del paquete binario
            uint8_t ttl = buffer[3];

            // Extraer floats
            float lat, lon;
            memcpy(&lat, &buffer[4], 4);
            memcpy(&lon, &buffer[8], 4);
            uint8_t stateCode = buffer[12];

            Serial.printf("[MESH-RX] src=%d, seq=%d, ttl=%d, rssi=%.1f dBm\n",
                          src, seq, ttl, rssi);
            Serial.printf("          lat=%.6f, lon=%.6f, state=%d\n",
                          lat, lon, stateCode);

            // Mostrar en pantalla
            displayText(String("src: " + String(src)).c_str(), 0, 0);
            displayText(String("seq: " + String(seq)).c_str(), 0, 15);
            displayText(String("rssi: " + String(rssi, 1) + " dBm").c_str(), 0, 30);
            delay(1000);
            clearDisplay();

            // === ANTI-LOOP: Verificar si el paquete ya fue visto ===
            if (hasSeenPacket(src, seq))
            {
                Serial.printf("[MESH] Paquete ya visto (src=%d, seq=%d). Ignorando.\n", src, seq);
                switchToReceiveMode();
                return;
            }

            // === GUARDAR COMO VISTO ===
            markPacketAsSeen(src, seq);

            // === STORE & FORWARD: Si ttl > 0, reenviar ===
            bool forwarded = false;
            if (ttl > 0)
            {
                // === MEJORA 3: Verificar separación temporal antes de reenviar ===
                if (!canForward())
                {
                    unsigned long timeSinceLastTx = millis() - lastAnyTxTime;
                    Serial.printf("[MESH] Esperando separación temporal antes de reenviar: %lu ms\n", 
                                  SEPARATION_AFTER_OWN_TX_MS - timeSinceLastTx);
                    switchToReceiveMode();
                    return;
                }
                
                // Decrementar TTL
                buffer[3] = ttl - 1;

                // Reempaquetar y reenviar
                Serial.printf("[MESH] Reenviando paquete (nuevo ttl=%d)...\n", ttl - 1);
                
                // Preparar para TX (actualizar estado esperado)
                switchToTransmitMode();
                
                int16_t txState = radio.startTransmit(buffer, PACKET_SIZE);

                if (txState == RADIOLIB_ERR_NONE)
                {
                    // === MEJORA 3: Actualizar rastreo de reenvío ===
                    setLastTxType(2); // 2 = reenvío
                    
                    Serial.println("[MESH] Reenvío iniciado.");
                    forwarded = true; // Marcar que iniciamos un TX
                    // No volver a RX aquí - esperar a que termine el TX (se manejará en loop principal)
                    return; // Salir - el TX completará y se manejará después
                }
                else
                {
                    Serial.printf("[MESH] Error en reenvío: %d\n", txState);
                    // Si falló, volver a RX
                    switchToReceiveMode();
                }
            }
            else
            {
                // TTL llegó a 0, no reenviar
                Serial.println("[MESH] TTL agotado, no se reenvía el paquete.");
            }

            // Volver al modo recepción (solo si no iniciamos un reenvío)
            if (!forwarded)
            {
                switchToReceiveMode();
            }
        }
        else
        {
            Serial.printf("[MESH-RX] readData error: %d\n", state);
            switchToReceiveMode();
        }
    }
}
