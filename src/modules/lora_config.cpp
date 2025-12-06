#include "lora_config.h"
#include <string.h>

// --- Instancias RadioLib ---
SPIClass spiLoRa(HSPI); // bus SPI exclusivo
Module lora(PIN_SS, PIN_DIO1, PIN_RST, PIN_BUSY, spiLoRa);
SX1262 radio(&lora);

// --- Flags compartidos para interrupciones (mesh) ---
volatile bool txDone = false;
volatile bool paqueteRecibido = false;

// --- Estado de la radio para determinar tipo de interrupción ---
volatile bool expectingTX = false;
volatile bool expectingRX = true; // Por defecto en RX

// --- Rastreo del último tipo de TX para separación temporal ---
// 0 = ninguna, 1 = propio, 2 = reenvío
volatile uint8_t lastTxType = 0;
volatile unsigned long lastAnyTxTime = 0;

// --- Callback unificado para interrupciones TX y RX ---
void onRadioIrq(void)
{
    // RadioLib llama a este callback cuando hay una interrupción en DIO1
    // Verificamos qué tipo de interrupción es basándonos en el estado esperado

    // Si estábamos esperando TX, es TX-done
    if (expectingTX)
    {
        txDone = true;
        expectingTX = false;
        expectingRX = true;
    }
    // Si estábamos esperando RX, es RX-done
    else if (expectingRX)
    {
        paqueteRecibido = true;
    }
}

// --- Inicialización unificada de radio para modo mesh ---
void meshRadioSetup()
{
    // 1) Arrancar SPI en los pines físicos correctos
    spiLoRa.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);

    // 2) Inicializar el SX1262 directamente en nuestra banda
    int16_t state = radio.begin(868.0F); // EU868
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.printf("[MESH] begin() falló: %d\n", state);
        while (true)
            delay(1);
    }

    // 3) Parámetros LoRa
    radio.setBandwidth(125.0F);
    radio.setSpreadingFactor(12); // SF12 para mayor alcance
    radio.setCodingRate(5);
    radio.setSyncWord(0x12);
    radio.setCRC(true);
    radio.setOutputPower(14); // 14 dBm (~25 mW)

    // 4) Registrar callback unificado para TX y RX
    radio.setDio1Action(onRadioIrq);

    // 5) Iniciar en modo recepción por defecto
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println("[MESH] Radio inicializada - Modo RECEPCIÓN");
    }
    else
    {
        Serial.printf("[MESH] startReceive() error: %d\n", state);
    }

    // 6) Inicializar buffer de paquetes vistos (anti-loop)
    memset(seenPackets, 0, sizeof(seenPackets));
    seenPacketsIndex = 0;
}

// --- Cambiar a modo recepción ---
void switchToReceiveMode()
{
    expectingRX = true;
    expectingTX = false;
    int16_t state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.printf("[MESH] Error al cambiar a RX: %d\n", state);
    }
}

// --- Cambiar a modo transmisión (preparar para TX) ---
void switchToTransmitMode()
{
    // La radio se pone en modo TX automáticamente cuando llamamos startTransmit()
    // Esta función solo asegura que estamos fuera de RX
    expectingTX = true;
    expectingRX = false;
    radio.standby();
}

// --- Verificar si la radio está lista para transmitir ---
bool isRadioReadyForTX()
{
    // No está lista si:
    // 1. Está transmitiendo actualmente (expectingTX = true)
    // 2. Acaba de llegar un paquete que necesita procesarse (paqueteRecibido = true)
    // 3. Hay un TX completado que aún no se procesó (txDone = true)
    return !expectingTX && !paqueteRecibido && !txDone;
}

// --- Funciones para rastrear tipo de TX y separación temporal ---
void setLastTxType(uint8_t type)
{
    lastTxType = type;
    lastAnyTxTime = millis();
}

bool canSendOwnTx()
{
    // Puede enviar TX propio si:
    // 1. No hay TX reciente O
    // 2. El último TX fue propio (puede repetir) O
    // 3. Ha pasado suficiente tiempo desde el último reenvío
    if (lastAnyTxTime == 0 || lastTxType == 1)
    {
        return true; // No hay restricción o el último fue propio
    }
    
    unsigned long timeSinceLastTx = millis() - lastAnyTxTime;
    if (lastTxType == 2)
    {
        // Último fue reenvío, verificar separación
        return timeSinceLastTx >= SEPARATION_AFTER_FORWARD_MS;
    }
    
    return true; // Por defecto permitir
}

bool canForward()
{
    // Puede reenviar si:
    // 1. No hay TX reciente O
    // 2. El último TX fue reenvío (puede repetir) O
    // 3. Ha pasado suficiente tiempo desde el último TX propio
    if (lastAnyTxTime == 0 || lastTxType == 2)
    {
        return true; // No hay restricción o el último fue reenvío
    }
    
    unsigned long timeSinceLastTx = millis() - lastAnyTxTime;
    if (lastTxType == 1)
    {
        // Último fue TX propio, verificar separación
        return timeSinceLastTx >= SEPARATION_AFTER_OWN_TX_MS;
    }
    
    return true; // Por defecto permitir
}
