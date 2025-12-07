#ifndef LORA_CONFIG_H
#define LORA_CONFIG_H

#include <Arduino.h>
#include <RadioLib.h>

// --- Pines dedicados al SX1262 en el Heltec LoRa 32 V3 ---
#define PIN_SCK 9
#define PIN_MISO 11
#define PIN_MOSI 10
#define PIN_SS 8 // NSS / CS
#define PIN_RST 12
#define PIN_BUSY 13
#define PIN_DIO1 14 // IRQ de RX/TX-done

// --- Instancias RadioLib ---
extern SPIClass spiLoRa;
extern Module lora;
extern SX1262 radio;

// --- PROTOCOLO BINARIO (MESH) ---
// ID del nodo actual (cambiar según necesidad)
#define MY_ID 1
#define GATEWAY_ID 1
#define BROADCAST_ID 255

// Tamaño del paquete binario: 14 bytes
#define PACKET_SIZE 14

// --- CONFIGURACIÓN DE DESINCRONIZACIÓN (anti-colisiones) ---
// Jitter máximo en milisegundos (delay aleatorio antes de cada TX)
#define MAX_JITTER_MS 300

// Offset por ID en milisegundos (cada nodo espera este tiempo inicial basado en su ID)
#define OFFSET_PER_ID_MS 1000

// Separación temporal entre reenvío y envío propio (milisegundos)
// Tiempo mínimo que debe pasar después de reenviar antes de permitir TX propio
#define SEPARATION_AFTER_FORWARD_MS 500

// Separación temporal entre envío propio y reenvío (milisegundos)
// Tiempo mínimo que debe pasar después de TX propio antes de permitir reenvío
#define SEPARATION_AFTER_OWN_TX_MS 500

// Estructura para anti-loop (últimos paquetes vistos)
#define MAX_SEEN_PACKETS 10
struct SeenPacket
{
    uint8_t src;
    uint16_t seq;
};

// Buffer circular de paquetes vistos
extern SeenPacket seenPackets[MAX_SEEN_PACKETS];
extern uint8_t seenPacketsIndex;

// --- FUNCIONES MESH UNIFICADAS ---
// Flags compartidos para interrupciones (declarados extern, definidos en módulos)
extern volatile bool txDone;
extern volatile bool paqueteRecibido;

// Inicialización unificada de radio para modo mesh
void meshRadioSetup();

// Callback unificado para manejar interrupciones TX y RX
void onRadioIrq(void);

// Funciones auxiliares para cambio de modo
void switchToReceiveMode();
void switchToTransmitMode();

// Verificar si la radio está lista para transmitir (no está transmitiendo ni procesando RX)
bool isRadioReadyForTX();

// Funciones para rastrear tipo de TX y separación temporal
// 0 = ninguna, 1 = propio, 2 = reenvío
extern volatile uint8_t lastTxType;
extern volatile unsigned long lastAnyTxTime;
void setLastTxType(uint8_t type); // 1=propio, 2=reenvío
bool canSendOwnTx();              // Verifica si puede enviar TX propio (separación temporal)
bool canForward();                // Verifica si puede reenviar (separación temporal)

#endif // LORA_CONFIG_H
