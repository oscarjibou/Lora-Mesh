#ifndef RECEIVER_H
#define RECEIVER_H

#include <Arduino.h>
#include <RadioLib.h>

// Inicializa el módulo receptor LoRa
void receiverSetup();

// Procesa la recepción (debe llamarse periódicamente)
void receiverLoop();

// [NUEVO] Escucha, procesa y reenvía paquetes binarios (mesh)
void listenAndForward();

// Funciones para gestión de paquetes vistos (anti-loop)
bool hasSeenPacket(uint8_t src, uint16_t seq);
void markPacketAsSeen(uint8_t src, uint16_t seq);

#endif // RECEIVER_H