#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include <Arduino.h>
#include <RadioLib.h>

// Inicializa el módulo transmisor LoRa
void transmitterSetup();

// Procesa la transmisión (debe llamarse periódicamente)
void transmitterLoop();

// Envía un mensaje por el transmisor LoRa (formato antiguo string)
bool sendMessage(String message);

// [NUEVO] Envía el estado del nodo en protocolo binario (18 bytes)
void sendMyStatus();

#endif // TRANSMITTER_H