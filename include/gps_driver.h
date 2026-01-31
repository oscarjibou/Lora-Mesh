#ifndef GPS_DRIVER_H
#define GPS_DRIVER_H

#include <Arduino.h>
#include <TinyGPS++.h>

// Pines del GPS NEO-6M
#define GPS_TX_PIN 2
#define GPS_RX_PIN 3
#define GPS_BAUD 9600

// Valor cuando no hay fix GPS válido
#define GPS_NO_FIX_VALUE -1.0f

class GpsDriver {
public:
    void begin(uint8_t rxPin, uint8_t txPin, uint32_t baud);
    bool update();
    bool hasFix();
    double getLatitude();
    double getLongitude();
    double getAltitude();
    uint8_t getHour();
    uint8_t getMinute();
    uint8_t getSecond();
    uint32_t getAge();
    uint32_t getCharsProcessed();
    uint32_t getSatellites();
    uint32_t getHdop();
    uint32_t getSentencesWithFix();
    uint32_t getFailedChecksum();

private:
    TinyGPSPlus _gps;
    uint8_t _rxPin;
    uint8_t _txPin;
    uint32_t _baud;
    bool _begun;
};

// Instancia global del GPS (declarada en main.cpp)
extern GpsDriver gps;

#endif // GPS_DRIVER_H
