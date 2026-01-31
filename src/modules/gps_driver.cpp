#include "gps_driver.h"

void GpsDriver::begin(uint8_t rxPin, uint8_t txPin, uint32_t baud) {
    _rxPin = rxPin;
    _txPin = txPin;
    _baud = baud;
    Serial2.begin(_baud, SERIAL_8N1, _rxPin, _txPin);
    _begun = true;
}

bool GpsDriver::update() {
    if (!_begun) return false;
    while (Serial2.available() > 0) {
        _gps.encode(Serial2.read());
    }
    return _gps.location.isValid() && _gps.location.isUpdated();
}

bool GpsDriver::hasFix() {
    return _gps.location.isValid();
}

double GpsDriver::getLatitude() {
    return _gps.location.lat();
}

double GpsDriver::getLongitude() {
    return _gps.location.lng();
}

double GpsDriver::getAltitude() {
    return _gps.altitude.isValid() ? _gps.altitude.meters() : 0.0;
}

uint8_t GpsDriver::getHour() {
    return _gps.time.isValid() ? _gps.time.hour() : 0;
}

uint8_t GpsDriver::getMinute() {
    return _gps.time.isValid() ? _gps.time.minute() : 0;
}

uint8_t GpsDriver::getSecond() {
    return _gps.time.isValid() ? _gps.time.second() : 0;
}

uint32_t GpsDriver::getAge() {
    return _gps.location.age();
}

uint32_t GpsDriver::getCharsProcessed() {
    return _gps.charsProcessed();
}

uint32_t GpsDriver::getSatellites() {
    return _gps.satellites.value();
}

uint32_t GpsDriver::getHdop() {
    return _gps.hdop.value();
}

uint32_t GpsDriver::getSentencesWithFix() {
    return _gps.sentencesWithFix();
}

uint32_t GpsDriver::getFailedChecksum() {
    return _gps.failedChecksum();
}
