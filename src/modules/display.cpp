#include <Arduino.h>
#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include "display.h"

// Pines de la Heltec WiFi LoRa 32 V3 (ESP32-S3)
#define SDA_OLED 17
#define SCL_OLED 18
#define RST_OLED 21
#define Vext 36 // En la V3, RST y Vext pueden compartir pin

// Función para encender Vext (alimentación de la OLED)
static void vextON()
{
    pinMode(Vext, OUTPUT);
    // En la Heltec V3, poner LOW en Vext enciende la OLED
    digitalWrite(Vext, LOW);
}

// Instancia global (static) del display (SSD1306, 128x64)
static SSD1306Wire display(
    0x3c,   // Dirección I2C de la OLED
    400000, // Frecuencia del bus I2C
    SDA_OLED,
    SCL_OLED,
    GEOMETRY_128_64,
    RST_OLED);

// Implementación de la función para inicializar la pantalla
void initDisplay()
{
    // Enciende la alimentación del OLED
    vextON();
    delay(100);

    // Inicia I2C en los pines de la V3
    Wire.begin(SDA_OLED, SCL_OLED);

    // Inicializa la pantalla
    display.init();
    display.clear();
    display.setFont(ArialMT_Plain_16);

    // Muestra un texto
    display.drawString(0, 0, "Starting...");
    display.display();
    delay(2000);

    // limpiar la pantalla
    display.clear();
    display.display();

    Serial.println("Display initialized");
}

void displayText(const char *text, int x, int y)
{
    display.drawString(x, y, text);
    display.display();
}

void clearDisplay()
{
    display.clear();
    display.display();
}
