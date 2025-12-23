#ifndef DISPLAY_H
#define DISPLAY_H

#define SDA_OLED 17
#define SCL_OLED 18

// Función para inicializar y mostrar algo en la pantalla
void initDisplay();

// Función para mostrar un texto en la pantalla
void displayText(const char *text, int x = 0, int y = 0);

void displayTextLarge(const char *text, int x = 0, int y = 0);

void clearDisplay();

#endif
