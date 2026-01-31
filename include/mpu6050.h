#ifndef MPU6050_H
#define MPU6050_H

#include <Adafruit_Sensor.h>

// Estructura para almacenar los datos del sensor
struct MPU6050Data {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
};

// Tipos de caida detectada
enum FallType {
  FALL_NONE = 0,
  FALL_FREE_FALL,    // Caida libre (aceleracion muy baja)
  FALL_IMPACT,       // Impacto (aceleracion muy alta)
  FALL_TILT,         // Inclinacion brusca
  FALL_ROTATION      // Rotacion rapida
};

// Estructura con informacion detallada de la caida
struct FallInfo {
  FallType type;           // Tipo de caida detectada
  float accelMagnitude;    // Magnitud de aceleracion actual
  float roll;              // Angulo roll actual
  float pitch;             // Angulo pitch actual
  float gyroMagnitude;     // Magnitud de velocidad angular actual
  float thresholdTriggered; // Valor del umbral que se supero
  float valueTriggered;    // Valor que supero el umbral
};

// Función para inicializar el sensor MPU6050
void initMPU6050();

// Función para leer los datos del sensor
bool readMPU6050(MPU6050Data* data);

// Función para calcular el ángulo de inclinación roll
float calculateRoll(sensors_event_t* accel);

// Función para calcular el ángulo de inclinación pitch
float calculatePitch(sensors_event_t* accel);

// Función para escanear dispositivos I2C (opcional, útil para debug)
void scanI2C();

// Función para mostrar los datos del sensor en display y Serial
void displayMPU6050Data(MPU6050Data* data);

// Algoritmo de deteccion de caidas (simple)
bool detectFalls(MPU6050Data* data);

// Algoritmo de deteccion de caidas con informacion detallada
bool detectFallsWithInfo(MPU6050Data* data, FallInfo* info);

// Funcion para obtener el nombre del tipo de caida
const char* getFallTypeName(FallType type);

#endif

