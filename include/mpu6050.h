#ifndef MPU6050_H
#define MPU6050_H

#include <Adafruit_Sensor.h>

// Estructura para almacenar los datos del sensor
struct MPU6050Data {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
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

#endif

