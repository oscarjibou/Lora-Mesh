#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include "mpu6050.h"
#include "display.h"

#ifndef PI
#define PI 3.14159265359
#endif

// Pines para el MPU6050
#define SDA_PIN 4
#define SCL_PIN 5

// Umbrales configurables para deteccion de caidas
#define FREE_FALL_THRESHOLD 2.0      // m/s2 (magnitud minima para caida libre)
#define IMPACT_THRESHOLD 25.0        // m/s2 (magnitud maxima para impacto)
#define TILT_THRESHOLD 45.0          // grados (inclinacion brusca)
#define ROTATION_THRESHOLD 2.0       // rad/s (rotacion rapida)

// Instancia global (static) del sensor MPU6050
static Adafruit_MPU6050 mpu;

// Función para escanear dispositivos I2C en el bus del MPU6050
static void scanI2CBus(TwoWire &bus) {
  Serial.println("\n--- Escaneo I2C ---");
  int n = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    bus.beginTransmission(addr);
    uint8_t err = bus.endTransmission();
    if (err == 0) {
      Serial.printf("I2C encontrado: 0x%02X\n", addr);
      n++;
    }
    delay(2);
  }
  if (!n)
    Serial.println("No se encontraron dispositivos I2C");
}

// Implementación de la función para inicializar el sensor MPU6050
void initMPU6050() {
  // Inicializa Wire1 para el MPU6050 en pines diferentes
  Wire1.begin(SDA_PIN, SCL_PIN);
  Wire1.setClock(100000);

  scanI2CBus(Wire1);

  if (!mpu.begin(0x68, &Wire1) && !mpu.begin(0x69, &Wire1)) {
    Serial.println("ERROR: Failed to find MPU6050 chip");
    while (1)
      delay(10);
  }

  Serial.println("MPU6050 OK!");
}

// Implementación de la función para leer los datos del sensor
bool readMPU6050(MPU6050Data* data) {
  if (data == nullptr) {
    return false;
  }
  
  // Obtener eventos de sensores
  mpu.getEvent(&data->accel, &data->gyro, &data->temp);
  
  return true;
}

// Implementación de la función para calcular el ángulo de inclinación roll
float calculateRoll(sensors_event_t* accel) {
  if (accel == nullptr) {
    return 0.0;
  }
  return atan2(accel->acceleration.y, accel->acceleration.z) * 180.0 / PI;
}

// Implementación de la función para calcular el ángulo de inclinación pitch
float calculatePitch(sensors_event_t* accel) {
  if (accel == nullptr) {
    return 0.0;
  }
  return atan2(-accel->acceleration.x,
               sqrt(accel->acceleration.y * accel->acceleration.y +
                    accel->acceleration.z * accel->acceleration.z)) * 180.0 / PI;
}

// Función pública para escanear I2C (útil para debug desde main)
void scanI2C() {
  scanI2CBus(Wire1);
}

// Implementación de la función para mostrar los datos del sensor en display y Serial
void displayMPU6050Data(MPU6050Data* data) {
  if (data == nullptr) {
    return;
  }

  clearDisplay();

  // Mostrar datos del acelerómetro en el display
  String textXAccel = "Ax: " + String(data->accel.acceleration.x, 2);
  displayText(textXAccel.c_str(), 0, 0);
  String textYAccel = "Ay: " + String(data->accel.acceleration.y, 2);
  displayText(textYAccel.c_str(), 0, 20);
  String textZAccel = "Az: " + String(data->accel.acceleration.z, 2);
  displayText(textZAccel.c_str(), 0, 40);

  // Mostrar datos del giroscopio en el display
  String textXGyro = "Gx: " + String(data->gyro.gyro.x, 2);
  displayText(textXGyro.c_str(), 40, 0);
  String textYGyro = "Gy: " + String(data->gyro.gyro.y, 2);
  displayText(textYGyro.c_str(), 40, 20);
  String textZGyro = "Gz: " + String(data->gyro.gyro.z, 2);
  displayText(textZGyro.c_str(), 40, 40);

  // Mostrar temperatura en el display
  String textTemp = "T: " + String(data->temp.temperature, 2);
  displayText(textTemp.c_str(), 85, 0);

  // Calcular y mostrar ángulos de inclinación
  float roll = calculateRoll(&data->accel);
  float pitch = calculatePitch(&data->accel);

  String textRoll = "R: " + String(roll, 2);
  displayText(textRoll.c_str(), 85, 20);
  String textPitch = "P: " + String(pitch, 2);
  displayText(textPitch.c_str(), 85, 40);

  Serial.println("Ángulos de inclinación:");
  Serial.printf("Roll:  %.2f°\n", roll);
  Serial.printf("Pitch: %.2f°\n", pitch);

  Serial.println("---------------------------------\n");
}

// Funcion auxiliar para calcular magnitud de aceleracion
static float calculateAccelMagnitude(sensors_event_t* accel) {
  if (accel == nullptr) {
    return 0.0;
  }
  return sqrt(accel->acceleration.x * accel->acceleration.x +
              accel->acceleration.y * accel->acceleration.y +
              accel->acceleration.z * accel->acceleration.z);
}

// Funcion auxiliar para calcular magnitud de velocidad angular
static float calculateGyroMagnitude(sensors_event_t* gyro) {
  if (gyro == nullptr) {
    return 0.0;
  }
  return sqrt(gyro->gyro.x * gyro->gyro.x +
              gyro->gyro.y * gyro->gyro.y +
              gyro->gyro.z * gyro->gyro.z);
}

// Funcion para obtener el nombre del tipo de caida
const char* getFallTypeName(FallType type) {
  switch (type) {
    case FALL_FREE_FALL: return "CAIDA LIBRE";
    case FALL_IMPACT:    return "IMPACTO";
    case FALL_TILT:      return "INCLINACION";
    case FALL_ROTATION:  return "ROTACION";
    default:             return "NINGUNA";
  }
}

// Algoritmo de deteccion de caidas con informacion detallada
bool detectFallsWithInfo(MPU6050Data* data, FallInfo* info) {
  // Validacion
  if (data == nullptr || info == nullptr) {
    return false;
  }
  
  // Inicializar info
  info->type = FALL_NONE;
  info->thresholdTriggered = 0;
  info->valueTriggered = 0;
  
  // 1. Calcular magnitud de aceleracion
  info->accelMagnitude = calculateAccelMagnitude(&data->accel);
  
  // 2. Detectar caida libre (magnitud muy baja)
  if (info->accelMagnitude < FREE_FALL_THRESHOLD) {
    info->type = FALL_FREE_FALL;
    info->thresholdTriggered = FREE_FALL_THRESHOLD;
    info->valueTriggered = info->accelMagnitude;
    return true;
  }
  
  // 3. Detectar impacto (magnitud muy alta)
  if (info->accelMagnitude > IMPACT_THRESHOLD) {
    info->type = FALL_IMPACT;
    info->thresholdTriggered = IMPACT_THRESHOLD;
    info->valueTriggered = info->accelMagnitude;
    return true;
  }
  
  // 4. Calcular angulos de inclinacion
  info->roll = calculateRoll(&data->accel);
  info->pitch = calculatePitch(&data->accel);
  
  // // 5. Detectar inclinacion brusca
  // if (fabs(info->roll) > TILT_THRESHOLD) {
  //   info->type = FALL_TILT;
  //   info->thresholdTriggered = TILT_THRESHOLD;
  //   info->valueTriggered = info->roll;
  //   return true;
  // }
  // if (fabs(info->pitch) > TILT_THRESHOLD) {
  //   info->type = FALL_TILT;
  //   info->thresholdTriggered = TILT_THRESHOLD;
  //   info->valueTriggered = info->pitch;
  //   return true;
  // }
  
  // // 6. Calcular magnitud de velocidad angular
  // info->gyroMagnitude = calculateGyroMagnitude(&data->gyro);
  
  // // 7. Detectar rotacion rapida
  // if (info->gyroMagnitude > ROTATION_THRESHOLD) {
  //   info->type = FALL_ROTATION;
  //   info->thresholdTriggered = ROTATION_THRESHOLD;
  //   info->valueTriggered = info->gyroMagnitude;
  //   return true;
  // }
  
  return false;
}

// Algoritmo de deteccion de caidas (simple)
bool detectFalls(MPU6050Data* data) {
  FallInfo info;
  return detectFallsWithInfo(data, &info);
}
