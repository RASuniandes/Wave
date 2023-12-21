#ifndef SENSORS_H
#define SENSORS_H

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP280.h>

class Sensors {
private:
  Adafruit_BMP280 bmp280;
  Adafruit_MPU6050 mpu6050;
  long tiempo_prev; // Variable para almacenar el tiempo previo
  float dt; // Variable para almacenar el cambio en el tiempo entre lecturas anteriores
  float temperature; // Variable para almacenar la temperatura
  sensors_event_t accelEvent; // Estructura para almacenar datos de aceleración
  sensors_event_t gyroEvent; // Estructura para almacenar datos de giro
  sensors_event_t tempEvent; // Estructura para almacenar datos de aceleración

  float ang_x_prev; // Variable para almacenar el ángulo X previo
  float ang_y_prev; // Variable para almacenar el ángulo Y previo
  float ang_z_prev; // Variable para almacenar el ángulo Z previo
  bool bmpWorking; // Variable para verificar si el BMP280 está funcionando
  bool mpuWorking; // Variable para verificar si el MPU6050 está funcionando

  
  // ... otros métodos ...
  
  // Variables para offsets y calibración
  int ax_offset;
  int ay_offset;
  int az_offset;
  int gx_offset;
  int gy_offset;
  int gz_offset;
  bool calibrationInProgress; 




public:
  Sensors();
  bool initialize();
  float getTemperature();
  float getPressure();
  float getAltitude();
  void getOrientation(float &yaw, float &pitch, float &roll);
  void calibrateMPU();
  void getAcceleration(float &accelX, float &accelY, float &accelZ);
  void getRotation(float &gyroX, float &gyroY, float &gyroZ);
  bool isBmpWorking() {
    return bmpWorking;
  }

  bool isMpuWorking() {
    return mpuWorking;
  }

private:
  void disableBMP(); // Método privado para desactivar el BMP280
  void disableMPU(); // Método privado para desactivar el MPU6050
};

#endif // SENSORS_H