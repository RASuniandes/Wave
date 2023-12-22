#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include "BNO055_support.h"

Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;
struct bno055_t myBNO;
struct bno055_euler myEulerData;

boolean bmpInitialized = false;
boolean mpuInitialized = false;


// Parámetros del filtro de Kalman para Roll
float angle_roll = 0.0; // Ángulo Roll calculado por el filtro de Kalman
float bias_roll = 0.0;  // Sesgo del giroscopio para Roll calculado por el filtro de Kalman
float P_roll[2][2] = {{0, 0}, {0, 0}};

float devicesFound = 0.0;

float temperature = 0.0;
float pressure = 0.0;
float altitude = 0.0;

float aX = 0.0;
float aY = 0.0;
float aZ = 0.0;
float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;

float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;

float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;
float dt = 0.0;
long tiempo_prev = 0;

unsigned char accelCalibStatus = 0;
unsigned char magCalibStatus = 0;
unsigned char gyroCalibStatus = 0;
unsigned char sysCalibStatus = 0;
unsigned long lastTime = 0;

// Parámetros del filtro de Kalman
float Q_angle = 0.001;
float Q_bias = 0.003;
float R_measure = 0.03;

float angle_y = 0.0; // Ángulo Y calculado por el filtro de Kalman
float angle_x = 0.0; // Ángulo X calculado por el filtro de Kalman
float bias_y = 0.0;  // Sesgo del giroscopio Y calculado por el filtro de Kalman
float bias_x = 0.0;  // Sesgo del giroscopio X calculado por el filtro de Kalman

float P_y[2][2] = {{0, 0}, {0, 0}};
float P_x[2][2] = {{0, 0}, {0, 0}};

#define BNO055_SAMPLERATE_DELAY_MS (1000)

void Bno() {
  if (millis() - lastTime >= BNO055_SAMPLERATE_DELAY_MS) {
    lastTime = millis();
    bno055_read_euler_hrp(&myEulerData);
    Serial.print(F("Orientation: "));
    Serial.print(360 - (float(myEulerData.h) / 16.00));
    Serial.print(F(", "));
    Serial.print(360 - (float(myEulerData.p) / 16.00));
    Serial.print(F(", "));
    Serial.print(360 - (float(myEulerData.r) / 16.00));
    Serial.println(F(""));
    bno055_get_accelcalib_status(&accelCalibStatus);
    bno055_get_gyrocalib_status(&gyroCalibStatus);
    bno055_get_syscalib_status(&sysCalibStatus);
    bno055_get_magcalib_status(&magCalibStatus);
    Serial.print(F("Calibration: "));
    Serial.print(sysCalibStatus, DEC);
    Serial.print(F(", "));
    Serial.print(gyroCalibStatus, DEC);
    Serial.print(F(", "));
    Serial.print(accelCalibStatus, DEC);
    Serial.print(F(", "));
    Serial.print(magCalibStatus, DEC);
    Serial.println(F(""));
  }
}

void initSensors() {
  bmpInitialized = bmp.begin(0x76);
  mpuInitialized = mpu.begin(0x68);
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
}

void readBMP280Data() {
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure() / 100.0F;
  altitude = bmp.readAltitude(1013.25);
}

void KalmanFilter(float newAngle, float newRate, float *angle, float *bias, float P[2][2]) {
  float S, K[2], y;
  float dt = (millis() - tiempo_prev) / 1000.0;

  *angle += dt * (newRate - *bias);
  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  y = newAngle - *angle;
  S = P[0][0] + R_measure;
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  *angle += K[0] * y;
  *bias += K[1] * y;

  P[0][0] -= K[0] * P[0][0];
  P[0][1] -= K[0] * P[0][1];
  P[1][0] -= K[1] * P[0][0];
  P[1][1] -= K[1] * P[0][1];
}

void readMPU6050Data() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  aX = a.acceleration.x;
  aY = a.acceleration.y;
  aZ = a.acceleration.z;

  accelX = aX * 9.81; // Convertir de m/s^2 a g 
  accelY = aY * 9.81; // Convertir de m/s^2 a g
  accelZ = aZ * 9.81; // Convertir de m/s^2 a g

  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;

  // Filtro de Kalman para Yaw (Z)
  float angleAccelZ = atan2(aY, aZ) * 180 / PI;
  KalmanFilter(angleAccelZ, gyroZ, &angle_y, &bias_y, P_y);
  yaw = angle_y;

  // Filtro de Kalman para Pitch (Y)
  float angleAccelY = atan2(-aX, sqrt(aY * aY + aZ * aZ)) * 180 / PI;
  KalmanFilter(angleAccelY, gyroY, &angle_x, &bias_x, P_x);
  pitch = angle_x;

  // Filtro de Kalman para Roll (X)
  float angleAccelX = atan2(aY, aZ) * 180 / PI; // Calcular ángulo con acelerómetro
  float rate_roll = gyroX - bias_roll;
    
  // Predicción para Roll
  angle_roll += dt * rate_roll;
  P_roll[0][0] += dt * (dt*P_roll[1][1] - P_roll[0][1] - P_roll[1][0] + Q_angle);
  P_roll[0][1] -= dt * P_roll[1][1];
  P_roll[1][0] -= dt * P_roll[1][1];
  P_roll[1][1] += Q_bias * dt;

  // Actualización para Roll
  float y_roll = angleAccelX - angle_roll;
  float S_roll = P_roll[0][0] + R_measure;
  float K_roll[2] = { P_roll[0][0] / S_roll, P_roll[1][0] / S_roll };

  angle_roll += K_roll[0] * y_roll;
  bias_roll += K_roll[1] * y_roll;

  P_roll[0][0] -= K_roll[0] * P_roll[0][0];
  P_roll[0][1] -= K_roll[0] * P_roll[0][1];
  P_roll[1][0] -= K_roll[1] * P_roll[0][0];
  P_roll[1][1] -= K_roll[1] * P_roll[0][1];

  roll = angle_roll;

  // Actualización del tiempo para el próximo cálculo
  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();
}

void scanI2C() {
  Serial.println("Scanning I2C Addresses");
  uint8_t cnt = 0;
  Wire.setClock(100000);

  for (uint8_t i = 0; i < 128; i++) {
    Wire.beginTransmission(i);
    uint8_t ec = Wire.endTransmission(true);

    if (ec == 0) {
      if (i < 16) Serial.print('0');
      Serial.print(i, HEX);
      cnt++;
    } else {
      Serial.print("..");
    }
    
    Serial.print(' ');
    if ((i & 0x0F) == 0x0F) Serial.println();
  }

  Serial.print("Scan Completed, ");
  Serial.print(cnt);
  Serial.println(" I2C Devices found.");

  devicesFound = cnt;

  if (devicesFound == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.println("I2C devices found.");
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  scanI2C();
  Serial.println("Setup completed.");
  delay(1000);
  initSensors();
}

void loop() {
  readBMP280Data();
  Serial.print("Temperatura (C): ");
  Serial.println(temperature);
  Serial.print("Presión (hPa): ");
  Serial.println(pressure);
  Serial.print("Altitud (mSA) ");
  Serial.println(altitude);
  
  readMPU6050Data();
  Serial.print("Acelerómetro (X, Y, Z): ");
  Serial.print(aX);
  Serial.print(", ");
  Serial.print(aY);
  Serial.print(", ");
  Serial.println(aZ);
  Serial.print("Yaw: ");
  Serial.print(yaw);
  Serial.print(", Pitch: ");
  Serial.print(pitch);
  Serial.print(", Roll: ");
  Serial.println(roll);
  
  delay(1000); // Pausa de 1 segundo entre lecturas
}
