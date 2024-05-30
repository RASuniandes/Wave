#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include "BNO055_support.h"
#include <Adafruit_SSD1306.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <TinyGPSPlus.h>
#include <Adafruit_PWMServoDriver.h>
#include "telemetria/FlySky.h"
#include <SPI.h>
#include <RF24.h>
#include "NAVEGACION\control.h"
#include <vector>
#include <iostream>
#include "HMC5883L.h"
#include "sensorica\PressureSensor.h"
#include <SPI.h>
#include <SD.h>

// Pines para el lector de tarjetas SD
#define MOSI_PIN 35
#define SCK_PIN 36
#define MISO_PIN 37
#define CS_PIN 38


File myFile;
bool fileCreated = false;
String fileName;


//-----------Pressure Sensor------------------------------
// Pin del sensor de presión (ADC1 CH0 en ESP32-S3)
int sensorPin = 15;
// Constantes de conversión del sensor de presión
float V_S = 5.0; // Voltaje de suministro
float sensitivity = 0.09; // Sensibilidad del sensor (0.09 V/kPa)
float offset = 0.04; // Offset del sensor (0.04 V)
 float RHO_AIR=1.225;
PressureSensor pressureSensor( sensorPin, V_S, sensitivity, offset);

//========================================================================================

float kpYaw = 1;
float kdYaw = 0.1;
float kiYaw = 0.01;

float kpPitch = 1;
float kdPitch = 0.1;
float kiPitch = 0.01;

float kpRoll = 8;
float kdRoll = 0.5;
float kiRoll = 0.01;

float toErrorYaw=0;
float priErrorYaw=0;
float toErrorPitch=0;
float priErrorPitch=0;
float toErrorRoll=0;
float priErrorRoll=0;

float PosicionDeseadaYaw = 0;
float PosicionDeseadaPitch = 0;
float PosicionDeseadaRoll = 0;

//====================================



float kp = 0.1;
float kd = 0.1;


float VelocidadActual = 0;
float distanciaAuxiliar = 100;
float cte_saturacion = 10;
float condicionActualizacion = 0;
float toError=0;
float priError=0;

float tiempo=0;
float compass_value=0;
Control Controlador(distanciaAuxiliar, kp, kd, cte_saturacion, condicionActualizacion);


float PosicionDeseadaGrados=80;

float inputCoords[Control::MAX_COORDINATES][2] = {{4.653453, -74.093492}, 
                          {4.691751, -74.124330}, 
                          {4.635802, -74.127502},
                          {4.705188, -74.037882},
                          {4.648194, -74.101032},
                          {4.686006, -74.074529},
                          {4.591923, -74.123288}
                          };
float outputCoords[Control::MAX_COORDINATES][2];
int numCoords = 8;  // Asegúrate de que esta variable refleje la cantidad real de coordenadas proporcionadas




#define CE_PIN   21
#define CSN_PIN 16

RF24 radio(CE_PIN, CSN_PIN);

const byte address[6] = "00001";

// Compass
HMC5883L compass;
float compass_degrees = 0.0;



// GPS declaracion
TinyGPSPlus gps;
 // Pin digital del MPS20N0040D (D15)
const float seaLevelPressure = 101.325;  // Presión atmosférica al nivel del mar en kPa


// GPS values 
 float Latitud=0;
 float Longitud=0;

const int TX2 = 11; // Pines de transmisión y recepción del GPS
const int RX2 = 10;
bool gpsDetected = false;

// Pines pitot
const int DOUT_Pin = 15;   //sensor data pin
const int SCK_Pin  = 34;   //sensor clock pin

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;

struct bno055_t myBNO;
struct bno055_euler myEulerData;
struct bno055_mag magData;

float mag_x, mag_y, mag_z;

boolean bmpInitialized = false;
boolean mpuInitialized = false;


unsigned long previousMillis = 0;
unsigned long previousMillis2 = 0;
const long interval1 = 10; // Intervalo de actualización de 10 ms
const long interval2 = 1000; // Intervalo de actualización de 10 ms
// Parámetros del filtro de Kalman para Roll
float angle_roll = 0.0; // Ángulo Roll calculado por el filtro de Kalman
float bias_roll = 0.0;  // Sesgo del giroscopio para Roll calculado por el filtro de Kalman
float P_roll[2][2] = {{0, 0}, {0, 0}};

float devicesFound = 0.0;

float temperature = 0.0;
float pressure = 0.0;
float altitude = 0.0;

float rawTemperature = 0.0;
float rawPressure = 0.0;
float rawAltitude = 0.0;



float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;

// MPU 6050 VALUES 
float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;
float dt = 0.0;
long tiempo_prev = 0;

float yaw_raw_mpu = 0.0;
float pitch_raw_mpu = 0.0;
float roll_raw_mpu = 0.0;


float aX = 0.0;
float aY = 0.0;
float aZ = 0.0;

float accelX = 0.0;
float accelY = 0.0;
float accelZ = 0.0;

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

// BNO055 VALUES

float yawValue = 0.0;
float rollValue = 0.0;
float pitchValue = 0.0;

// boleano 



boolean conexion=true;



unsigned char accelCalibStatus = 0;
unsigned char magCalibStatus = 0;
unsigned char gyroCalibStatus = 0;
unsigned char sysCalibStatus = 0;
unsigned long lastTime = 0;

// EMA

const float alpha = 0.1; // Factor de suavizado


#define BNO055_SAMPLERATE_DELAY_MS (100)

//SD values
const int chipSelect = 38; 

// Canales radio control 
#define CH1 7
#define CH2 6
#define CH3 5
#define CH4 4
#define CH5 3
#define CH6 2



// Configurar receptor

// receptor y servos
FlySky flySky(CH1, CH2, CH3, CH4, CH5,CH6);
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

#define SER0_ALERONES 0 
#define SER1_ELEVADORES  1   //Servo Motor 0 on connector 0
 //Servo Motor 1 on connector 12

#define SER2_MOTOR  2   //Servo Motor 0 on connector 0
#define SER3_TIMON  3


const int servoMin = 150;  // Valor mínimo del pulso PWM para el servo (ajusta según sea necesario)
const int servoMax = 600;  // Valor máximo del pulso PWM para el servo (ajusta según sea necesario)


unsigned int pos0=172; // ancho de pulso en cuentas para pocicion 0°
unsigned int pos180=565; // ancho de pulso en cuentas para la pocicion 180°

#define MIN_PULSE_WIDTH 600
#define MAX_PULSE_WIDTH 2600
#define FREQUENCY 50

int ch1Value = 0;
int ch2Value = 0;
int ch3Value = 0;
int ch4Value = 0;
int ch5Value = 0;
int ch6Value = 0;

// values_default_chanels
int min_limit_c1 = 20;
int max_limit_c1 = 140;
int default_value_c1 = 80; // alas (roll)

int min_limit_c2 = 10;
int max_limit_c2 = 160;
int default_value_c2 = 85; // Cola del avion (pitch)

int min_limit_c3 = 30;
int max_limit_c3 = 150;
int default_value_c3 = 30; // Acelerador

int min_limit_c4 = 30;
int max_limit_c4 = 150;
int default_value_c4 = 90; // Lateral (yaw)


int servo0Value = 0;
int servo1Value = 0;
int servo2Value = 0;
int servo3Value = 0;
int servo4Value = 0;

//Buzzer
int numberError=0 ;
bool beepCount = true;
#define BUZZER 46
#define NOTE_E7 2637
#define NOTE_C7 2093
#define NOTE_G7 3136
#define NOTE_G6 1568
#define NOTE_E6 1319
#define NOTE_A6 1760
#define NOTE_AS6 1865
#define NOTE_B6 1976
#define NOTE_A7 3520
#define NOTE_F7 2794
#define NOTE_D7 2349


void init_buzzer(){
  pinMode(BUZZER, OUTPUT);
}
// Notas de la canción de Mario (parte corta)
int melody[] = {
  NOTE_E7, NOTE_E7, 0, NOTE_E7,
  0, NOTE_C7, NOTE_E7, 0,
  NOTE_G7, 0, 0,  0,
  NOTE_G6, 0, 0, 0,
  NOTE_C7, 0, 0, NOTE_G6,
  0, 0, NOTE_E6, 0,
  0, NOTE_A6, NOTE_B6, NOTE_AS6,
  NOTE_A6, NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, NOTE_F7, NOTE_G7, 0,
  NOTE_E7, NOTE_C7, NOTE_D7, NOTE_B6,
  0, 0, NOTE_C7, 0, 0, NOTE_G6,
  0, 0, NOTE_E6, 0, 0, NOTE_A6,
  NOTE_B6, NOTE_AS6, NOTE_A6, NOTE_G6,
  NOTE_E7, NOTE_G7, NOTE_A7, NOTE_F7,
  NOTE_G7, 0, NOTE_E7, NOTE_C7,
  NOTE_D7, NOTE_B6, 0, 0
};

int noteDurations[] = {
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12
};
// Función para reproducir la canción de Mario (parte corta)
void playBuzzer() {
  for (int thisNote = 0; thisNote < 16; thisNote++) {

    int noteDuration = 1000 / noteDurations[thisNote];
    tone(BUZZER, melody[thisNote], noteDuration);

    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(BUZZER);
  }
}
const long interval = 1000;
void beepOnGpsDetection() {
    if (gpsDetected) {
      if (beepCount) {
        Serial.print("tono: ");
        Serial.println(beepCount);
        tone(BUZZER, NOTE_C7, 300);  // Emitir un pitido
        delay(500);  // Esperar a que el pitido termine
        noTone(BUZZER);  // Apagar el buzzer
        tone(BUZZER, NOTE_C7, 300);  // Emitir un pitido
        delay(500);  // Esperar a que el pitido termine
        noTone(BUZZER);  // Apagar el buzzer
        tone(BUZZER, NOTE_C7, 300);  // Emitir un pitido
        delay(500);  // Esperar a que el pitido termine
        noTone(BUZZER);  // Apagar el buzzer
        beepCount=false;
      }
    }
}


void print_channels(){

  Serial.println("Valores leidos de los canales:");
  Serial.print("Ch1: ");
  Serial.print(ch1Value);
  Serial.print(" | Ch2: ");
  Serial.print(ch2Value);
  Serial.print(" | Ch3: ");
  Serial.print(ch3Value);
  Serial.print(" | Ch4: ");
  Serial.print(ch4Value);
  Serial.print(" | Ch5: ");
  Serial.print(ch5Value);
  Serial.print(" | Ch6: ");
  Serial.println(ch6Value);

  Serial.println("Valores PWM enviados");
  Serial.print("s1: ");
  Serial.print(servo0Value);
  Serial.print(" | s2: ");
  Serial.print(servo1Value);
  Serial.print(" | s3: ");
  Serial.print(servo2Value);
  Serial.print(" | s4: ");
  Serial.println(servo3Value);
 


}
void setServos() {
  // Lee los valores actuales de los servos


  // Verifica si los nuevos valores son diferentes de los actuales
    pca9685.setPWM(SER0_ALERONES, 0, servo0Value);
    pca9685.setPWM(SER1_ELEVADORES, 0, servo1Value);
    pca9685.setPWM(SER2_MOTOR, 0, servo2Value);
    pca9685.setPWM(SER3_TIMON, 0, servo3Value);
  
}

int pulseWidth(int angle) {
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}


double CalcularPid(double actual, double PosicionDeseada, double priError, double toError, double min, double max, double kp, double ki, double kd,double minMaxPid, float signo) {
    double error = PosicionDeseada - actual;
    toError += error;
    double Pvalue = error * kp;
    double Ivalue = toError * ki;
    double Dvalue = (error - priError) * kd;
    double PIDVal = Pvalue + Ivalue + Dvalue;
    
    priError = error;

     // Limitar el valor de PIDVal dentro del rango de -90 a 90
    if (PIDVal >  minMaxPid) PIDVal =  minMaxPid;
    if (PIDVal < - minMaxPid) PIDVal = - minMaxPid;

    // Mapear PIDVal de -90 a 90 al rango del servo (min a max)
    double valToreturn = 0;

    if (signo) valToreturn = map(PIDVal, -minMaxPid, minMaxPid, min, max);
    else valToreturn = map(PIDVal, -minMaxPid, minMaxPid, max, min);

    // Limitar el valor de retorno dentro de los límites del servo (min a max)
    if (valToreturn > max) valToreturn = max;
    if (valToreturn < min) valToreturn = min;

    // Serial.print("Error: ");
    // Serial.print(error);
    // Serial.print(" | P: ");
    // Serial.print(Pvalue);
    // Serial.print(" | I: ");
    // Serial.print(Ivalue);
    // Serial.print(" | D: ");
    // Serial.print(Dvalue);
    // Serial.print(" | PID: ");
    // Serial.print(PIDVal);
    // Serial.print(" | valToretrun: ");
    // Serial.println(valToreturn);
    return valToreturn;
}

void updateChannelsAuto(){

    // Obtiene los valores dos canais dentro da faixa de -100 a 100
  float pidRoll = CalcularPid(rollValue, PosicionDeseadaRoll, priErrorRoll, toErrorRoll, min_limit_c1, max_limit_c1, kpRoll, kiRoll, kdRoll, 60, 1);
  servo0Value = pulseWidth(pidRoll);

  float pidPitch = CalcularPid(pitchValue, PosicionDeseadaPitch, priErrorPitch, toErrorPitch, min_limit_c2, max_limit_c2, kpPitch, kiPitch, kdPitch,30, 0);
  servo1Value = pulseWidth(pidPitch);
  /*
  float pidYaw = CalcularPid(yawValue, PosicionDeseadaYaw, priErrorYaw, toErrorYaw, min_limit_c4, max_limit_c4, kpYaw, kiYaw, kdYaw);
  servo3Value = pulseWidth(pidYaw);
  ch4Value  = pidYaw;


  */

}
void updateChannels(){

    // Obtiene los valores dos canais dentro da faixa de -100 a 100

  ch1Value = flySky.getChannel1Value(60, -60, default_value_c1);
  ch2Value = flySky.getChannel2Value(30, -30, default_value_c2);
  ch3Value = flySky.getChannel3Value(min_limit_c3, max_limit_c3, default_value_c3);
  ch4Value = flySky.getChannel4Value(min_limit_c4, max_limit_c4, default_value_c4);

  double pidRoll = CalcularPid(rollValue, ch1Value, priErrorRoll, toErrorRoll, min_limit_c1, max_limit_c1, kpRoll, kiRoll, kdRoll, 60, 1);
  servo0Value = pulseWidth(pidRoll);

  double pidPitch = CalcularPid(pitchValue, ch2Value, priErrorPitch, toErrorPitch, min_limit_c2, max_limit_c2, kpPitch, kiPitch, kdPitch,30, 0);
  servo1Value = pulseWidth(pidPitch);

  servo2Value = pulseWidth(ch3Value);
  servo3Value = pulseWidth(ch4Value);
}



float calculateHeading(float mx, float my) {
    // Calcular el ángulo en radianes
    float heading_rad = atan2(my, mx);
    
    // Convertir el ángulo a grados
    float heading_deg = heading_rad * 180.0 / M_PI;
    
    // Asegurarse de que el ángulo esté en el rango de 0 a 360 grados
    if (heading_deg < 0) {
        heading_deg += 360;
    }
    
    return heading_deg;
}


void managePlaneMode(){
  ch5Value = flySky.readSwitch(CH5, false); // Canal 5 es el switch 5
  ch6Value = flySky.readSwitch(CH6, false);

  if (ch5Value)
    updateChannelsAuto();
  else{
    updateChannels();
  }
  //print_channels();
}

void Bno() {
  if (millis() - lastTime >= BNO055_SAMPLERATE_DELAY_MS) {
    lastTime = millis();
    
    bno055_read_euler_hrp(&myEulerData);
    // Actualizar variables globales en lugar de imprimir
    yawValue =  360-(float(myEulerData.h) / 16.00);
    pitchValue =(float(myEulerData.r) / 16.00);
    rollValue = -(float(myEulerData.p) / 16.00);

    // Actualizar estados de calibración (sin imprimir)
    bno055_read_mag_xyz(&magData);
    mag_x=magData.x;
    mag_y=magData.y;
    mag_z=magData.z;

    compass_value = calculateHeading(mag_x, mag_y);
    

    bno055_get_accelcalib_status(&accelCalibStatus);
    bno055_get_gyrocalib_status(&gyroCalibStatus);
    bno055_get_syscalib_status(&sysCalibStatus);
    bno055_get_magcalib_status(&magCalibStatus);
  }
}


void printBNO055Values() {
  Serial.print(F("Orientation (Yaw, Pitch, Roll): "));
  Serial.print(yawValue);
  Serial.print(F(", "));
  Serial.print(pitchValue);
  Serial.print(F(", "));
  Serial.println(rollValue);

  // Imprimir también los estados de calibración si lo necesitas
  Serial.print(F("Calibration (Sys, Gyro, Accel, Mag): "));
  Serial.print(sysCalibStatus, DEC);
  Serial.print(F(", "));
  Serial.print(gyroCalibStatus, DEC);
  Serial.print(F(", "));
  Serial.print(accelCalibStatus, DEC);
  Serial.print(F(", "));
  Serial.println(magCalibStatus, DEC);
}
void initializeCompass() {
  compass.initialize();
  if (compass.testConnection()) {
    Serial.println("HMC5883L initialized successfully.");
  } else {
    Serial.println("Error initializing HMC5883L.");
  }
}

void initSensors() {
  bmpInitialized = bmp.begin(0x76);
  mpuInitialized = mpu.begin(0x68);
  BNO_Init(&myBNO);
  bno055_set_operation_mode(OPERATION_MODE_NDOF);
  //initializeCompass();
}
float calculateEMA(float currentReading, float previousEMA, float alpha) {
  return (alpha * currentReading) + ((1 - alpha) * previousEMA);
}
void readBMP280Data() {
  float currentTemperature = bmp.readTemperature();
  float currentPressure = bmp.readPressure() / 100.0F;
  float currentAltitude = bmp.readAltitude(1028); //  La presión del aire al nivel del mar es 1028 hPa (QNH).

  pressure = calculateEMA(currentPressure, pressure, alpha);
  temperature = calculateEMA(currentTemperature, temperature, alpha);
  altitude = calculateEMA(currentAltitude, altitude, alpha);

  float rawTemperature = currentTemperature;
  float rawPressure = currentPressure;
  float rawAltitude = currentAltitude;

  pressureSensor.updateEnvironmentalData(temperature, pressure);

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

  //temperature=temp.temperature;

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
  yaw_raw_mpu=angleAccelZ;
  KalmanFilter(angleAccelZ, gyroZ, &angle_y, &bias_y, P_y);
  yaw = angle_y;

  // Filtro de Kalman para Pitch (Y)
  float angleAccelY = atan2(-aX, sqrt(aY * aY + aZ * aZ)) * 180 / PI;
  pitch_raw_mpu=angleAccelZ;
  KalmanFilter(angleAccelY, gyroY, &angle_x, &bias_x, P_x);
  pitch = -angle_x;

  // Filtro de Kalman para Roll (X)
  float angleAccelX = atan2(aY, aZ) * 180 / PI; // Calcular ángulo con acelerómetro
  float rate_roll = gyroX - bias_roll;
  roll_raw_mpu=angleAccelZ;
    
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

void displayInfo() {
  if (gps.location.isValid()) {
    Latitud=gps.location.lat();
    Longitud=gps.location.lng();
      Serial.print(", ");
    Serial.print(Latitud, 6);
      Serial.print(", ");
    Serial.println(Longitud, 6);
  } else {
     Latitud=0;
    Longitud=0;
      Serial.print(", ");
    Serial.print(Latitud, 6);
      Serial.print(", ");
    Serial.println(Longitud, 6);
  }
}
void show_sensors(){

  Serial.print("Temperatura (C): ");
  Serial.println(temperature);
  Serial.print("Presión (hPa): ");
  Serial.println(pressure);
  Serial.print("Altitud (mSA) ");
  Serial.println(altitude);

  Serial.print("Acelerómetro (X, Y, Z): ");
  Serial.print(aX);
  Serial.print(", ");
  Serial.print(aY);
  Serial.print(", ");
  Serial.println(aZ);
  Serial.print("Yaw: ");
  Serial.print(-yaw);
  Serial.print(", Pitch: ");
  Serial.print(pitch);
  Serial.print(", Roll: ");
  Serial.println(roll);

  Serial.print(F("Orientation (Yaw, Pitch, Roll): "));
  Serial.print(-yawValue);
  Serial.print(F(", "));
  Serial.print(pitchValue);
  Serial.print(F(", "));
  Serial.println(rollValue);
   while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      displayInfo();
    }
  }

}

void data_gps(){
  bool validLocation = false;
  numberError++;
  while (Serial2.available() > 0) {
    validLocation = true;
    if (gps.encode(Serial2.read())) {
      if (gps.location.isValid()) {
        
        gpsDetected = true;
        numberError=0;
        
        Latitud = gps.location.lat();
        Longitud = gps.location.lng();
    
      } 


    }


  }

  if (!validLocation && (numberError > 10)) {
    gpsDetected = false;
              
        beepCount=true;
      }
  }


void createNewFile() {
  int fileCounter = 0;
  File root = SD.open("/");
  File file = root.openNextFile();

  while (file) {
    fileCounter++;
    file = root.openNextFile();
  }

  fileName = "/dataSaved_" + String(fileCounter + 1) + ".csv";

  // Crea y abre el archivo para escribir, y escribe la cabecera
  myFile = SD.open(fileName, FILE_WRITE);
  if (myFile) {
    myFile.println("temperatura,presion,altitud,yaw1,pitch1,roll1,yaw,pitch,roll,compass,latitud,longitud");
    myFile.close();
    fileCreated = true;
    Serial.println("File created: " + fileName);
  } else {
    Serial.println("Error creating file.");
  }
}



void saveData() {
  // Abre el archivo para añadir datos
  myFile = SD.open(fileName, FILE_APPEND);
  if (myFile) {
    // Escribe los datos en el archivo
    myFile.print(temperature);
    myFile.print(",");
    myFile.print(pressure);
    myFile.print(",");
    myFile.print(altitude);
    myFile.print(",");
    myFile.print(yawValue);
    myFile.print(",");
    myFile.print(pitchValue);
    myFile.print(",");
    myFile.print(rollValue);
    myFile.print(",");
    myFile.print(yaw);
    myFile.print(",");
    myFile.print(pitch);
    myFile.print(",");
    myFile.print(roll);
    myFile.print(",");
    myFile.print(compass_value);
    myFile.print(",");
    myFile.print(Latitud);
    myFile.print(",");
    myFile.println(Longitud);
    myFile.close();
    //Serial.println("Data written to file: " + fileName);
  } else {
    //Serial.println("Error opening file for writing.");
  }
}




void show_sensors2() {
  Serial.print("{temperatura:");
  Serial.print(temperature); // valor sensor temperatura MPU-6050 acelerometro y giroscopio no tan preciso
  Serial.print(", rawTemperatura:");
  Serial.print(rawTemperature); // valor sensor temperatura Bmp280 sin filtrar
  Serial.print(", presion:");
  Serial.print(pressure); // presion del BMP180 sensor presion barometrica
  Serial.print(", rawPresion:");
  Serial.print(rawPressure); // presion del BMP180 sensor presion barometrica sin filtrar
  Serial.print(", altitud:");
  Serial.print(altitude); // altitude sensor presion barometrica
  Serial.print(", rawAltitud:");
  Serial.print(rawAltitude); // altitude sensor presion barometrica sin filtrar
  Serial.print(", yaw1:");
  Serial.print(yaw); // yaw del MPU-6050
  Serial.print(", pitch1:");
  Serial.print(pitch); // pitch del MPU-6050
  Serial.print(", roll1:");
  Serial.print(roll); // roll del MPU-6050
  Serial.print(", yaw:");
  Serial.print(yawValue); // yaw del BNO055 brujula y giroscopio de precision
  Serial.print(", pitch:");
  Serial.print(pitchValue); // pitch del BNO055 brujula y giroscopio de precision
  Serial.print(", roll:");
  Serial.print(rollValue); // roll del BNO055 brujula y giroscopio de precision
  Serial.print(", compass:");
  Serial.print(compass_value); // magnetometro x
  Serial.print(", latitud:");
  Serial.print(Latitud,6);
  Serial.print(", longitud:");
  Serial.print(Longitud,6);
  Serial.print("}");
  Serial.println(); // Agregar nueva línea al final para separar las lecturas

  /*char text[10];
  dtostrf(rollValue, 8, 3, text) ;
  radio.write(&text, sizeof(text));
  //Serial.print("Imprimiendo en el radio: ");
  Serial.print(text);
  */
}
void sendMessage(String message) {
  
  radio.write(&message, sizeof(message));
}

void printValueWithFixedWidth(float value, int totalWidth) {
  char sign = (value < 0) ? '-' : ' '; // Determina el signo
  int intValue = (int)abs(value); // Parte entera del valor, siempre positiva
  float decimalValue = abs(value) - intValue; // Parte decimal del valor
  int intWidth = (intValue == 0) ? 1 : (int)log10(intValue) + 1; // Ancho de la parte entera
  int padding = totalWidth - intWidth - 4; // Calcula el espaciado necesario, 4 es para el signo, punto y dos decimales

  // Imprime el signo y los espacios de padding
  display.print(sign);
  for (int i = 0; i < padding; i++) {
    display.print(' ');
  }

  // Imprime la parte entera
  display.print(intValue);

  // Imprime la parte decimal con dos dígitos
  display.print('.');
  int decimalPart = (int)(decimalValue * 100); // Multiplica por 100 para obtener dos dígitos decimales
  if (decimalPart < 10) display.print('0'); // Añade un cero si es necesario
  display.print(decimalPart);
}


void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);

  // Imprime los títulos

  // Imprime Yaw
  display.print("Compas: "); display.print(compass_value); display.println(" °");
  display.print("Yaw: ");printValueWithFixedWidth(yawValue, 3);display.println();
  display.print("Pitch: ");printValueWithFixedWidth(pitchValue, 3);display.println();
  display.print("Roll: ");printValueWithFixedWidth(rollValue, 1);display.println();
  display.print("Temp: "); display.print(temperature); display.println(" C");
  display.print("Alt: "); display.print(altitude); display.println(" m");
  display.print("Lat: "); display.print(Latitud,6);display.println();
  display.print("Long: "); display.print(Longitud,6);display.println();
  display.display();
}


void updateSerial() {
  delay(500);
  while (Serial.available()) {
    Serial2.write(Serial.read()); // Forward what Serial received to Software Serial Port
  }
  while (Serial2.available()) {
    Serial.write(Serial2.read()); // Forward what Software Serial received to Serial Port
  
}
}

//Compass


void  Control(){
  Controlador.Update_Position(Latitud, Longitud,yawValue);
  Controlador.Update_Velocidad(VelocidadActual);
  Controlador.Update_orientation(yawValue,rollValue,pitchValue);
  
  Controlador.Update_tiempo(tiempo);
  Controlador.Update_Velocidad(60);
  Controlador.UAV_Search();

}

TaskHandle_t Tarea0;
void loop0(void* parameter);
void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);  
  scanI2C();
  Serial.println("Setup completed.");
  initSensors();
  
  Serial2.begin(9600,SERIAL_8N1, RX2, TX2); // RX2 (GPIO16) y TX2 (GPIO17) en ESP32 NO CAMBIAR EL BAUD RATE
  Serial.println(F("Iniciando GPS..."));
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
    
  }

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

  if (!SD.begin(CS_PIN, SPI)) {
    Serial.println("Initialization of SD card failed!");
    return;
  }
  Serial.println("SD card is ready to use.");

  createNewFile();
  
  pca9685.begin();
  pca9685.setPWMFreq(FREQUENCY); 
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  // Display static text
  display.println("UAV Variables");
  display.display(); 
  //chipSetup();
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  // 13 0X0d DIRECCIÓN i2c
  pressureSensor.begin();
  Controlador.waitPoints_coordenadas_a_rectangulares(inputCoords, numCoords);

    for (int i = 0; i < numCoords; ++i) {
      Serial.print("Coordenada ");
      Serial.print(i);
      Serial.print(": X = ");
      Serial.print(outputCoords[i][0]);
      Serial.print(", Y = ");
      Serial.println(outputCoords[i][1]);
  }
  
  if (gps.location.isValid()){
    
   }
  init_buzzer();

  

  playBuzzer();

  xTaskCreatePinnedToCore(loop0, "Tarea_0", 2048, NULL, 1, &Tarea0, 0);
}

 



void loop() {
  unsigned long currentMillis = millis();
  tiempo=currentMillis;
  //updateChannels();
  //updateChannelsAuto();
  
  data_gps();
  readBMP280Data(); 
  readMPU6050Data();
  Bno();
  beepOnGpsDetection();
  //show_sensors2();
  //compass_degrees=getCompassHeading() ;
  //Control();
  
  //imprimirCoordenadas(CoordenadasRectangulares);
   
  
  if (currentMillis - previousMillis2 >= interval2) {
    previousMillis2 = currentMillis;
    saveData();
    pressureSensor.printVelocity();
    //Serial.print("tiempo: "); 
    //Serial.println(tiempo);
    //Controlador.ImprimirDatos();
    //show_sensors();
    //print_channels();
    //show_sensors2();
    //printBNO055Values();
    
    //Serial.print("Heading: ");
    //Serial.print(mag_x);
    //Serial.print(",");
    //Serial.println(mag_y);
  }
  if (currentMillis - previousMillis >= interval1) {
    previousMillis = currentMillis;
    
    // Actualizar los valores de los sensores
    // Actualizar la pantalla
    updateDisplay();
  }

  }

  //delay(1000); // Pausa de 1 segundo entre lecturas

void loop0(void*parameter){
  while(1==1){
    managePlaneMode();
    setServos();
  }
}