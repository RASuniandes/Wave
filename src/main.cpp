#include <Arduino.h>
#include <Wire.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <RF24.h>
// #include "NAVEGACION/control.h"
#include <vector>
#include <iostream>
#include "control/ReguladorServos.h"
#include "navegacion/Waypoints.h"

ReguladorServos reguladorServos;

#include "sensorica/Sensors.h"
Sensors sensors;
// Pines para el lector de tarjetas SD
#define MOSI_PIN 35
#define SCK_PIN 36
#define MISO_PIN 37
#define CS_PIN 38

// #define BUFFER_SIZE 64 // Tamaño del buffer circular

// volatile char inputBuffer[BUFFER_SIZE];
// volatile int bufferHead = 0;
// volatile int bufferTail = 0;

File myFile;
bool fileCreated = false;
String fileName;

// // Buffers circulares para los sensores
// float bmp280Buffer[BUFFER_SIZE];
// int bmp280BufferHead = 0;
// int bmp280BufferTail = 0;

// float mpu6050Buffer[BUFFER_SIZE];
// int mpu6050BufferHead = 0;
// int mpu6050BufferTail = 0;

float tiempo = 0;
// Control Controlador(distanciaAuxiliar, kp, kd, cte_saturacion, condicionActualizacion);

#define CE_PIN 21
#define CSN_PIN 16

RF24 radio(CE_PIN, CSN_PIN);

  

// const byte address[6] = "00001";

// GPS declaracion
float Latitud = 0;
float Longitud = 0;

bool gpsDetected = false;

const int DOUT_Pin = 15;
const int SCK_Pin = 34;

unsigned long previousMillis1 = 0;
const long interval1 = 100;

float devicesFound = 0.0;

long tiempo_prev = 0;

const int chipSelect = 38;

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

#define SER0_ALERONES 0
#define SER1_ELEVADORES 1
#define SER2_MOTOR 2
#define SER3_TIMON 3


#define FREQUENCY 60

//======================================== Sonido Mario =======


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

void init_buzzer()
{
  pinMode(BUZZER, OUTPUT);
}
int melody[] = {
    NOTE_E7, NOTE_E7, 0, NOTE_E7,
    0, NOTE_C7, NOTE_E7, 0,
    NOTE_G7, 0, 0, 0,
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
    NOTE_D7, NOTE_B6, 0, 0};

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
    12, 12, 12, 12};

//======================================== Sonido Mario =======

void playBuzzer()
{
  for (int thisNote = 0; thisNote < 16; thisNote++)
  {
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(BUZZER, melody[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(BUZZER);
  }
}

void beepOnGpsDetection()
{
  if (gpsDetected)
  {
    if (beepCount)
    {
      Serial.print("tono: ");
      Serial.println(beepCount);
      tone(BUZZER, NOTE_C7, 300);
      delay(500);
      noTone(BUZZER);
      tone(BUZZER, NOTE_C7, 300);
      delay(500);
      noTone(BUZZER);
      tone(BUZZER, NOTE_C7, 300);
      delay(500);
      noTone(BUZZER);
      beepCount = false;
    }
  }
}

void setServos()
{
  pca9685.setPWM(SER0_ALERONES, 0, reguladorServos.getservo0Value());
  pca9685.setPWM(SER1_ELEVADORES, 0, reguladorServos.getservo1Value());
  pca9685.setPWM(SER2_MOTOR, 0, reguladorServos.getservo2Value());
  pca9685.setPWM(SER3_TIMON, 0, reguladorServos.getservo3Value());
}

void scanI2C()
{
  Serial.println("Scanning I2C Addresses");
  uint8_t cnt = 0;
  Wire.setClock(100000);

  for (uint8_t i = 0; i < 128; i++)
  {
    Wire.beginTransmission(i);
    uint8_t ec = Wire.endTransmission(true);

    if (ec == 0)
    {
      if (i < 16)
        Serial.print('0');
      Serial.print(i, HEX);
      cnt++;
    }
    else
    {
      Serial.print("..");
    }

    Serial.print(' ');
    if ((i & 0x0F) == 0x0F)
      Serial.println();
  }

  Serial.print("Scan Completed, ");
  Serial.print(cnt);
  Serial.println(" I2C Devices found.");

  devicesFound = cnt;

  if (devicesFound == 0)
  {
    Serial.println("No I2C devices found.");
  }
  else
  {
    Serial.println("I2C devices found.");
  }
}


void createNewFile()
{
  int fileCounter = 0;
  File root = SD.open("/");
  File file = root.openNextFile();

  while (file)
  {
    fileCounter++;
    file = root.openNextFile();
  }

  fileName = "/dataSaved_" + String(fileCounter + 1) + ".csv";

  myFile = SD.open(fileName, FILE_WRITE);
  if (myFile)
  {
    myFile.println("temperatura,presion,altitud,yaw1,pitch1,roll1,yaw,pitch,roll,compass,latitud,longitud");
    myFile.close();
    fileCreated = true;
    Serial.println("File created: " + fileName);
  }
  else
  {
    Serial.println("Error creating file.");
  }
}

void saveData()
{
  // myFile = SD.open(fileName, FILE_APPEND);
  // if (myFile)
  // {
  //   myFile.print(temperature);
  //   myFile.print(",");
  //   myFile.print(pressure);
  //   myFile.print(",");
  //   myFile.print(altitude);
  //   myFile.print(",");
  //   myFile.print(yawValue);
  //   myFile.print(",");
  //   myFile.print(pitchValue);
  //   myFile.print(",");
  //   myFile.print(rollValue);
  //   myFile.print(",");
  //   myFile.print(yaw);
  //   myFile.print(",");
  //   myFile.print(pitch);
  //   myFile.print(",");
  //   myFile.print(roll);
  //   myFile.print(",");
  //   myFile.print(compass_value);
  //   myFile.print(",");
  //   myFile.print(Latitud);
  //   myFile.print(",");
  //   myFile.println(Longitud);
  //   myFile.close();
  // }
}

// void sendMessage(String message)
// {
//   radio.write(&message, sizeof(message));
// }

// void Control()
// {
//   Controlador.Update_Position(Latitud, Longitud, yawValue);
//   Controlador.Update_Velocidad(VelocidadActual);
//   Controlador.Update_orientation(yawValue, rollValue, pitchValue);
//   Controlador.Update_tiempo(tiempo);
//   Controlador.Update_Velocidad(60);
//   Controlador.UAV_Search();
// }
void SDBegin()
{
  if (!SD.begin(CS_PIN, SPI))
  {
    Serial.println("Initialization of SD card failed!");
    return;
  }
  Serial.println("SD card is ready to use.");

  createNewFile();
}
TaskHandle_t Tarea0;
void loop0(void *parameter);

void setup()
{
  Serial.begin(115200);
  Wire.begin(8, 9);
  Wire.setClock(400000);
  scanI2C();
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

  SDBegin();

  pca9685.begin();
  pca9685.setPWMFreq(FREQUENCY);

  sensors.begin();
  // radio.begin();
  // radio.openWritingPipe(address);
  // radio.setPALevel(RF24_PA_LOW);
  // Controlador.waitPoints_coordenadas_a_rectangulares(inputCoords, numCoords);
  // if (gps.location.isValid()) {

  // }
  init_buzzer();
  playBuzzer();
  xTaskCreatePinnedToCore(loop0, "Tarea_0", 2048, NULL, 1, &Tarea0, 0);

 
  // Matrices de prueba


}

void loop()
{
  unsigned long currentMillis = millis();
  tiempo = currentMillis;
  sensors.readData();

  
  //  sensors.showSensors();
  // reguladorServos.print_channels();
  if (currentMillis - previousMillis1 >= interval1)
  {
    previousMillis1 = currentMillis;
    saveData();
    sensors.updateDisplay();
    //sensors.showPressure();
    // updateDisplay();
  }
}

void loop0(void *parameter)
{
  while (1 == 1)
  {
    reguladorServos.managePlaneMode(sensors.getRoll(), sensors.getPitch(), sensors.getLatitude(), sensors.getLongitude(), sensors.getAirSpeed(), sensors.getAltitude(), sensors.getYaw(), sensors.getAlture());
    //reguladorServos.print_channels();
    setServos();
  }
}
