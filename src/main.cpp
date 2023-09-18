#include <Arduino.h>
#include <IBusBM.h>

#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Servo.h>
#include <HardwareSerial.h>
#include <TinyGPS.h>

#include <TinyGPSPlus.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Create iBus Object
#include "telemetria/FlySky.h"

#include <Adafruit_PWMServoDriver.h>
// Include Adafruit PCA9685 Servo Library
// Creat object to represent PCA9685 at default I2C address
// Definição das conexões de entrada
#define CH1 34
#define CH2 35
#define CH3 32
#define CH4 33
#define CH5 26
#define CH6 25
 
// Configurar receptor
FlySky flySky(CH1, CH2, CH3, CH4, CH6);
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

#define SER0  0   //Servo Motor 0 on connector 0
#define SER1  1  //Servo Motor 1 on connector 12

#define SER2  2   //Servo Motor 0 on connector 0
#define SER3  3


const int servoMin = 150;  // Valor mínimo del pulso PWM para el servo (ajusta según sea necesario)
const int servoMax = 600;  // Valor máximo del pulso PWM para el servo (ajusta según sea necesario)




void setup(){

  Serial.begin(115200);
  pca9685.begin();
  pca9685.setPWMFreq(60); 

}
 
 
void loop() {
  
  // Obtenção dos valores dos canais dentro da faixa de -100 a 100
  int ch1Value = flySky.getChannel1Value();
  int ch2Value = flySky.getChannel2Value();
  int ch3Value = flySky.getChannel3Value();
  int ch4Value = flySky.getChannel4Value();
  bool ch5Value = flySky.getAutomaticFly(); // Canal 5 es el switch 5
  bool ch6Value = flySky.readSwitch(25, false);


  // Enviar a informação dos valores dos canais através da comunicação serial
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


  int servo0Value = map(ch1Value, 1000, 2000, servoMin, servoMax);
  int servo1Value = map(ch2Value, 1000, 2000, servoMin, servoMax);
  int servo2Value = map(ch3Value, 1000, 2000, servoMin, servoMax);
  int servo3Value = map(ch4Value, 1000, 2000, servoMin, servoMax);

  Serial.print("s1: ");
  Serial.print(servo0Value);
  Serial.print(" | s2: ");
  Serial.print(servo1Value);
  Serial.print(" | s3: ");
  Serial.print(servo2Value);
  Serial.print(" | s4: ");
  Serial.print(servo3Value);
  Serial.println(" | s5: ");



  // Envía señales PWM a los servos
  pca9685.setPWM(SER0, 0, servo0Value);
  pca9685.setPWM(SER1, 0, servo1Value);
  pca9685.setPWM(SER2, 0, servo2Value);
  pca9685.setPWM(SER3, 0, servo3Value);



  delay(100);
}