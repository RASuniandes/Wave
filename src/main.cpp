 
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>
#include <Servo.h>
#include <HardwareSerial.h>
#include <TinyGPS.h>

#include <TinyGPSPlus.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


// The TinyGPSPlus object



#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


TinyGPSPlus gps;


void setup() {


  Serial.begin(9600);
  Serial2.begin(9600);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  delay(2000);
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  // Display static text
  display.println("UAV Variables");
  display.display(); 
  delay(1000);
}

void displayInfo()
{


  Serial.print(F("Location: "));


  if (gps.location.isValid()){
    display.clearDisplay();
    display.setCursor(0, 0);
    Serial.print(gps.location.lat(), 6);
    display.print("Lat: ");
    display.println(gps.location.lat(), 6);
    display.print(F(","));
    display.print("Lng: ");
    display.println(gps.location.lng(), 6);
    display.display(); 
    delay(100);
    





  }  
  else
  {
    display.clearDisplay();
    display.setCursor(0, 0);
    Serial.print(gps.location.lat(), 6);
    display.print("Lat: ");
    display.println(gps.location.lat(), 6);
    display.print(F(","));
    display.print("Lng: ");
    display.println(gps.location.lng(), 6);
    display.display(); 

    delay(100);
    
  }

}

void loop() {
  //updateSerial();
  while (Serial2.available() > 0)
    if (gps.encode(Serial2.read()))
      displayInfo();

  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }
  display.display(); 


}





void updateSerial()
{
  delay(500);
  while (Serial.available())
  {
    Serial2.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (Serial2.available())
  {
    Serial.write(Serial2.read());//Forward what Software Serial received to Serial Port
  }
}