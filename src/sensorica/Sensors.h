#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include "BNO055_support.h"
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include "HMC5883L.h"
#include "ms4525do.h"

class Sensors {
public:
    Sensors();
    void begin();
    void readData();
    void updateDisplay();
    void printValues();
    void updateGPS();

    float getTemperature() const { return temperature; }
    float getPressure() const { return pressure; }
    float getAltitude() const { return altitude; }
    float getYaw() const { return yaw; }
    float getPitch() const { return pitch; }
    float getRoll() const { return roll; }
    float getCompass() const { return compass_value; }
    float getLatitude() const { return Latitud; }
    float getLongitude() const { return Longitud; }
    float getAirSpeed() const { return airSpeed; }
    float getAirTemperature() const { return airTemperature; }
    float getAirPressure() const { return airPressure; }
    float getAirPressurePsi() const { return airPressurePsi; }

private:
    Adafruit_BMP280 bmp;
    Adafruit_MPU6050 mpu;
    HMC5883L compass;
    Adafruit_SSD1306 display;
    TinyGPSPlus gps;
    bfs::Ms4525do pitot;
    struct bno055_t myBNO;
    struct bno055_euler myEulerData;
    struct bno055_mag magData;
    #define SCREEN_WIDTH 128 // OLED display width, in pixels
    #define SCREEN_HEIGHT 64 // OLED display height, in pixels

    
 

    //---------BNO055-------------------------
    float yaw;
    float pitch;
    float roll;
    float compass_value;
    unsigned char accelCalibStatus = 0;
    unsigned char magCalibStatus = 0;
    unsigned char gyroCalibStatus = 0;
    unsigned char sysCalibStatus = 0;
    unsigned long lastTime = 0;

    //----------MPU6050----------------------
    float yawMpu, pitchMpu, rollMpu;
    float yaw_raw_mpu;
    float pitch_raw_mpu;
    float roll_raw_mpu;
    float aX, aY, aZ;
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float angle_y, bias_y, P_y[2][2];
    float angle_x, bias_x, P_x[2][2];
    float angle_roll, bias_roll, P_roll[2][2];
    const float alpha = 0.1; // Factor de suavizado
    const int TX2 = 11;
    const int RX2 = 10;
    const float Q_angle = 0.001;
    const float Q_bias = 0.003;
    const float R_measure = 0.03;
    long tiempo_prev;

    //----------GPS----------------------
    float Latitud;
    float Longitud;

    //----------BMP280----------------------
    float hpaZone=1028;//hPa Bogotá
    float initialAltitude;
    float alture;
    float rawTemperature;
    float rawPressure;
    float rawAltitude;

    float temperature;
    float pressure;
    float altitude;

    //------------PITOT-----------------
    float airTemperature;
    float airPressure;
    float airPressurePsi;
    float airSpeed;
    float RHO_AIR;



    void initializeCompass();
    void readBMP280Data();
    void readMPU6050Data();
    void readPitotData();
    void updateRho();
    void updateAirSpeed();
    void PressurePSI();

    void KalmanFilter(float newAngle, float newRate, float *angle, float *bias, float P[2][2]);
    float calculateEMA(float currentReading, float previousEMA, float alpha);
    float calculateHeading(float mx, float my);

    //------------Datalog-------------
    void displayInfo();
    void showSensors();
    void readBnoData();
    void showPressure();
};

#endif // SENSORS_H
