#include "Sensors.h"

Sensors::Sensors()
  : display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire),
    temperature(0), pressure(0), altitude(0), yaw(0), pitch(0), roll(0), compass_value(0), Latitud(0), Longitud(0) {
}

void Sensors::begin() {
    bmp.begin(0x76);
    mpu.begin(0x68);
    BNO_Init(&myBNO);
    bno055_set_operation_mode(OPERATION_MODE_NDOF);

    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;) {}
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("UAV Variables");
    display.display();

    Serial2.begin(9600, SERIAL_8N1, RX2, TX2);
}

void Sensors::readData() {
    readBMP280Data();
    readMPU6050Data();
    Bno();
    updateGPS();
}

void Sensors::readBMP280Data() {
    float currentTemperature = bmp.readTemperature();
    float currentPressure = bmp.readPressure() / 100.0F;
    float currentAltitude = bmp.readAltitude(1028);

    temperature = calculateEMA(currentTemperature, temperature, alpha);
    pressure = calculateEMA(currentPressure, pressure, alpha);
    altitude = calculateEMA(currentAltitude, altitude, alpha);

    rawTemperature = currentTemperature;
    rawPressure = currentPressure;
    rawAltitude = currentAltitude;
}

void Sensors::readMPU6050Data() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    aX = a.acceleration.x;
    aY = a.acceleration.y;
    aZ = a.acceleration.z;
    accelX = aX * 9.81;
    accelY = aY * 9.81;
    accelZ = aZ * 9.81;

    gyroX = g.gyro.x;
    gyroY = g.gyro.y;
    gyroZ = g.gyro.z;

    float angleAccelZ = atan2(aY, aZ) * 180 / PI;
    yaw_raw_mpu = angleAccelZ;
    KalmanFilter(angleAccelZ, gyroZ, &angle_y, &bias_y, P_y);
    yaw = angle_y;

    float angleAccelY = atan2(-aX, sqrt(aY * aY + aZ * aZ)) * 180 / PI;
    pitch_raw_mpu = angleAccelZ;
    KalmanFilter(angleAccelY, gyroY, &angle_x, &bias_x, P_x);
    pitch = -angle_x;

    float angleAccelX = atan2(aY, aZ) * 180 / PI;
    float rate_roll = gyroX - bias_roll;
    angle_roll += (millis() - tiempo_prev) / 1000.0 * rate_roll;
    P_roll[0][0] += (millis() - tiempo_prev) / 1000.0 * (P_roll[1][1] - P_roll[0][1] - P_roll[1][0] + Q_angle);
    P_roll[0][1] -= (millis() - tiempo_prev) / 1000.0 * P_roll[1][1];
    P_roll[1][0] -= (millis() - tiempo_prev) / 1000.0 * P_roll[1][1];
    P_roll[1][1] += Q_bias * (millis() - tiempo_prev) / 1000.0;

    float y_roll = angleAccelX - angle_roll;
    float S_roll = P_roll[0][0] + R_measure;
    float K_roll[2] = {P_roll[0][0] / S_roll, P_roll[1][0] / S_roll};
    angle_roll += K_roll[0] * y_roll;
    bias_roll += K_roll[1] * y_roll;
    P_roll[0][0] -= K_roll[0] * P_roll[0][0];
    P_roll[0][1] -= K_roll[0] * P_roll[0][1];
    P_roll[1][0] -= K_roll[1] * P_roll[0][0];
    P_roll[1][1] -= K_roll[1] * P_roll[0][1];
    roll = angle_roll;

    tiempo_prev = millis();
}

void Sensors::Bno() {
    bno055_read_euler_hrp(&myEulerData);
    yaw = 360 - float(myEulerData.h) / 16.00;
    pitch = float(myEulerData.r) / 16.00;
    roll = -float(myEulerData.p) / 16.00;

    bno055_read_mag_xyz(&magData);
    compass_value = calculateHeading(magData.x, magData.y);
}

void Sensors::KalmanFilter(float newAngle, float newRate, float *angle, float *bias, float P[2][2]) {
    float S, K[2], y;
    float dt = (millis() - tiempo_prev) / 1000.0;

    *angle += dt * (newRate - *bias);
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
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

void Sensors::updateGPS() {
    while (Serial2.available() > 0) {
        if (gps.encode(Serial2.read())) {
            if (gps.location.isValid()) {
                Latitud = gps.location.lat();
                Longitud = gps.location.lng();
            } else {
                Latitud = 0;
                Longitud = 0;
            }
        }
    }
}

float Sensors::calculateEMA(float currentReading, float previousEMA, float alpha) {
    return (alpha * currentReading) + ((1 - alpha) * previousEMA);
}

float Sensors::calculateHeading(float mx, float my) {
    float heading_rad = atan2(my, mx);
    float heading_deg = heading_rad * 180.0 / M_PI;
    if (heading_deg < 0) {
        heading_deg += 360;
    }
    return heading_deg;
}

void Sensors::updateDisplay() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Compas: "); display.print(compass_value); display.println(" °");
    display.print("Yaw: "); display.print(yaw); display.println();
    display.print("Pitch: "); display.print(pitch); display.println();
    display.print("Roll: "); display.print(roll); display.println();
    display.print("Temp: "); display.print(temperature); display.println(" C");
    display.print("Alt: "); display.print(altitude); display.println(" m");
    display.print("Lat: "); display.print(Latitud, 6); display.println();
    display.print("Long: "); display.print(Longitud, 6); display.println();
    display.display();
}

void Sensors::printValues() {
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
    Serial.print(yaw);
    Serial.print(", Pitch: ");
    Serial.print(pitch);
    Serial.print(", Roll: ");
    Serial.println(roll);

    Serial.print(F("Orientation (Yaw, Pitch, Roll): "));
    Serial.print(yaw);
    Serial.print(F(", "));
    Serial.print(pitch);
    Serial.print(F(", "));
    Serial.println(roll);

    Serial.print("GPS (Lat, Long): ");
    Serial.print(Latitud, 6);
    Serial.print(", ");
    Serial.println(Longitud, 6);
}

void Sensors::displayInfo() {
    if (gps.location.isValid()) {
        Latitud = gps.location.lat();
        Longitud = gps.location.lng();
        Serial.print(", ");
        Serial.print(Latitud, 6);
        Serial.print(", ");
        Serial.println(Longitud, 6);
    } else {
        Latitud = 0;
        Longitud = 0;
        Serial.print(", ");
        Serial.print(Latitud, 6);
        Serial.print(", ");
        Serial.println(Longitud, 6);
    }
}