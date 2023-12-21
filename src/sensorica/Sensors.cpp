#include "Sensors.h"

Sensors::Sensors() : bmp280(), mpu6050() {
  bmpWorking = false; // Inicialmente, el sensor BMP280 no está funcionando
  mpuWorking = false; // Inicialmente, el sensor MPU6050 no está funcionando
}

bool Sensors::initialize() {
  // Inicializar el sensor BMP280
  if (bmp280.begin(0x76)) { // Dirección I2C BMP280
    bmpWorking = true;
  } else {
    // Si no se puede inicializar el BMP280, desactivamos sus métodos
    disableBMP();
  }

  // Inicializar el sensor MPU6050
  if (mpu6050.begin()) {
    // Configurar la escala de aceleración y rango del giroscopio para el MPU6050
    mpu6050.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu6050.setGyroRange(MPU6050_RANGE_500_DEG);

    // Configurar la banda de filtro del MPU6050
    mpu6050.setFilterBandwidth(MPU6050_BAND_21_HZ);

    // Inicializar variables previas
    tiempo_prev = millis();

    // Obtener la temperatura inicial
    temperature = bmp280.readTemperature();

    mpu6050.setSampleRateDivisor(100);

    mpuWorking = true;

    //calibrateMPU();
  } else {
    // Si no se puede inicializar el MPU6050, desactivamos sus métodos
    disableMPU();
  }

  // Devolver true solo si al menos uno de los sensores está funcionando
  return bmpWorking || mpuWorking;
}


void Sensors::calibrateMPU() {
  Serial.println("Calibrating MPU6050...");
  int numReadings = 100; // Número de lecturas para promediar
  int ax_sum = 0, ay_sum = 0, az_sum = 0;
  int gx_sum = 0, gy_sum = 0, gz_sum = 0;

  // Realizar lecturas y sumarlas
  for (int i = 0; i < numReadings; i++) {
    sensors_event_t a, g, temp;
    mpu6050.getEvent(&a, &g, &temp);
    ax_sum += a.acceleration.x;
    ay_sum += a.acceleration.y;
    az_sum += a.acceleration.z;
    gx_sum += g.gyro.x;
    gy_sum += g.gyro.y;
    gz_sum += g.gyro.z;
    delay(10); // Añadir un pequeño retraso para evitar lecturas muy rápidas
  }

  // Calcular los promedios
  ax_offset = ax_sum / numReadings;
  ay_offset = ay_sum / numReadings;
  az_offset = az_sum / numReadings;
  gx_offset = gx_sum / numReadings;
  gy_offset = gy_sum / numReadings;
  gz_offset = gz_sum / numReadings;

  // Aplicar offsets manualmente a las lecturas

    Serial.print(" ax_offset: ");
    Serial.println(ax_offset);
    Serial.print(" ay_offset: ");
    Serial.println(ay_offset);
    Serial.print(" az_offset: ");
    Serial.println(az_offset);
    Serial.print(" gx_offset: ");
    Serial.println(gx_offset);
    Serial.print(" gy_offset: ");
    Serial.println(gy_offset);
    Serial.print(" gz_offset: ");

    delay(100);

  Serial.println("MPU6050 calibration completed.");
}






float Sensors::getTemperature() {
  if (bmpWorking) {
    return bmp280.readTemperature();
  }
  return temperature;
}

float Sensors::getPressure() {
  if (bmpWorking) {
    return bmp280.readPressure();
  }
  return 0.0; // Devuelve 0 si el BMP280 no está funcionando
}

float Sensors::getAltitude() {
  if (bmpWorking) {
    return bmp280.readAltitude(1013.25); // Presión estándar al nivel del mar
  }
  return 0.0; // Devuelve 0 si el BMP280 no está funcionando
}

void Sensors::getOrientation(float &yaw, float &pitch, float &roll) {
  if (mpuWorking) {
    mpu6050.getEvent(&accelEvent, &gyroEvent, &tempEvent);
  
    // Cálculos para obtener yaw, pitch y roll

    float ax=accelEvent.acceleration.x-ax_offset;
    float ay=accelEvent.acceleration.y-ay_offset;
    float az=accelEvent.acceleration.z-az_offset;



    float gx=gyroEvent.gyro.x-gx_offset;
    float gy=gyroEvent.gyro.y-gy_offset;
    float gz=gyroEvent.gyro.z-gz_offset;

    yaw = atan2(gx, gz);
    pitch = atan2(-gx, sqrt(ay * ay + az * az));
    roll = atan2(ay, az);
  }
}

void Sensors::getAcceleration(float &accelX, float &accelY, float &accelZ) {
  if (mpuWorking) {
    mpu6050.getEvent(&accelEvent, &gyroEvent, &tempEvent);
  
    accelX = (accelEvent.acceleration.x -ax_offset)* 9.81; // Convertir de m/s^2 a g
    accelY = (accelEvent.acceleration.y -ay_offset)* 9.81; // Convertir de m/s
    accelZ = (accelEvent.acceleration.z-az_offset)* 9.81; // Convertir de m/s* 9.81
  }
}

void Sensors::getRotation(float &gyroX, float &gyroY, float &gyroZ) {
  if (mpuWorking) {
    mpu6050.getEvent(&accelEvent, &gyroEvent, &tempEvent);

    dt = (millis() - tiempo_prev) / 1000.0;
    tiempo_prev = millis();


    float ax=accelEvent.acceleration.x-ax_offset;
    float ay=accelEvent.acceleration.y-ay_offset;
    float az=accelEvent.acceleration.z-az_offset;



    float gx=gyroEvent.gyro.x-gx_offset;
    float gy=gyroEvent.gyro.y-gy_offset;
    float gz=gyroEvent.gyro.z-gz_offset;

    //Calcular los ángulos con acelerometro
    float accel_ang_x=atan(ay/sqrt(pow(ax,2) + pow(az,2)))*(180.0/3.14);
    float accel_ang_y=atan(-ax/sqrt(pow(ay,2) + pow(az,2)))*(180.0/3.14);
    
    //Calcular angulo de rotación con giroscopio y filtro complemento  
    float ang_x = 0.98*(ang_x_prev+(gx/131)*dt) + 0.02*accel_ang_x;
    float ang_y = 0.98*(ang_y_prev+(gy/131)*dt) + 0.02*accel_ang_y;
  

    // Calcular ángulo de rotación en el eje Z utilizando el giroscopio (gyroZ)
    float ang_z = ang_z_prev + (gyroEvent.gyro.z * dt);

    gyroX = ang_x;
    gyroY = ang_y;
    gyroZ = ang_z;

    ang_x_prev = ang_x;
    ang_y_prev = ang_y;
    ang_z_prev = ang_z;
  }
}

void Sensors::disableBMP() {
  // Desactivar el BMP280 estableciendo bmpWorking a false
  bmpWorking = false;
}

void Sensors::disableMPU() {
  // Desactivar el MPU6050 estableciendo mpuWorking a false
  mpuWorking = false;
}





