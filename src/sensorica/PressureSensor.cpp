#include "PressureSensor.h"

PressureSensor::PressureSensor(int pin, float v_s, float sensitivity, float offset)
  : sensorPin(pin), V_S(v_s), sensitivity(sensitivity), offset(offset), temperature(0), pressure_atmospheric(0) {
}

void PressureSensor::begin() {
  int sensor_value_sum = 0;
  for (int ii = 0; ii < offset_size; ii++) {
    sensor_value_sum += analogRead(sensorPin);
    delay(10);
  }
  offset_value = sensor_value_sum / offset_size;
}

float PressureSensor::getPressure() {
  float adc_avg = 0;
  for (int ii = 0; ii < veloc_mean_size; ii++) {
    adc_avg += analogRead(sensorPin);
  }
  adc_avg /= veloc_mean_size;

  float adjusted_adc = adc_avg - offset_value;
  float voltage = adjusted_adc * (3.3 / 4095.0);
  float pressure = (voltage - (offset * V_S)) / (sensitivity * V_S);

  if (pressure < 0) {
    pressure = 0;
  }

  return pressure;
}

float PressureSensor::getVelocity() {
  float pressure = getPressure();
  if (pressure > 0) {
    return sqrt(2 * pressure * 1000 / RHO_AIR);
  } else {
    return 0;
  }
}

void PressureSensor::printVelocity() {
  float velocity = getVelocity();
  float pressure = getPressure();
  Serial.print("Velocidad del aire (m/s): ");
  Serial.print(velocity);
  Serial.print(" Presión (kpa): ");
  Serial.println(pressure);
}

void PressureSensor::updateEnvironmentalData(float temp, float pressure) {
  temperature = temp;
  pressure_atmospheric = pressure;
  calculateAirDensity();
}

void PressureSensor::calculateAirDensity() {
  float T = temperature + 273.15; // Convertir a Kelvin
  RHO_AIR = pressure_atmospheric * 100 / (287.05 * T); // Convertir presión a Pascales
}
  
