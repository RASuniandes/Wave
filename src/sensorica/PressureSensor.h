#ifndef PRESSURESENSOR_H
#define PRESSURESENSOR_H
#include <Arduino.h>

class PressureSensor {
  public:
    PressureSensor(int pin, float v_s, float sensitivity, float offset);
    void begin();
    float getPressure();
    float getVelocity();
    void printVelocity();
    void updateEnvironmentalData(float temp, float pressure);

  private:
    int sensorPin;
    float V_S;
    float sensitivity;
    float offset;
    float RHO_AIR;
    int offset_value;
    const int offset_size = 100;
    const int veloc_mean_size = 20;
    const int zero_span = 2;
    float temperature;
    float pressure_atmospheric;
    void calculateAirDensity();
};

#endif // PRESSURESENSOR_H