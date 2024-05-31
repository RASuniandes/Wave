#ifndef ReguladorServos_h
#define ReguladorServos_h

#include <Arduino.h>

class ReguladorServos {
public:
  // ReguladorServos();
  double CalcularPID(double actual, double PosicionDeseada, double priError, double toError, double min, double max, double kp, double ki, double kd,double minMaxPid, float signo);
  int pulseWidth(int angle);
};

#endif