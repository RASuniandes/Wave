#ifndef ReguladorServos_h
#define ReguladorServos_h

#include <Arduino.h>

class ReguladorServos
{
public:
  void BeginServos();
  double CalcularPid(double actual, double PosicionDeseada, double priError, double toError, double min, double max, double kp, double ki, double kd, int minMaxPid, bool signo);
  int pulseWidth(int angle);
  void updateChannelsAuto(float rollValue, float pitchValue);
  void updateChannels(float rollValue, float pitchValue);
  void managePlaneMode(float rollValue, float pitchValue);
  void print_channels();
  int getservo0Value() const { return servo0Value; }
  int getservo1Value() const { return servo1Value; }
  int getservo2Value() const { return servo2Value; }
  int getservo3Value() const { return servo3Value; }
  int getservo4Value() const { return servo4Value; }

private:
  float PosicionDeseadaYaw = 0;
  float PosicionDeseadaPitch = 0;
  float PosicionDeseadaRoll = 0;
  int servo0Value;
  int servo1Value;
  int servo2Value;
  int servo3Value;
  int servo4Value;
};

#endif