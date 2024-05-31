#ifndef ReguladorServos_h
#define ReguladorServos_h

#include <Arduino.h>

class ReguladorServos
{
public:
  double CalcularPid(double actual, double PosicionDeseada, double priError, double toError, double min, double max, double kp, double ki, double kd, int minMaxPid, bool signo);
  int pulseWidth(int angle);
  void updateChannelsAuto(int servo0Value, int servo1Value, float rollValue, float pitchValue);
  void updateChannels(int servo0Value, int servo1Value, int servo2Value, int servo3Value, float rollValue, float pitchValue);
  void managePlaneMode(int servo0Value, int servo1Value, int servo2Value, int servo3Value, float rollValue, float pitchValue);
  void print_channels(int servo0Value, int servo1Value, int servo2Value, int servo3Value);
};

#endif