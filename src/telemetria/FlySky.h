#ifndef FlySky_h
#define FlySky_h

#include <Arduino.h>

class FlySky {
public:
  FlySky(int pin1, int pin2, int pin3, int pin4, int pin5,int pin6);

  int getChannel1Value(int minLimit, int maxLimit, int defaultValue);
  int getChannel2Value(int minLimit, int maxLimit, int defaultValue);
  int getChannel3Value(int minLimit, int maxLimit, int defaultValue);
  int getChannel4Value(int minLimit, int maxLimit, int defaultValue);
  bool getChannel6Value();
  bool getAutomaticFly();
  void updateAutomaticFly();

  int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue);
  bool readSwitch(byte channelInput, bool defaultValue);

private:
  int ch1_pin;
  int ch2_pin;
  int ch3_pin;
  int ch4_pin;
  int ch5_pin;
  int ch6_pin;
  bool automatic;
  bool channel6;
};

#endif