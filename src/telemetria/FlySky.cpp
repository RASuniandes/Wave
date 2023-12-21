#include "FlySky.h"

FlySky::FlySky(int pin1, int pin2, int pin3, int pin4, int pin5) {
  ch1_pin = pin1;
  ch2_pin = pin2;
  ch3_pin = pin3;
  ch4_pin = pin4;
  ch5_pin = pin5;
  automatic = false;
  pinMode(ch1_pin, INPUT);
  pinMode(ch2_pin, INPUT);
  pinMode(ch3_pin, INPUT);
  pinMode(ch4_pin, INPUT);
  pinMode(ch5_pin, INPUT);
}

int FlySky::readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue) {
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

bool FlySky::readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

int FlySky::getChannel1Value() {
  return readChannel(ch1_pin, 0, 180, 90);
}

int FlySky::getChannel2Value() {
  return readChannel(ch2_pin, 0, 100, 90);
}

int FlySky::getChannel3Value() {
  return readChannel(ch3_pin, 0, 180, 90);
}

int FlySky::getChannel4Value() {
  return readChannel(ch4_pin, 0, 180, 90);
}

bool FlySky::getAutomaticFly() {
  return automatic;
}

void FlySky::updateAutomaticFly() {
  automatic = digitalRead(ch5_pin);
}