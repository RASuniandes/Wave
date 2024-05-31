#include "ReguladorServos.h"
// #include "telemetria/FlySky.h"

// ReguladorServos::ReguladorServos()
// {
//   // Inicializa los servos
// }

double ReguladorServos::CalcularPID(double actual, double PosicionDeseada, double priError, double toError, double min, double max, double kp, double ki, double kd, double minMaxPid, bool signo)
{

  // Funcion que genera el angulo que debe tener los servos para regular la posicion (yaw, pitch, roll) mediante el PID

  double error = PosicionDeseada - actual;
  toError += error;
  double Pvalue = error * kp;
  double Ivalue = toError * ki;
  double Dvalue = (error - priError) * kd;
  double PIDVal = Pvalue + Ivalue + Dvalue;

  priError = error;

  // Limitar el valor de PIDVal dentro del rango de -90 a 90

  if (PIDVal > minMaxPid)
  {
    PIDVal = minMaxPid;
  }
  if (PIDVal < -minMaxPid)
  {
    PIDVal = -minMaxPid;
  }

  double valToreturn = 0;

  // Regulando la orientacion para coincidir con los servos

  if (signo)
  {
    valToreturn = map(PIDVal, -minMaxPid, minMaxPid, min, max);
  }
  else
  {
    valToreturn = map(PIDVal, -minMaxPid, minMaxPid, max, min);
  }

  // Limitar el valor de retorno dentro de los límites del servo (min a max)
  if (valToreturn > max)
  {
    valToreturn = max;
  }
  if (valToreturn < min)
  {
    valToreturn = min;
  }

  return valToreturn;
}

int ReguladorServos::pulseWidth(int angle)
{

  // Funcion a cual convierte un angulo entre 0 y 180 a ancho de pulso (Tipo de dato que leen los servos)

  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}

// void ReguladorServos::updateChannelsAuto()
// {
//   float pidRoll = CalcularPid(rollValue, PosicionDeseadaRoll, priErrorRoll, toErrorRoll, min_limit_c1, max_limit_c1, kpRoll, kiRoll, kdRoll, 60, 1);
//   servo0Value = pulseWidth(pidRoll);

//   float pidPitch = CalcularPid(pitchValue, PosicionDeseadaPitch, priErrorPitch, toErrorPitch, min_limit_c2, max_limit_c2, kpPitch, kiPitch, kdPitch, 30, 0);
//   servo1Value = pulseWidth(pidPitch);
// }