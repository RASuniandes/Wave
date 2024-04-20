#ifndef CONTROL_H
#define CONTROL_H

#include <vector>

std::vector<std::vector<double>> waitPoints_coordenadas_a_rectangulares(std::vector<std::vector<double>> ListaCoordenadas);
std::vector<double> darPoseFutura(std::vector<double> PoseActual, double VelocidadActual, double VelocidadAngularActual, std::vector<double> Icc, double dt);
std::vector<double> darVelocidadAngular_ICC(std::vector<double> PoseActual, double AnguloBanqueo, double VelocidadActual);
std::vector<double> darPolar(double x, double y);
std::vector<double> darRectangular(double mag, double ang);
int signo(double x);
std::vector<double> darPendiente(std::vector<double> Punto1, std::vector<double> Punto2);
std::vector<double> proyectarPunto(std::vector<double> Punto, double Pendiente, double interescto);
std::vector<double> darPuntoIntermedio(std::vector<double> PuntoProyectado, double angulo, double Pendiente, double interescto, double k_distanciaAuxiliar, std::vector<double> PuntoObjetivo);
double darCorreccionAngular(double AnguloActual, double AnguloDeseado);
double darDerivada(double ValorActual, double ValorAnterior, double dt);
std::vector<double> darControlAleron(double Error, double DerivadaError, double kp, double kd, double cte_saturacion, double AnguloBanqueo);
double darAnguloBanqueo(double PorcentajeAleronIzquierdo, double AnguloBanqueoAnterior, double dt);
std::vector<double> darAngulo_360_180(double anguloLeido);
double darErrorPosicion(std::vector<double> PuntoActual, std::vector<double> PuntoFinal);

#endif