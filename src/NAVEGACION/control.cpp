#include "control.h"
#include <cmath>
#include <iostream>
#include <vector>

std::vector<std::vector<double>> waitPoints_coordenadas_a_rectangulares(std::vector<std::vector<double>> ListaCoordenadas) 
{
    double factor = 6371 * 2 * M_PI / 360; // equivalente de un grado terrestre en km

    std::vector<std::vector<double>> Coordenadas_rect;
    Coordenadas_rect.push_back({0, 0});

    for (int i = 0; i < ListaCoordenadas.size() - 1; ++i) {
        std::vector<double> coord = {ListaCoordenadas[i + 1][1] - ListaCoordenadas[0][1], ListaCoordenadas[i + 1][0] - ListaCoordenadas[0][0]};
        coord = {coord[0] * factor * 1000, coord[1] * factor * 1000};
        Coordenadas_rect.push_back(coord);
    }

    return Coordenadas_rect;
}


std::vector<double> darPoseFutura(std::vector<double> PoseActual, double VelocidadActual, double VelocidadAngularActual, std::vector<double> Icc, double dt) {
    if (VelocidadAngularActual == 0) {
        return {PoseActual[0] + VelocidadActual * dt * cos(M_PI * PoseActual[2] / 180), PoseActual[1] + VelocidadActual * dt * sin(M_PI * PoseActual[2] / 180), PoseActual[2]};
    } else {
        double Icc_vec_x = PoseActual[0] - Icc[0];
        double Icc_vec_y = PoseActual[1] - Icc[1];
        std::vector<double> Icc_vec = {Icc_vec_x, Icc_vec_y};
        std::vector<double> Icc_polar = darPolar(Icc_vec[0], Icc_vec[1]);
        double Icc_mag = Icc_polar[0];
        double Icc_ang = Icc_polar[1];
        double AnguloRecorrido = VelocidadAngularActual * M_PI / 180 * dt;
        std::vector<double> xy = darRectangular(Icc_mag, Icc_ang + AnguloRecorrido);
        double x = xy[0] + Icc[0];
        double y = xy[1] + Icc[1];
        double theta = PoseActual[2] + AnguloRecorrido * 180 / M_PI;
        theta = darAngulo_360_180(theta)[1];
        return {x, y, theta};
    }
}

std::vector<double> darVelocidadAngular_ICC(std::vector<double> PoseActual, double AnguloBanqueo, double VelocidadActual) {
    if (AnguloBanqueo == 0) {
        return {0, 0, 0, 0};
    } else {
        double RadioGiro = std::abs((VelocidadActual * VelocidadActual) / (9.81 * std::tan(M_PI * AnguloBanqueo / 180)));
        double VelocidadAngular = (VelocidadActual / RadioGiro) * signo(AnguloBanqueo) * 180 / M_PI;
        double Icc_x = RadioGiro * std::cos(M_PI * PoseActual[2] / 180 + signo(AnguloBanqueo) * M_PI / 2) + PoseActual[0];
        double Icc_y = RadioGiro * std::sin(M_PI * PoseActual[2] / 180 + signo(AnguloBanqueo) * M_PI / 2) + PoseActual[1];
        return {VelocidadAngular, Icc_x, Icc_y, RadioGiro};
    }
}

std::vector<double> darPolar(double x, double y) {
    return {std::hypot(x, y), std::atan2(y, x)};
}

std::vector<double> darRectangular(double mag, double ang) {
    return {mag * std::cos(M_PI * ang / 180), mag * std::sin(M_PI * ang / 180)};
}

int signo(double x) {
    return (x >= 0) ? 1 : -1;
}

std::vector<double> darPendiente(std::vector<double> Punto1, std::vector<double> Punto2) {
    double m, b;
    if (Punto2[0] - Punto1[0] == 0) {
        m = 9999;
    } else {
        m = (Punto2[1] - Punto1[1]) / (Punto2[0] - Punto1[0]);
    }
    b = Punto2[1] - m * Punto2[0];
    return {m, b};
}

std::vector<double> proyectarPunto(std::vector<double> Punto, double Pendiente, double intersepto) {
    double xi = -(intersepto * Pendiente - Pendiente * Punto[1] - Punto[0]) / ((Pendiente * Pendiente) + 1);
    double yi = Pendiente * xi + intersepto;
    return {xi, yi};
}

std::vector<double> darPuntoIntermedio(std::vector<double> PuntoProyectado, double angulo, double Pendiente, double intersepto, double k_distanciaAuxiliar, std::vector<double> PuntoObjetivo) {
    double x1 = PuntoProyectado[0] + k_distanciaAuxiliar * std::cos(std::atan(Pendiente));
    double y1 = Pendiente * x1 + intersepto;
    double x2 = PuntoProyectado[0] - k_distanciaAuxiliar * std::cos(std::atan(Pendiente));
    double y2 = Pendiente * x2 + intersepto;
    if (std::hypot(PuntoObjetivo[0] - x1, PuntoObjetivo[1] - y1) < std::hypot(PuntoObjetivo[0] - x2, PuntoObjetivo[1] - y2)) {
        return {x1, y1};
    } else {
        return {x2, y2};
    }
}

double darCorreccionAngular(double AnguloActual, double AnguloDeseado) {
    double ErrorAngular = AnguloDeseado - AnguloActual;
    if (std::abs(ErrorAngular) < 180) {
        return ErrorAngular;
    } else {
        return ErrorAngular - 360 * signo(ErrorAngular);
    }
}

double darDerivada(double ValorActual, double ValorAnterior, double dt) {
    return (ValorActual - ValorAnterior) / dt;
}

std::vector<double> darControlAleron(double Error, double DerivadaError, double kp, double kd, double cte_saturacion, double AnguloBanqueo) {
    double controlador;
    if (std::abs(AnguloBanqueo) < 25) {
        controlador = -kp * Error + kd * DerivadaError;
        if (std::abs(controlador) > 100) {
            controlador = 100 * signo(controlador);
        }
    } else if (std::abs(AnguloBanqueo) < 30) {
        controlador = -kp * Error + kd * DerivadaError;
        if (std::abs(controlador) > 100) {
            controlador = 100 * signo(controlador);
        }
        double penalizacion = (std::abs(AnguloBanqueo) - 25) / 5;
        controlador = controlador * (1 - penalizacion);
    } else {
        controlador = cte_saturacion * AnguloBanqueo / 100;
    }
    return {controlador, -controlador};
}

double darAnguloBanqueo(double PorcentajeAleronIzquierdo, double AnguloBanqueoAnterior, double dt) {
    return (-1 * 1.2 * PorcentajeAleronIzquierdo * dt + AnguloBanqueoAnterior);
}

std::vector<double> darAngulo_360_180(double anguloLeido) {
    double ang_encontrado_360, ang_encontrado_180;
    if (std::abs(anguloLeido) > 360) {
        double ang_encontrado_360 = std::modf(anguloLeido / 360, &ang_encontrado_360) * 360 * signo(anguloLeido);
        ang_encontrado_180 = (std::abs(ang_encontrado_360) <= 180) ? ang_encontrado_360 : ang_encontrado_360 - 360;
    } else {
        ang_encontrado_360 = anguloLeido;
        if (std::abs(anguloLeido) < 180) {
            ang_encontrado_180 = ang_encontrado_360;
        } else {
            ang_encontrado_180 = ang_encontrado_360 - 360 * signo(ang_encontrado_360);
        }
    }
    return {ang_encontrado_360, ang_encontrado_180};
}

double darErrorPosicion(const std::vector<double>& PuntoActual, const std::vector<double>& PuntoFinal) {
    double diffX = PuntoFinal[0] - PuntoActual[0];
    double diffY = PuntoFinal[1] - PuntoActual[1];
    
    // Llamar a la función darPolar para obtener la magnitud y el ángulo
    std::vector<double> polarResult = darPolar(diffX, diffY);
    
    // La magnitud se encuentra en el primer elemento del vector
    return polarResult[0];
}

