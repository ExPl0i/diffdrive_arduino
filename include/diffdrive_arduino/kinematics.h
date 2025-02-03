#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <Arduino.h>
#include <math.h>

// Структура, содержащая углы рулевых осей и скорости для каждого колеса.
struct VehicleDynamics {
  double theta_f; // Угол передней оси [градусы]
  double theta_r; // Угол задней оси [градусы]
  double V_FL;    // Скорость переднего левого колеса [м/с]
  double V_FR;    // Скорость переднего правого колеса [м/с]
  double V_RL;    // Скорость заднего левого колеса [м/с]
  double V_RR;    // Скорость заднего правого колеса [м/с]
};

// Функция расчёта динамики автомобиля по заданной скорости центра масс V и угловой скорости omega.
// Параметры L_f и L_r – расстояния от центра масс до передней и задней осей, W – ширина между колёсами.
inline VehicleDynamics computeWheelSpeedsAndAngles(double V, double omega, double L_f, double L_r, double W) {
  VehicleDynamics result;
  
  // Если вращение отсутствует, все углы равны нулю, а скорости всех колёс равны V.
  if (fabs(omega) < 1e-6) {
    result.theta_f = 0;
    result.theta_r = 0;
    result.V_FL = V;
    result.V_FR = V;
    result.V_RL = V;
    result.V_RR = V;
    return result;
  }

  double R = V / omega; // Радиус поворота центра масс
  
  // Вычисляем углы рулевых осей (в градусах)
  result.theta_f = atan2(L_f, R) * 180.0 / PI;
  result.theta_r = atan2(L_r, R) * 180.0 / PI;
  
  // Определяем эффективные радиусы для каждого колеса
  double R_FL = R - W / 2;
  double R_FR = R + W / 2;
  double R_RL = R - W / 2;
  double R_RR = R + W / 2;
  
  // Вычисляем скорости колёс
  result.V_FL = omega * R_FL;
  result.V_FR = omega * R_FR;
  result.V_RL = omega * R_RL;
  result.V_RR = omega * R_RR;
  
  return result;
}

#endif // KINEMATICS_H
