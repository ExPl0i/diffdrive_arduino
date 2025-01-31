#include "diffdrive_arduino/steering.h"
#include <cmath>  // Подключаем для использования константы M_PI (число π)

// Конструктор класса Steering (повротная ось)
Steering::Steering(const std::string &steering_name, double steering_angle)
{
  // Вызываем метод setup для инициализации параметров поворотной оси
  setup(steering_name, steering_angle);
}

// Метод настройки колеса
void Steering::setup(const std::string &steering_name, double steering_angle)
{
  name = steering_name;  // Присваиваем поворотной оси имя, переданное в аргументах

  // Вычисляем количество радиан на угол поворота
  steering_angle = std::atan2(L_f, R) * 180.0 / M_PI;
}

// Метод вычисления угла поворота колеса на основе данных энкодера
double Wheel::calcEncAngle()
{
  return enc * rads_per_count;  // Угол поворота = число шагов энкодера * радианы за один шаг
}

