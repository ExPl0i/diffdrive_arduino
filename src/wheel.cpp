#include "diffdrive_arduino/wheel.h"
#include <cmath>  // Подключаем для использования константы M_PI (число π)

// Конструктор класса Wheel (колесо)
Wheel::Wheel(const std::string &wheel_name, int counts_per_rev)
{
  // Вызываем метод setup для инициализации параметров колеса
  setup(wheel_name, counts_per_rev);
}

// Метод настройки колеса
void Wheel::setup(const std::string &wheel_name, int counts_per_rev)
{
  name = wheel_name;  // Присваиваем колесу имя, переданное в аргументах

  // Вычисляем количество радиан на один шаг энкодера
  // (2 * π) - полный оборот (360 градусов), деленный на количество шагов энкодера
  rads_per_count = (2 * M_PI) / counts_per_rev;
}

// Метод вычисления угла поворота колеса на основе данных энкодера
double Wheel::calcEncAngle()
{
  return enc * rads_per_count;  // Угол поворота = число шагов энкодера * радианы за один шаг
}

