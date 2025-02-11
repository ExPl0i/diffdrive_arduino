#ifndef DIFFDRIVE_ARDUINO_REAL_ROBOT_H
#define DIFFDRIVE_ARDUINO_REAL_ROBOT_H

// Подключение стандартных библиотек
#include <cstring>    // Для работы со строками C
#include <chrono>     // Для работы с временными метками

// Подключение библиотек ROS2
#include "rclcpp/rclcpp.hpp"                     // Основной заголовок для создания узлов ROS2
#include "geometry_msgs/msg/twist.hpp"           // Для работы с сообщениями, содержащими линейную и угловую скорость
#include "nav_msgs/msg/odometry.hpp"             // Для работы с сообщениями одометрии (положение, ориентация и скорость)

// Подключение интерфейсов ROS2 Hardware Interface
#include "hardware_interface/base_interface.hpp" // Базовый интерфейс для реализации оборудования
#include "hardware_interface/system_interface.hpp" // Интерфейс системы для оборудования
#include "hardware_interface/handle.hpp"         // Для работы с "ручками" (handles) оборудования
#include "hardware_interface/hardware_info.hpp"    // Структура с информацией об оборудовании
#include "hardware_interface/types/hardware_interface_return_values.hpp" // Определения возвращаемых значений методов
#include "hardware_interface/types/hardware_interface_status_values.hpp" // Определения статусов оборудования

// Подключение пользовательских заголовочных файлов
#include "config.h"           // Конфигурация (структура Config), содержащая параметры оборудования
#include "wheel.h"            // Класс для работы с колесами (подсчёт углов, скорости и т.д.)
#include "steering.h"         // Класс для управления поворотными осями (стиринг)
#include "arduino_comms.h"    // Класс для связи с Arduino (отправка и приём команд)

// Определяем тип возвращаемого значения для методов (например, OK или ERROR)
using hardware_interface::return_type;

// Класс DiffDriveArduino реализует аппаратный интерфейс для робота с дифференциальным приводом,
// управляемого через Arduino. Класс наследуется от rclcpp::Node для работы как узел ROS2
// и от BaseInterface для поддержки аппаратного интерфейса.
class DiffDriveArduino : public rclcpp::Node,
                         public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  // Конструктор класса: инициализирует узел ROS2 и прочие компоненты
  DiffDriveArduino();

  // Виртуальный деструктор (по умолчанию)
  virtual ~DiffDriveArduino() = default;

  // Методы жизненного цикла оборудования, переопределённые из базового интерфейса:
  
  // Метод для конфигурации оборудования, принимает информацию о параметрах оборудования.
  return_type configure(const hardware_interface::HardwareInfo & info) override;

  // Метод для запуска оборудования (например, инициализация связи, отправка стартовых команд).
  return_type start() override;
  
  // Метод для остановки оборудования.
  return_type stop() override;

  // Методы для обмена данными с оборудованием:
  
  // Метод для чтения данных с оборудования (например, считывание значений энкодеров).
  return_type read() override;

  // Callback для обработки входящих сообщений с командами скорости (например, топик /cmd_vel)
  return_type cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  // Метод для отправки команд на Arduino (скорости для левого и правого двигателя, углы поворотных осей)
  return_type sendCommandsToArduino(double v_left, double v_right, double f_angle, double r_angle);

  // Метод для обновления одометрии робота и публикации соответствующего сообщения
  return_type updateOdometry();


private:
  // Паблишер для публикации сообщений одометрии (напр., топик "odom")
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

  // (Опционально) подписчик для получения команд скорости (например, топик /cmd_vel)
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;

  // Параметры, загружаемые из конфигурационного файла (структура Config)
  Config cfg_;
  // Объект для связи с Arduino
  ArduinoComms arduino_;
   
  // Переменные для хранения одометрии: положение (x, y) и ориентация (theta)
  double x_, y_, theta_;
  // Параметры робота: радиус колеса и расстояние между колесами (база)
  double wheel_radius_; 
  double wheel_base_; 
  
  // Объекты для управления колесами:
  Wheel fl_wheel_;  // Переднее левое колесо
  Wheel fr_wheel_;  // Переднее правое колесо
  Wheel rl_wheel_;  // Заднее левое колесо
  Wheel rr_wheel_;  // Заднее правое колесо

  // Объекты для управления поворотными осями (стиринг)
  Steering f_steering_;  // Передняя ось (стиринг)
  Steering r_steering_;  // Задняя ось (стиринг)

  // Статус оборудования (например, UNKNOWN, CONFIGURED, STARTED, STOPPED)
  hardware_interface::status status_ = hardware_interface::status::UNKNOWN;

  // Временные метки для вычисления разницы времени между итерациями (для расчёта скоростей)
  std::chrono::time_point<std::chrono::system_clock> time_;
  // Временная метка последнего обновления одометрии (в формате ROS2)
  rclcpp::Time last_odom_time_;
};

#endif // DIFFDRIVE_ARDUINO_REAL_ROBOT_H
