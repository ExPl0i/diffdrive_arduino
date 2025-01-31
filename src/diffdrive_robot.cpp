#include <ros/ros.h>  // Подключение библиотеки ROS
#include <controller_manager/controller_manager.h>  // Менеджер контроллеров
#include "diffdrive_arduino/diffdrive_arduino.h"  // Подключение класса управления роботом

int main(int argc, char **argv)
{
  // Инициализация ROS-ноды с именем "diffdrive_robot"
  ros::init(argc, argv, "diffdrive_robot");

  // Создание приватного узла (NodeHandle) для работы с параметрами ROS
  ros::NodeHandle n("~");

  // Создаем объект структуры конфигурации для хранения параметров робота
  DiffDriveArduino::Config robot_cfg;

  // Получение параметров из ROS Parameter Server
  // Если параметры не найдены, остаются значения по умолчанию из структуры
  n.getParam("front_steering_name", robot_cfg.front_steering_name); // Имя передней поворотной оси
  n.getParam("rear_steering_name", robot_cfg.reae_steering_name); // Имя задней поворотной оси
  n.getParam("front_left_wheel_name", robot_cfg.front_left_wheel_name);  // Имя переднего левого колеса
  n.getParam("front_right_wheel_name", robot_cfg.front_right_wheel_name); // Имя переднего правого колеса
  n.getParam("rear_left_wheel_name", robot_cfg.rear_left_wheel_name);  // Имя заднего левого колеса
  n.getParam("rear_right_wheel_name", robot_cfg.rear_right_wheel_name); // Имя заднего правого колеса
  n.getParam("baud_rate", robot_cfg.baud_rate);  // Скорость передачи данных по Serial (если используется)
  n.getParam("device", robot_cfg.device);  // Устройство подключения (например, /dev/ttyUSB0)
  n.getParam("enc_counts_per_rev", robot_cfg.enc_counts_per_rev);  // Количество шагов энкодера за один оборот
  n.getParam("robot_loop_rate", robot_cfg.loop_rate);  // Частота цикла управления (Гц)

  // Создание объекта управления роботом с переданными параметрами
  DiffDriveArduino robot(robot_cfg);

  // Создание менеджера контроллеров и привязка его к объекту робота
  controller_manager::ControllerManager cm(&robot);

  // Создание асинхронного спиннера для обработки обратных вызовов в отдельном потоке
  ros::AsyncSpinner spinner(1);
  spinner.start();  // Запуск обработки обратных вызовов в фоновом режиме

  // Фиксируем время начала работы
  ros::Time prevTime = ros::Time::now();

  // Устанавливаем частоту цикла управления (10 Гц)
  ros::Rate loop_rate(10);

  // Основной цикл работы контроллера
  while (ros::ok())  // Пока нода ROS работает
  {
    robot.read();  // Читаем данные с сенсоров (например, энкодеры)
    cm.update(robot.get_time(), robot.get_period());  // Обновляем контроллер
    robot.write();  // Отправляем команды моторам

    loop_rate.sleep();  // Ждем до следующего такта (для работы в 10 Гц)
  }
}

