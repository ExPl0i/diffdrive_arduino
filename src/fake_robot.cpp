// #include "diffdrive_arduino/fake_robot.h"

// #include "hardware_interface/types/hardware_interface_type_values.hpp"

// // Конструктор класса FakeRobot
// FakeRobot::FakeRobot()
//   : logger_(rclcpp::get_logger("FakeRobot")) // Инициализация логгера
// {}

// // Метод конфигурации фейкового робота
// return_type FakeRobot::configure(const hardware_interface::HardwareInfo &info)
// {
//   // Вызов стандартного метода конфигурации и проверка успешности
//   if (configure_default(info) != return_type::OK) {
//     return return_type::ERROR;
//   }

//   RCLCPP_INFO(logger_, "Configuring..."); // Логируем начало конфигурации

//   // Запоминаем текущее время
//   time_ = std::chrono::system_clock::now();

//   // Получаем параметры имен колес из конфигурации оборудования
//   cfg_.left_wheel_name = info.hardware_parameters.at("left_wheel_name");
//   cfg_.right_wheel_name = info.hardware_parameters.at("right_wheel_name");

//   // Настраиваем колеса
//   // Примечание: количество шагов энкодера за оборот неважно, так как фейковый робот не использует энкодеры
//   l_wheel_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
//   r_wheel_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

//   RCLCPP_INFO(logger_, "Finished Configuration"); // Логируем завершение конфигурации

//   status_ = hardware_interface::status::CONFIGURED; // Устанавливаем статус "СКОНФИГУРИРОВАН"
//   return return_type::OK;
// }

// // Экспорт интерфейсов состояния оборудования (состояние скорости и позиции для каждого колеса)
// std::vector<hardware_interface::StateInterface> FakeRobot::export_state_interfaces()
// {
//   std::vector<hardware_interface::StateInterface> state_interfaces;

//   // Добавляем интерфейсы скорости и позиции для левого колеса
//   state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.vel));
//   state_interfaces.emplace_back(hardware_interface::StateInterface(l_wheel_.name, hardware_interface::HW_IF_POSITION, &l_wheel_.pos));

//   // Добавляем интерфейсы скорости и позиции для правого колеса
//   state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.vel));
//   state_interfaces.emplace_back(hardware_interface::StateInterface(r_wheel_.name, hardware_interface::HW_IF_POSITION, &r_wheel_.pos));

//   return state_interfaces;
// }

// // Экспорт интерфейсов команд (команды скорости для каждого колеса)
// std::vector<hardware_interface::CommandInterface> FakeRobot::export_command_interfaces()
// {
//   std::vector<hardware_interface::CommandInterface> command_interfaces;

//   // Добавляем командные интерфейсы для управления скоростью колес
//   command_interfaces.emplace_back(hardware_interface::CommandInterface(l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &l_wheel_.cmd));
//   command_interfaces.emplace_back(hardware_interface::CommandInterface(r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &r_wheel_.cmd));

//   return command_interfaces;
// }

// // Метод запуска контроллера
// return_type FakeRobot::start()
// {
//   RCLCPP_INFO(logger_, "Starting Controller..."); // Логируем запуск контроллера
//   status_ = hardware_interface::status::STARTED; // Устанавливаем статус "ЗАПУЩЕН"

//   return return_type::OK;
// }

// // Метод остановки контроллера
// return_type FakeRobot::stop()
// {
//   RCLCPP_INFO(logger_, "Stopping Controller..."); // Логируем остановку контроллера
//   status_ = hardware_interface::status::STOPPED; // Устанавливаем статус "ОСТАНОВЛЕН"

//   return return_type::OK;
// }

// // Метод чтения данных (эмулируем работу энкодеров)
// hardware_interface::return_type FakeRobot::read()
// {
//   // TODO: Исправить вычисление разницы времени

//   // Вычисляем разницу времени с последнего вызова
//   auto new_time = std::chrono::system_clock::now();
//   std::chrono::duration<double> diff = new_time - time_;
//   double deltaSeconds = diff.count();
//   time_ = new_time;

//   // Эмулируем обновление позиции колес на основе их текущей скорости
//   l_wheel_.pos = l_wheel_.pos + l_wheel_.vel * deltaSeconds;
//   r_wheel_.pos = r_wheel_.pos + r_wheel_.vel * deltaSeconds;

//   return return_type::OK;
// }

// // Метод записи команд в оборудование (эмулируем передачу команд на двигатели)
// hardware_interface::return_type FakeRobot::write()
// {
//   // Присваиваем колесам переданные управляющие команды (скорости)
//   l_wheel_.vel = l_wheel_.cmd;
//   r_wheel_.vel = r_wheel_.cmd;

//   return return_type::OK;
// }

// // Подключение класса в систему плагинов ROS 2
// #include "pluginlib/class_list_macros.hpp"

// PLUGINLIB_EXPORT_CLASS(
//   FakeRobot,
//   hardware_interface::SystemInterface
// )

