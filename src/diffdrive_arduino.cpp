#include "diffdrive_arduino/diffdrive_arduino.h"        // Подключение заголовочного файла данного класса
#include "hardware_interface/types/hardware_interface_type_values.hpp"  // Типы значений для аппаратного интерфейса
#include "rclcpp/rclcpp.hpp"                              // Основной заголовок ROS2
#include "geometry_msgs/msg/twist.hpp"                   // Для работы с сообщениями типа Twist (команды скорости)
#include "nav_msgs/msg/odometry.hpp"                     // Для работы с сообщениями одометрии
#include <tf2/LinearMath/Quaternion.h>                   // Для работы с кватернионами (представление ориентации)
#include <cmath>                                         // Для математических функций (cos, sin и т.д.)

// ----------------------------------------------------------------------------------------
// Конструктор класса DiffDriveArduino
// ----------------------------------------------------------------------------------------
// В конструкторе происходит инициализация узла ROS2 с именем "diff_drive_arduino", а также
// создаются необходимые паблишер (для одометрии) и (при необходимости) подписчик (для команд скорости).
// Кроме того, инициализируются переменные времени и начальное состояние робота.
DiffDriveArduino::DiffDriveArduino()
: Node("diff_drive_arduino")  // Инициализация узла ROS2 с заданным именем
{
    // Создаём паблишер для одометрии на топике "odom" с размером очереди 10 сообщений.
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    
    // (При необходимости) создаём подписчика на команды скорости (например, топик /cmd_vel).
    // Раскомментируйте и настройте, если требуется:
    // cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
    //     "cmd_vel", 10,
    //     std::bind(&DiffDriveArduino::cmdVelCallback, this, std::placeholders::_1)
    // );
    
    // Инициализируем временные метки:
    // - time_ используется для вычисления промежутка времени между вызовами метода read()
    // - last_odom_time_ используется для расчёта дельты времени при обновлении одометрии
    time_ = std::chrono::system_clock::now();
    last_odom_time_ = this->now();
    
    // Устанавливаем начальные координаты и ориентацию робота (начальное положение: 0,0,0)
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    
    // Задаём базовые параметры робота:
    wheel_radius_ = 0.08; // Радиус колеса в метрах
    wheel_base_   = 0.234; // Расстояние между левыми и правыми колесами (база робота) в метрах
}

// ----------------------------------------------------------------------------------------
// Метод configure
// ----------------------------------------------------------------------------------------
// Метод для конфигурации оборудования. Здесь происходит считывание параметров из структуры info,
// настройка объектов управления (колёса, стиринг) и установка соединения с Arduino.
return_type DiffDriveArduino::configure(const hardware_interface::HardwareInfo &info)
{
    // Выводим информационное сообщение о начале конфигурации.
    RCLCPP_INFO(this->get_logger(), "Configuring...");

    // Задаём примерные параметры для поворотных осей:
    float f_axle_offset = 0.262f;    // Смещение передней оси
    float f_track_width = 0.234f;      // Расстояние между колесами передней оси
    float f_steering_sign = 1.0f;      // Знак для передней оси (обычно +1)

    float r_axle_offset = 0.262f;      // Смещение задней оси
    float r_track_width = 0.234f;      // Расстояние между колесами задней оси
    float r_steering_sign = -1.0f;     // Знак для задней оси (обычно -1)

    // Обновляем временную метку
    time_ = std::chrono::system_clock::now();

    // Считываем параметры оборудования из предоставленной структуры info.
    try
    {
        cfg_.front_left_wheel_name  = info.hardware_parameters.at("front_left_wheel_name");
        cfg_.front_right_wheel_name = info.hardware_parameters.at("front_right_wheel_name");
        cfg_.rear_left_wheel_name   = info.hardware_parameters.at("rear_left_wheel_name");
        cfg_.rear_right_wheel_name  = info.hardware_parameters.at("rear_right_wheel_name");
        cfg_.front_steering_name     = info.hardware_parameters.at("front_steering_name");
        cfg_.rear_steering_name      = info.hardware_parameters.at("rear_steering_name");
        cfg_.loop_rate              = std::stof(info.hardware_parameters.at("loop_rate"));
        cfg_.host                   = info.hardware_parameters.at("host");
        cfg_.port                   = std::stoi(info.hardware_parameters.at("port"));
        cfg_.enc_counts_per_rev     = std::stoi(info.hardware_parameters.at("enc_counts_per_rev"));
    }
    // Обработка исключений, если какой-либо параметр отсутствует или имеет неверный формат.
    catch (const std::invalid_argument& e)
    {
        if (info.hardware_parameters.find("front_left_wheel_name") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Missing parameter: front_left_wheel_name");
        }
        else if (info.hardware_parameters.find("front_right_wheel_name") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Missing parameter: front_right_wheel_name");
        }
        else if (info.hardware_parameters.find("rear_left_wheel_name") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Missing parameter: rear_left_wheel_name");
        }
        else if (info.hardware_parameters.find("rear_right_wheel_name") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Missing parameter: rear_right_wheel_name");
        }
        else if (info.hardware_parameters.find("front_steering_name") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Missing parameter: front_steering_name");
        }
        else if (info.hardware_parameters.find("rear_steering_name") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Missing parameter: rear_steering_name");
        }
        else if (info.hardware_parameters.find("loop_rate") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Missing parameter: loop_rate");
        }
        else if (info.hardware_parameters.find("host") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Missing parameter: host");
        }
        else if (info.hardware_parameters.find("port") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Missing parameter: port");
        }
        else if (info.hardware_parameters.find("enc_counts_per_rev") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(this->get_logger(), "Missing parameter: enc_counts_per_rev");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid argument for parameter: %s", e.what());
        }
        return return_type::ERROR;
    }
    catch (const std::out_of_range& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Out of range for parameter: %s", e.what());
        return return_type::ERROR;
    }

    // Настройка объектов колес и поворотных осей с использованием полученных параметров:
    fl_wheel_.setup(cfg_.front_left_wheel_name, cfg_.enc_counts_per_rev);
    fr_wheel_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);
    rl_wheel_.setup(cfg_.rear_left_wheel_name, cfg_.enc_counts_per_rev);
    rr_wheel_.setup(cfg_.rear_right_wheel_name, cfg_.enc_counts_per_rev);
    f_steering_.setup(cfg_.front_steering_name, f_axle_offset, f_track_width, f_steering_sign);
    r_steering_.setup(cfg_.rear_steering_name, r_axle_offset, r_track_width, r_steering_sign);

    // Настройка подключения к Arduino
    arduino_.setup(cfg_.host, cfg_.port);

    // Выводим сообщение о завершении конфигурации
    RCLCPP_INFO(this->get_logger(), "Finished Configuration");

    // Обновляем статус оборудования до состояния CONFIGURED
    status_ = hardware_interface::status::CONFIGURED;
    return return_type::OK;
}

// ----------------------------------------------------------------------------------------
// Метод start
// ----------------------------------------------------------------------------------------
// Метод запускает контроллер, отправляя начальные команды на Arduino (например, пустое сообщение
// для установления связи и настройку PID-параметров).
return_type DiffDriveArduino::start()
{
    RCLCPP_INFO(this->get_logger(), "Starting Controller...");

    // Отправляем пустое сообщение на Arduino для инициализации связи.
    arduino_.sendEmptyMsg();
    // Устанавливаем PID-параметры для управления двигателями (примерные значения).
    arduino_.setPidValues(30, 30, 0, 100);

    // Обновляем статус оборудования до состояния STARTED.
    status_ = hardware_interface::status::STARTED;

    return return_type::OK;
}

// ----------------------------------------------------------------------------------------
// Метод stop
// ----------------------------------------------------------------------------------------
// Метод останавливает контроллер, обновляя статус оборудования до STOPPED.
// Здесь можно добавить дополнительную логику для корректной остановки оборудования.
return_type DiffDriveArduino::stop()
{
    RCLCPP_INFO(this->get_logger(), "Stopping Controller...");
    status_ = hardware_interface::status::STOPPED;
    return return_type::OK;
}

// ----------------------------------------------------------------------------------------
// Метод read
// ----------------------------------------------------------------------------------------
// Метод считывает данные с оборудования (например, значения энкодеров), вычисляет угловые
// скорости колес на основе разницы между текущим и предыдущим значением, а также учитывает
// прошедшее время (deltaSeconds).
return_type DiffDriveArduino::read()
{
    // Получаем текущее системное время и вычисляем разницу (deltaSeconds) с предыдущим вызовом.
    auto new_time = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = new_time - time_;
    double deltaSeconds = diff.count();
    time_ = new_time;

    // Проверяем, установлено ли соединение с Arduino. Если нет, возвращаем ошибку.
    if (!arduino_.connected())
    {
        return return_type::ERROR;
    }

    // Чтение значений энкодеров для каждого из четырёх колес через объект ArduinoComms.
    arduino_.readEncoderValues(fl_wheel_.enc, fr_wheel_.enc, rl_wheel_.enc, rr_wheel_.enc);

    // Для каждого колеса обновляем значение положения (угловое положение) и вычисляем скорость
    // как разницу между новым и предыдущим положением, делённую на deltaSeconds.
    double pos_prev = fl_wheel_.pos;
    fl_wheel_.pos = fl_wheel_.calcEncAngle();
    fl_wheel_.vel = (fl_wheel_.pos - pos_prev) / deltaSeconds;

    pos_prev = fr_wheel_.pos;
    fr_wheel_.pos = fr_wheel_.calcEncAngle();
    fr_wheel_.vel = (fr_wheel_.pos - pos_prev) / deltaSeconds;

    pos_prev = rl_wheel_.pos;
    rl_wheel_.pos = rl_wheel_.calcEncAngle();
    rl_wheel_.vel = (rl_wheel_.pos - pos_prev) / deltaSeconds;

    pos_prev = rr_wheel_.pos;
    rr_wheel_.pos = rr_wheel_.calcEncAngle();
    rr_wheel_.vel = (rr_wheel_.pos - pos_prev) / deltaSeconds;

    return return_type::OK;
}

// ----------------------------------------------------------------------------------------
// Метод cmdVelCallback
// ----------------------------------------------------------------------------------------
// Callback для обработки входящих сообщений типа geometry_msgs::msg::Twist.
// Извлекает линейную и угловую скорость, обновляет поворотные оси и вычисляет
// скорости для левой и правой стороны робота, после чего отправляет команды на Arduino.
return_type DiffDriveArduino::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Задаём расстояние между колесами (примерное значение)
    double wheel_separation = 0.234;

    // Извлекаем линейную и угловую скорость из сообщения
    double linear_vel  = msg->linear.x;
    double angular_vel = msg->angular.z;

    // Обновляем состояние поворотных осей (стиринг) на основе поступивших скоростей
    f_steering_.update(linear_vel, angular_vel);
    r_steering_.update(linear_vel, angular_vel);
    double f_angle = f_steering_.getSteeringAngle();
    double r_angle = r_steering_.getSteeringAngle();

    // Вычисляем скорости для левой и правой сторон робота:
    // При дифференциальном приводе скорость левой стороны уменьшается, а правой – увеличивается
    double v_left  = linear_vel - angular_vel * wheel_separation / 2.0;
    double v_right = linear_vel + angular_vel * wheel_separation / 2.0;

    // Отправляем вычисленные команды на Arduino
    sendCommandsToArduino(v_left, v_right, f_angle, r_angle);

    return return_type::OK;
}

// ----------------------------------------------------------------------------------------
// Метод sendCommandsToArduino
// ----------------------------------------------------------------------------------------
// Метод отправляет команды управления на Arduino: скорости для двигателей и углы поворотных осей.
// Здесь происходит проверка соединения, преобразование значений с учётом параметров колеса и
// частоты цикла, после чего вызываются соответствующие методы объекта ArduinoComms.
return_type DiffDriveArduino::sendCommandsToArduino(double v_left, double v_right, double f_angle, double r_angle)
{
    // Настраиваем подключение к Arduino, используя параметры host и port из конфигурации
    arduino_.setup(cfg_.host, cfg_.port);
    // Проверяем, установлено ли соединение с Arduino
    if (!arduino_.connected())
    {
        return return_type::ERROR;
    }

    // Выводим информационное сообщение с текущими значениями команд
    RCLCPP_INFO(this->get_logger(),
                "Sending commands: left=%f, right=%f, front_angle=%f, rear_angle=%f",
                v_left, v_right, f_angle, r_angle);

    // Отправляем команды для управления двигателями и углами поворота
    arduino_.setMotorValues(f_angle, r_angle, v_left, v_right);

    return return_type::OK;
}

// ----------------------------------------------------------------------------------------
// Метод updateOdometry
// ----------------------------------------------------------------------------------------
// Метод обновляет одометрию робота. Сначала вызывается метод read() для получения актуальных
// данных от энкодеров, затем вычисляются линейная и угловая скорости, проводится интегрирование
// (метод Эйлера) для определения нового положения, и формируется сообщение одометрии, которое публикуется.
return_type DiffDriveArduino::updateOdometry()
{
    // 1. Сначала обновляем данные с оборудования (считываем значения энкодеров)
    if (read() != return_type::OK) {
        return return_type::ERROR;
    }
    
    // 2. Получаем текущее время (ROS2 Time) и вычисляем прошедшее время dt с момента последнего обновления
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_odom_time_).seconds();
    last_odom_time_ = current_time;
    
    // 3. Рассчитываем линейные скорости для каждого колеса, умножая угловую скорость (полученную из энкодера)
    //    на радиус колеса.
    double V_FL = wheel_radius_ * fl_wheel_.vel;  // Скорость переднего левого колеса
    double V_FR = wheel_radius_ * fr_wheel_.vel;  // Скорость переднего правого колеса
    double V_RL = wheel_radius_ * rl_wheel_.vel;  // Скорость заднего левого колеса
    double V_RR = wheel_radius_ * rr_wheel_.vel;  // Скорость заднего правого колеса

    // 4. Получаем углы поворотных осей (стиринга) для передней и задней осей.
    //    Предполагается, что метод gegetSteeringAngle() возвращает угол в градусах.
    double theta_f = f_steering_.getSteeringAngle();  // Угол переднего стиринга (градусы)
    double theta_r = r_steering_.getSteeringAngle();   // Угол заднего стиринга (градусы)
    
    // 5. Задаём параметры, необходимые для расчёта:
    //    L_f и L_r — расстояния, используемые в расчётах (например, расстояния от осей до центра поворота);
    //    W — расстояние между левыми и правыми колесами (ширина колеи).
    //    Здесь в качестве примера используются фиксированные значения, которые можно заменить
    //    на параметры из конфигурации, если они у вас заданы.
    double L_f = 0.262;   // Примерное значение для передней оси (м)
    double L_r = 0.262;   // Примерное значение для задней оси (м)
    double W   = 0.234;   // Расстояние между левыми и правыми колесами (м)
    
    // 6. Вызываем ваш метод расчёта, который принимает скорости всех четырёх колес, углы стиринга,
    //    а также параметры L_f, L_r и W. Метод должен вернуть пару значений:
    //       - linear_velocity: линейная скорость транспортного средства (V)
    //       - angular_velocity: угловая скорость транспортного средства (ω)
    std::pair<double, double> speed_andomega = f_steering_.computeVehicleSpeedAndOmega(
          V_FL, V_FR, V_RL, V_RR,  // Скорости для всех четырёх колес
          theta_f, theta_r,        // Углы поворотных осей (градусы)
          L_f, L_r,                // Параметры для расчёта (расстояния)
          W);                      // Расстояние между колесами

    double linear_velocity  = speed_and_omega.first;   // Линейная скорость (м/с)
    double angular_velocity = speed_and_omega.second;  // Угловая скорость (рад/с)
    
    // 7. Интегрируем полученные скорости методом Эйлера для обновления положения робота:
    //    Вычисляем приращения по оси X, Y и изменение угла ориентации.
    double delta_x     = linear_velocity * std::cos(theta_) * dt;
    double delta_y     = linear_velocity * std::sin(theta_) * dt;
    double delta_theta = angular_velocity * dt;
    
    // 8. Обновляем одометрические координаты робота.
    x_     += delta_x;
    y_     += delta_y;
    theta_ += delta_theta;
    
    // 9. Формируем сообщение одометрии для публикации.
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp    = current_time;    // Устанавливаем временную метку
    odom_msg.header.frame_id = "odom";          // Фиксированная система координат одометрии
    odom_msg.child_frame_id  = "base_link";       // Система координат, привязанная к роботу
    
    // Заполняем позиционные данные
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    
    // Преобразуем угол поворота (yaw) в кватернион для корректного представления ориентации
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    
    // Заполняем данные о скоростях:
    // Линейная скорость передаётся в twist.linear.x, угловая — в twist.angular.z.
    odom_msg.twist.twist.linear.x  = linear_velocity;
    odom_msg.twist.twist.linear.y  = 0.0;
    odom_msg.twist.twist.linear.z  = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_velocity;
    
    // 10. Публикуем сообщение одометрии на соответствующем топике.
    odom_publisher_->publish(odom_msg);
    
    // 11. Выводим отладочное сообщение с текущими значениями одометрии и скоростей.
    RCLCPP_DEBUG(this->get_logger(),
                 "Odometry: x=%.2f, y=%.2f, theta=%.2f, V=%.2f, omega=%.2f",
                 x_, y_, theta_, linear_velocity, angular_velocity);
    
    return return_type::OK;
}

std::vector<hardware_interface::StateInterface> export_state_interfaces() override { return {}; }
std::vector<hardware_interface::CommandInterface> export_command_interfaces() override { return {}; }
hardware_interface::return_type write() override { return hardware_interface::return_type::OK; }


// ----------------------------------------------------------------------------------------
// Подключение класса в систему плагинов ROS2
// ----------------------------------------------------------------------------------------
// Этот макрос позволяет системе плагинов ROS2 обнаруживать и загружать данный класс как аппаратный интерфейс.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  DiffDriveArduino,
  hardware_interface::SystemInterface
)
