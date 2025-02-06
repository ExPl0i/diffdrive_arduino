#include "diffdrive_arduino/diffdrive_arduino.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "steering.cpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

// Конструктор класса DiffDriveArduino
DiffDriveArduino::DiffDriveArduino()
    : logger_(rclcpp::get_logger("DiffDriveArduino")) // Инициализация логгера
{}

return_type DiffDriveArduino:: Node(diffDriveArduino) 
{
    // Подписка на cmd_vel
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10,
    std::bind(&DiffDriveNode::cmdVelCallback, this, std::placeholders::_1));

    // Публикация одометрии
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    // Таймер для периодического обновления одометрии (например, 10 Гц)
    timer_ = this->create_wall_timer(100ms, std::bind(&DiffDriveNode::updateOdometry, this));
}

// Метод конфигурации оборудования
return_type DiffDriveArduino::configure(const hardware_interface::HardwareInfo &info)
{
    // Вызов стандартного метода конфигурации и проверка успешности
    if (configure_default(info) != return_type::OK)
    {
        return return_type::ERROR;
    }

    RCLCPP_INFO(logger_, "Configuring..."); // Логируем начало конфигурации

    // Задаём параметры передней поворотной оси:
    float f_axle_offset = 0.262f;    // Примерное смещение оси (в метрах)
    float f_track_width = 0.234f;    // Примерное расстояние между колёсами (в метрах)
    float f_steering_sign = 1.0f;  // +1 для передней оси (или -1 для задней оси)

    // Задаём параметры задней поворотной оси:
    float r_axle_offset = 0.262f;    // Примерное смещение оси (в метрах)
    float r_track_width = 0.234f;    // Примерное расстояние между колёсами (в метрах)
    float r_steering_sign = -1.0f;  // +1 для передней оси (или -1 для задней оси) 

    // Запоминаем текущее время
    time_ = std::chrono::system_clock::now();

    try
    {
        // Получаем параметры оборудования из конфигурации
        cfg_.front_left_wheel_name = info.hardware_parameters.at("front_left_wheel_name");
        cfg_.front_right_wheel_name = info.hardware_parameters.at("front_right_wheel_name");
        cfg_.rear_left_wheel_name = info.hardware_parameters.at("rear_left_wheel_name");
        cfg_.rear_right_wheel_name = info.hardware_parameters.at("rear_right_wheel_name");
        cfg_.front_steering_name = info.hardware_parameters.at("front_steering_name");
        cfg_.rear_steering_name = info.hardware_parameters.at("rear_steering_name");
        cfg_.loop_rate = std::stof(info.hardware_parameters.at("loop_rate"));
        cfg_.host = info.hardware_parameters.at("host");
        cfg_.port = std::stoi(info.hardware_parameters.at("port"));
        cfg_.enc_counts_per_rev = std::stoi(info.hardware_parameters.at("enc_counts_per_rev")); //это их файла колесо
    }
    catch (const std::invalid_argument& e) // Обрабатываем ошибку неверного формата параметров
    {
        // Проверяем, какого параметра не хватает, и логируем ошибку
        if (info.hardware_parameters.find("front_left_wheel_name") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(logger_, "Missing parameter: front_left_wheel_name");
        }
        else if (info.hardware_parameters.find("front_right_wheel_name") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(logger_, "Missing parameter: front_right_wheel_name");
        }
        else if (info.hardware_parameters.find("rear_left_wheel_name") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(logger_, "Missing parameter: rear_left_wheel_name");
        }
        else if (info.hardware_parameters.find("rear_right_wheel_name") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(logger_, "Missing parameter: rear_right_wheel_name");
        }
        else if (info.hardware_parameters.find("front_steering_name") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(logger_, "Missing parameter: front_steering_name");
        }
        else if (info.hardware_parameters.find("rear_steering_name") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(logger_, "Missing parameter: rear_steering_name");
        }
        else if (info.hardware_parameters.find("loop_rate") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(logger_, "Missing parameter: loop_rate");
        }
        else if (info.hardware_parameters.find("host") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(logger_, "Missing parameter: host");
        }
        else if (info.hardware_parameters.find("port") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(logger_, "Missing parameter: port");
        }
        else if (info.hardware_parameters.find("enc_counts_per_rev") == info.hardware_parameters.end())
        {
            RCLCPP_ERROR(logger_, "Missing parameter: enc_counts_per_rev");
        }
        else
        {
            RCLCPP_ERROR(logger_, "Invalid argument for parameter: %s", e.what());
        }
        return return_type::ERROR;
    }
    catch (const std::out_of_range& e) // Обрабатываем ошибку выхода за границы значений
    {
        RCLCPP_ERROR(logger_, "Out of range for parameter: %s", e.what());
        return return_type::ERROR;
    }

    // Настраиваем колеса и оси с полученными параметрами
    fl_wheel_.setup(cfg_.front_left_wheel_name, cfg_.enc_counts_per_rev);
    fr_wheel_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);
    rl_wheel_.setup(cfg_.rear_left_wheel_name, cfg_.enc_counts_per_rev);
    rr_wheel_.setup(cfg_.rear_right_wheel_name, cfg_.enc_counts_per_rev);
    f_steering_.setup(cfg_.front_steering_name, f_axle_offset, f_track_width, f_steering_sign);
    r_steering_.setup(cfg_.rear_steering_name, r_axle_offset, r_track_width, r_steering_sign);

    // Настраиваем подключение к Arduino
    arduino_.setup(cfg_.host, cfg_.port);

    RCLCPP_INFO(logger_, "Finished Configuration"); // Логируем завершение конфигурации

    status_ = hardware_interface::status::CONFIGURED; // Устанавливаем статус "СКОНФИГУРИРОВАН"
    return return_type::OK;
}

// Метод запуска контроллера
return_type DiffDriveArduino::start()
{
    RCLCPP_INFO(logger_, "Starting Controller..."); // Логируем запуск контроллера

    arduino_.sendEmptyMsg(); // Отправляем пустое сообщение на Arduino
    arduino_.setPidValues(30, 30, 0, 100); // Устанавливаем PID-параметры

    status_ = hardware_interface::status::STARTED; // Устанавливаем статус "ЗАПУЩЕН"

    return return_type::OK;
}

// Метод остановки контроллера
return_type DiffDriveArduino::stop()
{
    RCLCPP_INFO(logger_, "Stopping Controller..."); // Логируем остановку контроллера
    status_ = hardware_interface::status::STOPPED; // Устанавливаем статус "ОСТАНОВЛЕН"

    return return_type::OK;
}

// Метод чтения данных с оборудования (положение и скорость колес)
hardware_interface::return_type DiffDriveArduino::read()
{
    // Вычисляем разницу времени с последнего вызова
    auto new_time = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = new_time - time_;
    double deltaSeconds = diff.count();
    time_ = new_time;

    // Проверяем соединение с Arduino
    if (!arduino_.connected())
    {
        return return_type::ERROR;
    }

    // Читаем значения энкодеров с Arduino
    arduino_.readEncoderValues(fl_wheel_.enc, fr_wheel_.enc, rl_wheel_.enc, rr_wheel_.enc);

    // Вычисляем новое положение и скорость левого переднего колеса
    double pos_prev = fl_wheel_.pos;
    fl_wheel_.pos = fl_wheel_.calcEncAngle();
    fl_wheel_.vel = (fl_wheel_.pos - pos_prev) / deltaSeconds;// Вычисляем скорость вращения левого колеса (угловую скорость) - Разница между текущим и предыдущим положением делится на прошедшее время

    // Вычисляем новое положение и скорость правого переднего колеса
    pos_prev = fr_wheel_.pos;
    fr_wheel_.pos = fr_wheel_.calcEncAngle();// Обновляем текущее положение правого колеса, 
    fr_wheel_.vel = (fr_wheel_.pos - pos_prev) / deltaSeconds;

    // Вычисляем новое положение и скорость левого заднего колеса
    pos_prev = rl_wheel_.pos;
    rl_wheel_.pos = rl_wheel_.calcEncAngle();
    rl_wheel_.vel = (rl_wheel_.pos - pos_prev) / deltaSeconds;// Вычисляем скорость вращения левого колеса (угловую скорость) - Разница между текущим и предыдущим положением делится на прошедшее время

    // Вычисляем новое положение и скорость правого заднего колеса
    pos_prev = rr_wheel_.pos;
    rr_wheel_.pos = rr_wheel_.calcEncAngle();// Обновляем текущее положение правого колеса, 
    rr_wheel_.vel = (rr_wheel_.pos - pos_prev) / deltaSeconds;

    return return_type::OK;
}


hardware_interface::return_type DiffDriveArduino::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // Расстояние между колесами
    double wheel_separation = 0.234f;

    // Получение линейной и угловой скоростей из /cmd_vel
    double linear_vel = msg->linear.x;
    double angular_vel = msg->angular.z;

    // Расчёт углов поворота повротных осей:
    f_steering_.update(linear_vel, angular_vel);
    r_steering_.update(linear_vel, angular_vel);
    double f_angle = f_steering_.gegetSteeringAngle();
    double r_angle = r_steering_.gegetSteeringAngle();

    // Расчёт скоростей для левых и правых колёс:
    double v_left = linear_vel - angular_vel * wheel_separation / 2.0;
    double v_right = linear_vel + angular_vel * wheel_separation / 2.0;

    // Отправка команд на Arduino
    sendCommandsToArduino(v_left, v_right, f_angle, r_angle);
}

hardware_interface::return_type DiffDriveArduino::sendCommandsToArduino(double v_left, double v_right, double f_angle, double r_angle)
{
    // Настраиваем подключение к Arduino
    arduino_.setup(cfg_.host, cfg_.port);
    // Проверяем соединение с Arduino
    if (!arduino_.connected())
    {
        return return_type::ERROR;
    }

    // Отправляем управляющие команды на Arduino
    arduino_.setMotorValues(v_left / fl_wheel_.rads_per_count / cfg_.loop_rate, 
                            v_right / fr_wheel_.rads_per_count / cfg_.loop_rate,
                            v_left / rl_wheel_.rads_per_count / cfg_.loop_rate, 
                            v_right/ rr_wheel_.rads_per_count / cfg_.loop_rate,);

    return return_type::OK;


    RCLCPP_INFO(this->get_logger(), "Sending commands: left=%f, right=%f", v_left, v_right);
}

hardware_interface::return_type DiffDriveArduino::updateOdometry()
{
    // Здесь необходимо считывать данные с Arduino (например, значения энкодеров)
    // и на их основе вычислять одометрию.
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";

    // Заполните поля сообщения odom_msg:
    // odom_msg.pose.pose.position, odom_msg.twist.twist.linear, odom_msg.twist.twist.angular и т.д.
    // Например, если вы рассчитали одометрические данные:
    // odom_msg.twist.twist.linear.x = calculated_linear_velocity;
    // odom_msg.twist.twist.angular.z = calculated_angular_velocity;

    odom_publisher_->publish(odom_msg);
}

rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
rclcpp::TimerBase::SharedPtr timer_;



// Экспорт интерфейсов состояния оборудования (состояние скорости и позиции для каждого колеса)
std::vector<hardware_interface::StateInterface> DiffDriveArduino::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Добавляем интерфейсы скорости и позиции для левого переднего колеса
    state_interfaces.emplace_back(hardware_interface::StateInterface(fl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fl_wheel_.vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(fl_wheel_.name, hardware_interface::HW_IF_POSITION, &fl_wheel_.pos));

    // Добавляем интерфейсы скорости и позиции для правого переднего колеса
    state_interfaces.emplace_back(hardware_interface::StateInterface(fr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fr_wheel_.vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(fr_wheel_.name, hardware_interface::HW_IF_POSITION, &fr_wheel_.pos));

    // Добавляем интерфейсы скорости и позиции для левого заднего колеса
    state_interfaces.emplace_back(hardware_interface::StateInterface(rl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rl_wheel_.vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(rl_wheel_.name, hardware_interface::HW_IF_POSITION, &rl_wheel_.pos));

    // Добавляем интерфейсы скорости и позиции для правого заднего колеса
    state_interfaces.emplace_back(hardware_interface::StateInterface(rr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rr_wheel_.vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(rr_wheel_.name, hardware_interface::HW_IF_POSITION, &rr_wheel_.pos));

    return state_interfaces;
}

// Экспорт интерфейсов команд (команды скорости для каждого колеса)
std::vector<hardware_interface::CommandInterface> DiffDriveArduino::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Добавляем командные интерфейсы для управления скоростью передних колес
    command_interfaces.emplace_back(hardware_interface::CommandInterface(fl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fl_wheel_.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(fr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fr_wheel_.cmd));

     // Добавляем командные интерфейсы для управления скоростью задних колес
    command_interfaces.emplace_back(hardware_interface::CommandInterface(rl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rl_wheel_.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(rr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &rr_wheel_.cmd));

    return command_interfaces;
}


// Метод записи команд в оборудование (отправка управляющих сигналов на Arduino)
hardware_interface::return_type DiffDriveArduino::write()
{
    // Проверяем соединение с Arduino
    if (!arduino_.connected())
    {
        return return_type::ERROR;
    }

    // Отправляем управляющие команды на Arduino
    arduino_.setMotorValues(fl_wheel_.cmd / fl_wheel_.rads_per_count / cfg_.loop_rate, 
                            fr_wheel_.cmd / fr_wheel_.rads_per_count / cfg_.loop_rate,
                            rl_wheel_.cmd / rl_wheel_.rads_per_count / cfg_.loop_rate, 
                            rr_wheel_.cmd / rr_wheel_.rads_per_count / cfg_.loop_rate);

    return return_type::OK;
}

// Подключение класса в систему плагинов ROS 2
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  DiffDriveArduino,
  hardware_interface::SystemInterface
)

