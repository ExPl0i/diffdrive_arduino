#include "diffdrive_arduino/arduino_comms.h"
#include <asio.hpp>      // Библиотека для асинхронного сетевого взаимодействия
#include <iostream>      // Для вывода ошибок в консоль
#include <sstream>       // Для работы со строковыми потоками
#include <string>        // Для работы со строками
#include <regex>         // Для работы с регулярными выражениями

// Метод установки соединения с Arduino по UDP
void ArduinoComms::setup(const std::string &host, int port)
{
    // Создаем объект для разрешения DNS-имен
    asio::ip::udp::resolver resolver(io_context_);

    // Формируем запрос на разрешение IP-адреса по хосту и порту
    asio::ip::udp::resolver::query query(asio::ip::udp::v4(), host, std::to_string(port));

    // Получаем итератор с информацией о целевом узле
    asio::ip::udp::resolver::iterator iterator = resolver.resolve(query);

    // Сохраняем конечную точку соединения (IP-адрес и порт Arduino)
    endpoint_ = *iterator;

    // Открываем UDP-сокет для отправки и приема данных
    socket_.open(asio::ip::udp::v4());
}

// Отправляет пустое сообщение на Arduino для инициализации связи
void ArduinoComms::sendEmptyMsg()
{
    sendMsg("[0.0,0.0,0.0]\r"); // Отправляем сообщение с тремя нулевыми значениями
}

// Читает значения энкодеров с Arduino
void ArduinoComms::readEncoderValues(int &val_1, int &val_2, int &val_3, int &val_4)
{
    // Отправка запроса на получение данных с энкодеров
    // sendMsg("e\r"); // Закомментировано, но можно раскомментировать для явного запроса

    // Получаем ответ от Arduino
    std::string response = receiveMsg();

    // Регулярное выражение для поиска двух целых чисел в формате: [число, число]
    std::regex regex("\\[(-?\\d+),(-?\\d+)\\]");
    std::smatch matches;
    
    // Проверяем, соответствует ли ответ ожидаемому формату
    if (std::regex_search(response, matches, regex) && matches.size() == 5)
    {
        val_1 = std::stoi(matches[1].str()); // Преобразуем первую строку в число (левый энкодер)
        val_2 = std::stoi(matches[2].str()); // Преобразуем вторую строку в число (правый энкодер)
        val_3 = std::stoi(matches[3].str());
        val_4 = std::stoi(matches[4].str());
    }
    else
    {
        // Если формат данных неверный, выводим ошибку в консоль и присваиваем значения по умолчанию
        std::cerr << "Received message with incorrect format: " << response << std::endl;
        val_1 = 0;
        val_2 = 0;
        val_3 = 0;
        val_4 = 0;
    }
}

// Устанавливает значения скорости моторов (управляющие команды)
void ArduinoComms::setMotorValues(float val_1)
{
    std::stringstream ss;
    ss << "[" << val_1 << "," << (val_1) << "]";

    sendMsg(ss.str()); // Отправляем команду на Arduino
}

// Устанавливает PID-параметры моторов (пропорциональный, дифференциальный, интегральный коэффициенты)
void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";

    // Закомментирован вызов отправки, можно раскомментировать для реальной работы
    // sendMsg(ss.str());
}

// Отправляет сообщение через UDP-сокет
void ArduinoComms::sendMsg(const std::string &msg_to_send)
{
    socket_.send_to(asio::buffer(msg_to_send), endpoint_);
}

// Получает сообщение через UDP-сокет
std::string ArduinoComms::receiveMsg()
{
    // Объявляем объект для хранения информации об отправителе
    asio::ip::udp::endpoint sender_endpoint;

    // Получаем данные в буфер и определяем их длину
    size_t len = socket_.receive_from(asio::buffer(recv_buffer_), sender_endpoint);

    // Возвращаем строку с полученными данными
    return std::string(recv_buffer_, len);
}

