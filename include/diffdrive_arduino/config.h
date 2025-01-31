#ifndef DIFFDRIVE_ARDUINO_CONFIG_H
#define DIFFDRIVE_ARDUINO_CONFIG_H

#include <string>

struct Config
{
    std::string left_wheel_name = "left_wheel";
    std::string right_wheel_name = "right_wheel";
    float loop_rate = 60;
    std::string host = "172.20.76.101"; // IP-адрес вашего UDP-устройства
    int port = 5000; // Порт вашего UDP-устройства
    int enc_counts_per_rev = 150;
};

#endif // DIFFDRIVE_ARDUINO_CONFIG_H
