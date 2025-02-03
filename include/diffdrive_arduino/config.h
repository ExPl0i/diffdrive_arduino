#ifndef DIFFDRIVE_ARDUINO_CONFIG_H
#define DIFFDRIVE_ARDUINO_CONFIG_H

#include <string>

struct Config
{
    std::string front_steering_name = "front_steering_wheel";
    std::string rear_steering_name = "rear_steering_wheel";
    std::string front_left_wheel_name = "front_left_wheel";
    std::string front_right_wheel_name = "front_right_wheel";
    std::string rear_left_wheel_name = "rear_left_wheel";
    std::string rear_right_wheel_name = "rear_right_wheel";
    float loop_rate = 60;
    std::string host = "172.20.76.101"; // IP-адрес вашего UDP-устройства
    int port = 5000; // Порт вашего UDP-устройства
    int enc_counts_per_rev = 150;
};

#endif // DIFFDRIVE_ARDUINO_CONFIG_H
