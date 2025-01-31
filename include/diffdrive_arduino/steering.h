#ifndef DIFFDRIVE_ARDUINO_STEERING_H
#define DIFFDRIVE_ARDUINO_STEERING_H

#include <string>

class Steering
{
    public:

    std::string name = "";
    double steering_angle = 0;

    Steering() = default;

    Steering(const std::string &steering_name, double steering_angle);
    
    void setup(const std::string &steering_name, double steering_angle);

    double calcEncAngle();
};


#endif // DIFFDRIVE_ARDUINO_STEERING_H