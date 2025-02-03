#ifndef DIFFDRIVE_ARDUINO_STEERING_H
#define DIFFDRIVE_ARDUINO_STEERING_H

#include <cmath>
#include <string>

/**
 * @brief Класс Steering реализует логику расчёта угла поворота оси,
 *        на которой закреплены колёса, поворачивающиеся вместе.
 *
 * Параметры:
 *  - axle_offset: продольное смещение оси относительно центра робота
 *                 (для передней оси положительное, для задней – отрицательное).
 *  - track_width: расстояние между левым и правым колесом на оси.
 *  - steering_sign: коэффициент направления управления осью:
 *       +1 для осей, поворачивающихся в ту же сторону, что и направление поворота (например, передняя ось),
 *       -1 для осей с противофазным поворотом (например, задняя ось в 4-колёсном управлении).
 */
class Steering {
public:
    std::string name = "";

    void setup(const std::string &steering_name);

    /**
     * @brief Конструктор.
     * @param axle_offset Продольное смещение оси от центра робота.
     * @param track_width Расстояние между колёсами оси.
     * @param steering_sign Направление поворота оси (+1 или -1).
     */
    Steering(float axle_offset, float track_width, float steering_sign = 1.0f);

    /**
     * @brief Вычисляет угол поворота оси на основании заданных скоростей.
     * @param linear_velocity Линейная скорость робота (v).
     * @param angular_velocity Угловая скорость робота (ω).
     *
     * Если ω близко к нулю, ось выпрямляется (угол = 0).
     */
    void update(float linear_velocity, float angular_velocity);

    /**
     * @brief Возвращает вычисленный угол поворота оси (в радианах).
     */
    float getSteeringAngle() const;

private:
    float axle_offset_;    // Продольное смещение оси относительно центра робота.
    float track_width_;    // Расстояние между левым и правым колесом на оси.
    float steering_sign_;  // Направление поворота оси: +1 (прямая фаза) или -1 (противофазная).
    float steering_angle_; // Вычисленный угол поворота (в радианах).

    // Допустимая малая величина для проверки, равна ли угловая скорость нулю.
    static constexpr float EPSILON = 1e-6f;
};

#endif // DIFFDRIVE_ARDUINO_STEERING_H
