#include "diffdrive_arduino/steering.h"
#include <cmath>

/**
 * Конструктор: сохраняет параметры оси.
 */
Steering::Steering(double axle_offset, double track_width, double steering_sign)
    : axle_offset_(axle_offset),
      track_width_(track_width),
      steering_sign_(steering_sign),
      steering_angle_(0.0f)
{
}

// Метод настройки оси
void Steering::setup(const std::string &steering_name, double axle_offset, double track_width, double steering_sign)
{
  Steering(axle_offset, track_width, steering_sign);
  name = steering_name;  // Присваиваем оси имя, переданное в аргументах
}

/**
 * Метод update вычисляет угол поворота оси по следующей логике:
 *
 * 1. Если угловая скорость (ω) близка к нулю, считаем, что робот движется по прямой:
 *    → угол поворота оси равен 0.
 *
 * 2. Иначе вычисляем модуль радиуса поворота:
 *      R = v / ω   →   R_abs = |v/ω|
 *
 * 3. Для заданной оси с продольным смещением l (axle_offset_) и расстоянием между колёсами (track_width_)
 *    вычисляем «идеальные» углы для двух колёс, которые должны следовать тангенциально к круговой траектории:
 *
 *      - Для внутреннего колеса (ближе к ИМЦ): R_in = R_abs - (track_width_/2)
 *        угол α_in = arctan(l / R_in)
 *
 *      - Для внешнего колеса: R_out = R_abs + (track_width_/2)
 *        угол α_out = arctan(l / R_out)
 *
 * 4. В качестве базового угла оси берём среднее:
 *      α_base = (α_in + α_out) / 2
 *
 * 5. Применяем поправку по знаку:
 *      - Для передней оси (steering_sign_ > 0):
 *          при повороте налево (ω > 0) угол должен быть положительным, при повороте направо – отрицательным.
 *      - Для задней оси (steering_sign_ < 0) (противофазное управление):
 *          при повороте налево угол – отрицательный, при повороте направо – положительный.
 *
 *    То есть итоговый угол:
 *      steering_angle_ = (ω > 0 ? 1 : -1) * α_base для передней оси,
 *      steering_angle_ = (ω > 0 ? -1 : 1) * α_base для задней оси.
 *
 * @param linear_velocity Линейная скорость робота (v).
 * @param angular_velocity Угловая скорость робота (ω).
 */
void Steering::update(double linear_velocity, double angular_velocity)
{
    // Если угловая скорость очень мала – считаем, что поворота нет.
    if (std::fabs(angular_velocity) < EPSILON) {
        steering_angle_ = 0.0f;
        return;
    }

    // Вычисляем модуль радиуса поворота.
    double R_abs = std::fabs(linear_velocity / angular_velocity);

    // Расчёт идеальных углов для левого и правого колеса оси.
    // Здесь используется геометрическая зависимость: при движении по дуге
    // продольное смещение оси (axle_offset_) делится на расстояние до ИМЦ.
    double R_in = R_abs - (track_width_ / 2.0f);   // расстояние до ИМЦ для внутреннего колеса
    double R_out = R_abs + (track_width_ / 2.0f);  // для внешнего колеса

    // Чтобы избежать деления на ноль (если R_in очень мало), можно ограничить R_in снизу.
    if (R_in < EPSILON) {
        R_in = EPSILON;
    }

    double angle_inner = std::atan2(axle_offset_, R_in);
    double angle_outer = std::atan2(axle_offset_, R_out);

    // Средний (базовый) угол поворота оси.
    double base_angle = (angle_inner + angle_outer) / 2.0f;

    // Определяем знак, зависящий от направления поворота (ω) и типа оси (steering_sign_):
    // Для передней оси (steering_sign_ > 0):
    //    ω > 0 (поворот налево) → угол положительный, ω < 0 → угол отрицательный.
    // Для задней оси (steering_sign_ < 0):
    //    ω > 0 → угол должен быть противоположным (отрицательный), ω < 0 → положительным.
    if (steering_sign_ > 0) {
        steering_angle_ = (angular_velocity > 0 ? base_angle : -base_angle);
    } else {
        steering_angle_ = (angular_velocity > 0 ? -base_angle : base_angle);
    }
}

/**
 * Возвращает вычисленный угол поворота оси (в радианах).
 */
double Steering::getSteeringAngle() const
{
    return steering_angle_;
}

/**
 * Выполняет обратное преобразование: вычисляет линейную и угловую скорости
 * на основе скоростей колес и угла поворота оси.
 */
double Steering::computeVehicleSpeedAndOmega(double V_FL, double V_FR, double V_RL, double V_RR, double theta_f, double theta_r, double L_f, double L_r, double W) 
{
    
    double theta_f_rad = theta_f * M_PI / 180.0;
    double theta_r_rad = theta_r * M_PI / 180.0;

    double R_FL = L_f / std::tan(theta_f_rad);
    double R_FR = L_f / std::tan(theta_f_rad);
    double R_RL = L_r / std::tan(theta_r_rad);
    double R_RR = L_r / std::tan(theta_r_rad);
    
    double R = (R_FL + R_FR + R_RL + R_RR) / 4.0;
    double omega = (V_FR - V_FL) / W;
    double V = omega * R;
    
    return {V, omega};
}
