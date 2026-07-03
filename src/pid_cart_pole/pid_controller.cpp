#include "pid_controller.hpp"
#include <algorithm>

namespace cart_pole_controller
{

PidController::PidController(double Kp, double Ki, double Kd, double dt,
                             double out_min, double out_max, double int_max)
  : Kp_(Kp), Ki_(Ki), Kd_(Kd), dt_(dt),
    out_min_(out_min), out_max_(out_max), int_max_(int_max)
{
}

double PidController::step(double reference, double measurement)
{
  const double error = reference - measurement;

  // 比例
  const double P = Kp_ * error;

  // 积分（带限幅抗饱和）
  if (!first_)
    integral_ += error * dt_;
  integral_ = std::clamp(integral_, -int_max_, int_max_);
  const double I = Ki_ * integral_;

  // 微分
  double D = 0.0;
  if (!first_)
    D = Kd_ * (error - prev_error_) / dt_;

  prev_error_ = error;
  first_ = false;

  double out = P + I + D;
  out = std::clamp(out, out_min_, out_max_);
  return out;
}

void PidController::reset()
{
  integral_  = 0.0;
  prev_error_ = 0.0;
  first_ = true;
}

}  // namespace cart_pole_controller
