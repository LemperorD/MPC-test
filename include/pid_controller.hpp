#pragma once

namespace cart_pole_controller
{

/**
 * @brief 离散 PID 控制器（位置式）。
 *
 * u_k = Kp·e_k + Ki·∑e_i·dt + Kd·(e_k - e_{k-1})/dt
 *
 * 内置积分限幅和输出限幅，防止 windup 和过大出力。
 */
class PidController
{
public:
  /**
   * @param Kp  比例增益
   * @param Ki  积分增益
   * @param Kd  微分增益
   * @param dt  控制周期 [s]
   * @param out_min, out_max 输出限幅
   * @param int_max          积分项绝对值上限
   */
  PidController(double Kp, double Ki, double Kd, double dt,
                double out_min = -100.0, double out_max = 100.0,
                double int_max = 50.0);

  /** @brief 单步控制量计算。 */
  double step(double reference, double measurement);

  /** @brief 重置积分器和微分历史。 */
  void reset();

  // 访问器
  double Kp() const { return Kp_; }
  double Ki() const { return Ki_; }
  double Kd() const { return Kd_; }

private:
  double Kp_, Ki_, Kd_, dt_;
  double out_min_, out_max_, int_max_;

  double integral_  = 0.0;
  double prev_error_ = 0.0;
  bool   first_      = true;
};

}  // namespace cart_pole_controller
