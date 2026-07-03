#pragma once

#include <Eigen/Dense>
#include <utility>

namespace cart_pole_controller
{

/**
 * @brief 倒立摆非线性动力学模型。
 *
 * 状态 x = [y, θ, ẏ, θ̇]^T  (世界系)
 *   y   — 小车水平位移 [m]
 *   θ   — 摆杆与竖直方向夹角 [rad]（顺时针为正）
 *   ẏ  — 小车速度 [m/s]
 *   θ̇  — 摆杆角速度 [rad/s]
 *
 * 控制 u = F — 水平推力 [N]
 *
 * 系统方程（耦合二阶ODE）：
 *   (I + mL²) θ̈ = mgL sinθ - mL cosθ·ÿ
 *   (M+m)ÿ + mL cosθ·θ̈ - mL sinθ·θ̇² + kẏ = F
 */
class CartPoleModel
{
public:
  static constexpr int NX = 4;  // 状态维度
  static constexpr int NU = 1;  // 控制维度

  /**
   * @param M_c 小车质量 [kg]
   * @param m_p 摆杆质量 [kg]
   * @param L   摆杆质心到支点距离 [m]
   * @param I   摆杆绕质心转动惯量 [kg·m²]
   * @param k   小车阻尼系数 [N·s/m]
   * @param g   重力加速度 [m/s²]
   */
  CartPoleModel(double M_c, double m_p, double L, double I,
                double k = 0.0, double g = 9.81);

  // ---------- 访问器 ----------
  double M_c() const { return M_c_; }
  double m_p() const { return m_p_; }
  double L()   const { return L_;   }
  double I()   const { return I_;   }
  double k()   const { return k_;   }
  double g()   const { return g_;   }

  // ---------- 动力学接口 ----------

  /**
   * @brief 连续动力学右端 f_cont(x, u) → 返回 ẋ = [ẏ, θ̇, ÿ, θ̈]
   *
   * 内部解耦合方程求得 ÿ 和 θ̈。
   */
  [[nodiscard]] Eigen::Vector4d fCont(const Eigen::Vector4d & x,
                                      double u) const;

  /**
   * @brief RK4 一步递推（推荐用于高精度仿真）。
   * @param x  当前状态
   * @param u  控制输入
   * @param dt 时间步长 [s]
   */
  [[nodiscard]] Eigen::Vector4d rk4Step(const Eigen::Vector4d & x,
                                        double u, double dt) const;

  /**
   * @brief 前向欧拉一步递推（仅用于简单测试）。
   */
  [[nodiscard]] Eigen::Vector4d forwardEuler(const Eigen::Vector4d & x,
                                             double u, double dt) const;

  /**
   * @brief 对直立平衡点 (θ=0, 其他=0) 做线性化，返回 A, B 矩阵。
   *
   * 平衡点处 sinθ≈θ, cosθ≈1, 舍去二阶小量后：
   *   (I+mL²)θ̈ = mgL θ - mL·ÿ
   *   (M+m)ÿ + mL·θ̈ + kẏ = F
   *
   * 线性状态空间: ẋ = A x + B u
   */
  [[nodiscard]] std::pair<Eigen::Matrix4d, Eigen::Vector4d>
  linearizeUpright() const;

private:
  double M_c_, m_p_, L_, I_, k_, g_;
};

}  // namespace cart_pole_controller
