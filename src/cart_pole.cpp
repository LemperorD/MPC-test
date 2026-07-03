#include "cart_pole.hpp"
#include <cmath>
#include <stdexcept>

namespace cart_pole_controller
{

// ──────────────────────────────────────────────
// 构造
// ──────────────────────────────────────────────
CartPoleModel::CartPoleModel(double M_c, double m_p, double L, double I,
                             double k, double g)
  : M_c_(M_c), m_p_(m_p), L_(L), I_(I), k_(k), g_(g)
{
  if (M_c <= 0 || m_p <= 0 || L <= 0 || I <= 0)
    throw std::invalid_argument("CartPoleModel: 参数必须为正。");
}

// ──────────────────────────────────────────────
// 连续动力学: ẋ = f(x, u)
// ──────────────────────────────────────────────
Eigen::Vector4d CartPoleModel::fCont(const Eigen::Vector4d & x,
                                     double u) const
{
  const double y   = x(0);
  const double th  = x(1);
  const double dy  = x(2);
  const double dth = x(3);

  const double S = std::sin(th);
  const double C = std::cos(th);

  // 中间常数
  const double A = I_ + m_p_ * L_ * L_;   // I + mL²
  const double B = m_p_ * L_;             // mL

  // 解耦方程 (联立求解 ÿ, θ̈)：
  // (1)  A·θ̈ = m_p·g·L·S - B·C·ÿ
  // (2)  (M+m)·ÿ + B·C·θ̈ - B·S·dth² + k·dy = u
  //
  // 将 (1) 解为 θ̈ = (m_p·g·L·S - B·C·ÿ) / A
  // 代入 (2) 得：
  //   (M+m)·ÿ + B·C·(m_p·g·L·S - B·C·ÿ)/A - B·S·dth² + k·dy = u
  // → (M+m - B²·C²/A)·ÿ = u + B·S·dth² - k·dy - B·C·m_p·g·L·S/A
  //
  // 令 den = M + m - B²·C²/A

  const double den = (M_c_ + m_p_) - (B * B * C * C) / A;

  const double rhs_ydd = u
                       + B * S * dth * dth
                       - k_ * dy
                       - (B * C * m_p_ * g_ * L_ * S) / A;

  const double ydd = rhs_ydd / den;

  // 回代得 θ̈
  const double thdd = (m_p_ * g_ * L_ * S - B * C * ydd) / A;

  Eigen::Vector4d xdot;
  xdot << dy, dth, ydd, thdd;
  return xdot;
}

// ──────────────────────────────────────────────
// 前向欧拉
// ──────────────────────────────────────────────
Eigen::Vector4d CartPoleModel::forwardEuler(const Eigen::Vector4d & x,
                                            double u, double dt) const
{
  return x + dt * fCont(x, u);
}

// ──────────────────────────────────────────────
// RK4
// ──────────────────────────────────────────────
Eigen::Vector4d CartPoleModel::rk4Step(const Eigen::Vector4d & x,
                                       double u, double dt) const
{
  const auto k1 = fCont(x, u);
  const auto k2 = fCont(x + 0.5 * dt * k1, u);
  const auto k3 = fCont(x + 0.5 * dt * k2, u);
  const auto k4 = fCont(x + dt * k3, u);

  return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

// ──────────────────────────────────────────────
// 线性化（直立平衡点）
// ──────────────────────────────────────────────
std::pair<Eigen::Matrix4d, Eigen::Vector4d>
CartPoleModel::linearizeUpright() const
{
  // 平衡点附近 sinθ≈θ, cosθ≈1, 舍去二阶项（θ̇² 等）：
  //
  // (I+mL²)·θ̈ = mgLθ - mL·ÿ
  // (M+m)·ÿ + mL·θ̈ + kẏ = F
  //
  // 联立消元得：
  //   θ̈ = [mgL·(M+m)·θ + mL·k·ẏ - mL·F] / Δ
  //   ÿ  = [-(I+mL²)·k·ẏ + (I+mL²)·F + mL·mgL·θ] / Δ    ← 注意符号
  //
  // 其中 Δ = (I+mL²)(M+m) - (mL)²

  const double A  = I_ + m_p_ * L_ * L_;   // I + mL²
  const double B  = m_p_ * L_;             // mL
  const double Mt = M_c_ + m_p_;           // M + m
  const double Delta = A * Mt - B * B;     // 正定

  // 状态顺序: [y, θ, ẏ, θ̇]
  Eigen::Matrix4d A_mat = Eigen::Matrix4d::Zero();
  Eigen::Vector4d B_vec = Eigen::Vector4d::Zero();

  // ẏ = ẏ (trivial)
  A_mat(0, 2) = 1.0;

  // θ̇ = θ̇ (trivial)
  A_mat(1, 3) = 1.0;

  // ÿ  = (-A·k·ẏ + mL·mgL·θ) / Δ   +   (A/Δ)·F
  A_mat(2, 1) =  B * m_p_ * g_ * L_ / Delta;
  A_mat(2, 2) = -A * k_            / Delta;
  B_vec(2)    =  A                 / Delta;

  // θ̈  = (Mt·mgL·θ + B·k·ẏ) / Δ   +   (-B/Δ)·F
  A_mat(3, 1) =  Mt * m_p_ * g_ * L_ / Delta;
  A_mat(3, 2) =  B  * k_            / Delta;
  B_vec(3)    = -B                   / Delta;

  return {A_mat, B_vec};
}

}  // namespace cart_pole_controller
