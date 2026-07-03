#pragma once

#include <string>

namespace cart_pole_controller
{

// ─────────────────────────────────────────────────
//  模型参数
// ─────────────────────────────────────────────────
struct ModelConfig
{
  double M_c = 1.0;    // 小车质量 [kg]
  double m_p = 0.1;    // 摆杆质量 [kg]
  double L   = 0.5;    // 摆杆质心到支点距离 [m]
  double I   = -1.0;   // 转动惯量; 负值→自动计算 (1/3)*m_p*L*L
  double k   = 0.1;    // 阻尼系数 [N·s/m]
  double g   = 9.81;   // 重力加速度 [m/s²]
};

// ─────────────────────────────────────────────────
//  初始状态
// ─────────────────────────────────────────────────
struct InitialStateConfig
{
  double y         = 0.0;   // 小车位置 [m]
  double theta_deg = 5.0;   // 摆杆角度 [deg]
  double dy        = 0.0;   // 小车速度 [m/s]
  double dtheta    = 0.0;   // 摆杆角速度 [rad/s]
};

// ─────────────────────────────────────────────────
//  PID 参数
// ─────────────────────────────────────────────────
struct PidParams
{
  double Kp      = 1.0;
  double Ki      = 0.0;
  double Kd      = 0.0;
  double out_min = -100.0;
  double out_max =  100.0;
  double int_max =  50.0;
};

// ─────────────────────────────────────────────────
//  控制器配置
// ─────────────────────────────────────────────────
struct ControllerConfig
{
  double dt       = 0.005;   // 控制步长 [s]
  double sim_time = 10.0;    // 仿真时长 [s]

  PidParams outer_pid;       // 外环: y → theta_ref
  PidParams inner_pid;       // 内环: theta → F
};

// ─────────────────────────────────────────────────
//  整体仿真配置 (聚合)
// ─────────────────────────────────────────────────
struct SimulationConfig
{
  ModelConfig        model;
  InitialStateConfig initial_state;
  ControllerConfig   controller;
};

/// 从 YAML 文件加载仿真配置。
/// 文件中没有显式写出的键使用上述默认值。
/// 解析失败或参数非法时抛出 std::runtime_error。
SimulationConfig loadConfig(const std::string & yaml_path);

/// 如果 ModelConfig::I < 0, 自动计算为 (1/3)*m_p*L*L
void resolveInertia(ModelConfig & cfg);

}  // namespace cart_pole_controller
