#include "config_loader.hpp"

#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <cmath>

namespace cart_pole_controller
{

// ── 辅助: 安全读取标量, 键缺失时返回默认值 ──
static double safe_double(const YAML::Node & node,
                          const std::string & key,
                          double default_val)
{
  if (!node || !node[key] || !node[key].IsDefined())
    return default_val;
  return node[key].as<double>(default_val);
}

static PidParams loadPidParams(const YAML::Node & node, const PidParams & defaults)
{
  if (!node || !node.IsDefined())
    return defaults;

  PidParams p;
  p.Kp      = safe_double(node, "Kp",      defaults.Kp);
  p.Ki      = safe_double(node, "Ki",      defaults.Ki);
  p.Kd      = safe_double(node, "Kd",      defaults.Kd);
  p.out_min = safe_double(node, "out_min", defaults.out_min);
  p.out_max = safe_double(node, "out_max", defaults.out_max);
  p.int_max = safe_double(node, "int_max", defaults.int_max);
  return p;
}

// ─────────────────────────────────────────────────
SimulationConfig loadConfig(const std::string & yaml_path)
{
  YAML::Node root = YAML::LoadFile(yaml_path);
  if (!root.IsDefined())
    throw std::runtime_error("Config file is empty or unreadable: " + yaml_path);

  SimulationConfig cfg;  // 全部默认

  // ── model ──
  if (root["model"] && root["model"].IsDefined()) {
    const auto & m = root["model"];
    cfg.model.M_c = safe_double(m, "M_c", cfg.model.M_c);
    cfg.model.m_p = safe_double(m, "m_p", cfg.model.m_p);
    cfg.model.L   = safe_double(m, "L",   cfg.model.L);
    cfg.model.I   = safe_double(m, "I",   cfg.model.I);
    cfg.model.k   = safe_double(m, "k",   cfg.model.k);
    cfg.model.g   = safe_double(m, "g",   cfg.model.g);
  }

  // 自动计算 I (必须在参数校验之前)
  resolveInertia(cfg.model);

  // 参数校验
  if (cfg.model.M_c <= 0.0 || cfg.model.m_p <= 0.0 || cfg.model.L <= 0.0 || cfg.model.I <= 0.0)
    throw std::runtime_error(
      "ModelConfig: M_c, m_p, L, I 必须 > 0.");
  if (cfg.model.k < 0.0 || cfg.model.g <= 0.0)
    throw std::runtime_error(
      "ModelConfig: k 必须 >= 0, g 必须 > 0.");

  // ── initial_state ──
  if (root["initial_state"] && root["initial_state"].IsDefined()) {
    const auto & s = root["initial_state"];
    cfg.initial_state.y         = safe_double(s, "y",         cfg.initial_state.y);
    cfg.initial_state.theta_deg = safe_double(s, "theta_deg", cfg.initial_state.theta_deg);
    cfg.initial_state.dy        = safe_double(s, "dy",        cfg.initial_state.dy);
    cfg.initial_state.dtheta    = safe_double(s, "dtheta",    cfg.initial_state.dtheta);
  }

  // ── controller ──
  if (root["controller"] && root["controller"].IsDefined()) {
    const auto & c = root["controller"];

    cfg.controller.dt       = safe_double(c, "dt",       cfg.controller.dt);
    cfg.controller.sim_time = safe_double(c, "sim_time", cfg.controller.sim_time);

    cfg.controller.outer_pid = loadPidParams(c["outer_pid"], cfg.controller.outer_pid);
    cfg.controller.inner_pid = loadPidParams(c["inner_pid"], cfg.controller.inner_pid);
  }

  if (cfg.controller.dt <= 0.0 || cfg.controller.sim_time <= 0.0)
    throw std::runtime_error("ControllerConfig: dt 和 sim_time 必须 > 0.");

  return cfg;
}

// ─────────────────────────────────────────────────
void resolveInertia(ModelConfig & cfg)
{
  if (cfg.I < 0.0) {
    cfg.I = (1.0 / 3.0) * cfg.m_p * cfg.L * cfg.L;   // 均质杆绕端点
  }
}

}  // namespace cart_pole_controller
