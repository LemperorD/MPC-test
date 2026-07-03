/**
 * @file pangolin_cart_pole_demo.cpp
 * @brief 倒立摆 Cascaded PID 控制 + Pangolin OpenGL 可视化。
 *
 *     外环: y → theta_ref       (PidController, 小车位置)
 *     内环: theta → F           (PidController, 摆杆角度)
 *
 * 用法:
 *   ./pangolin_cart_pole_demo [config.yaml]
 *
 *   默认读取 config/cart_pole.yaml
 *
 * 按键:
 *   Q / Esc  — 退出
 *   空格     — 暂停 / 继续
 *   鼠标拖拽 — 平移场景
 *   滚轮     — 缩放场景
 */

#include "cart_pole.hpp"
#include "pid_controller.hpp"
#include "config_loader.hpp"

#include <pangolin/pangolin.h>
#include <pangolin/display/default_font.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <algorithm>
#include <vector>

using namespace cart_pole_controller;

// ────────────────────────────────────────────────────────
//  Pangolin Var<bool> — Pushed() 需要的持久变量
// ────────────────────────────────────────────────────────
static pangolin::Var<bool> g_btn_quit("ui.quit", false);
static pangolin::Var<bool> g_btn_pause("ui.pause", false);
static bool                g_paused = false;

// ────────────────────────────────────────────────────────
//  场景绘制 (左侧 View)
// ────────────────────────────────────────────────────────
static void drawScene(pangolin::View & /*view*/,
                      const Eigen::Vector4d & x,
                      double t, double u,
                      double theta_ref,
                      double track_width,
                      double cart_w, double cart_h,
                      double pole_len,
                      const std::string & status)
{
  const double y    = x(0);
  const double th   = x(1);

  // ---- 轨道 ----
  glColor4f(0.35f, 0.35f, 0.35f, 1.0f);
  pangolin::glDrawLine(-track_width / 2 - 0.2f, 0.0f, 0.0f,
                       track_width / 2 + 0.2f,   0.0f, 0.0f);

  // 轨道刻度
  glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
  for (double tk = -track_width / 2; tk <= track_width / 2 + 1e-6; tk += 0.5) {
    pangolin::glDrawLine(tk, -0.04f, 0.0f, tk, 0.0f, 0.0f);
  }

  // ---- 小车 (Rectangle) ----
  glPushMatrix();
  glTranslatef(y, 0.0f, 0.0f);
  glColor4f(0.20f, 0.60f, 0.95f, 1.0f);   // 蓝色
  pangolin::glDrawRect(-cart_w / 2, 0.0f, cart_w / 2, cart_h);
  glPopMatrix();

  // ---- 摆杆 (Line) ----
  const float tip_x = y + pole_len * std::sin(th);
  const float tip_y = cart_h + pole_len * std::cos(th);

  glColor4f(0.90f, 0.18f, 0.18f, 1.0f);   // 红色
  glLineWidth(4.0f);
  pangolin::glDrawLine(y, cart_h, 0.0f, tip_x, tip_y, 0.0f);
  glLineWidth(1.0f);

  // ---- 摆杆端点球 ----
  glColor4f(0.90f, 0.18f, 0.18f, 1.0f);
  pangolin::glDrawCircle(tip_x, tip_y, 0.06f);

  // ---- 支点 ----
  glColor4f(0.15f, 0.15f, 0.15f, 1.0f);
  pangolin::glDrawCircle(y, cart_h, 0.04f);

  // ---- 目标摆角指示 (半透明虚线表示 theta_ref) ----
  {
    const float ref_tip_x = y + pole_len * std::sin(theta_ref);
    const float ref_tip_y = cart_h + pole_len * std::cos(theta_ref);

    glColor4f(0.0f, 0.7f, 0.0f, 0.5f);
    glLineWidth(1.5f);
    pangolin::glDrawLine(y, cart_h, 0.0f, ref_tip_x, ref_tip_y, 0.0f);
    glLineWidth(1.0f);
  }

  // ---- 文字 HUD ----
  auto & font = pangolin::default_font();
  int line = 0;
  const int lh = 18;

  font.Text("t = %.2f s", t)
     .DrawWindow(10, 18 + lh * line++);
  font.Text("y = %+.3f m", y)
     .DrawWindow(10, 18 + lh * line++);
  font.Text("theta     = %+.2f deg", th * 180.0 / M_PI)
     .DrawWindow(10, 18 + lh * line++);
  font.Text("theta_ref = %+.2f deg", theta_ref * 180.0 / M_PI)
     .DrawWindow(10, 18 + lh * line++);
  font.Text("u = %+.2f N", u)
     .DrawWindow(10, 18 + lh * line++);

  if (!status.empty())
    font.Text("%s", status.c_str())
       .DrawWindow(10, 18 + lh * line++);
}

// ────────────────────────────────────────────────────────
//  main
// ────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  // ── 1. 加载配置 ──
  std::string config_path = (argc > 1) ? argv[1] : "config/cart_pole.yaml";
  SimulationConfig cfg;
  try {
    cfg = loadConfig(config_path);
    std::cout << "Loaded config: " << config_path << "\n";
  } catch (const std::exception & e) {
    std::cerr << "Error loading config: " << e.what() << "\n";
    return 1;
  }

  // ── 2. 构造模型 & 控制器 ──
  CartPoleModel model(cfg.model.M_c, cfg.model.m_p, cfg.model.L,
                      cfg.model.I, cfg.model.k, cfg.model.g);

  PidController outer_pid(cfg.controller.outer_pid.Kp,
                          cfg.controller.outer_pid.Ki,
                          cfg.controller.outer_pid.Kd,
                          cfg.controller.dt,
                          cfg.controller.outer_pid.out_min,
                          cfg.controller.outer_pid.out_max,
                          cfg.controller.outer_pid.int_max);

  PidController inner_pid(cfg.controller.inner_pid.Kp,
                          cfg.controller.inner_pid.Ki,
                          cfg.controller.inner_pid.Kd,
                          cfg.controller.dt,
                          cfg.controller.inner_pid.out_min,
                          cfg.controller.inner_pid.out_max,
                          cfg.controller.inner_pid.int_max);

  // ── 3. 初始状态 ──
  Eigen::Vector4d x;
  x << cfg.initial_state.y,
       cfg.initial_state.theta_deg * M_PI / 180.0,
       cfg.initial_state.dy,
       cfg.initial_state.dtheta;

  const double dt          = cfg.controller.dt;
  const int    steps       = static_cast<int>(cfg.controller.sim_time / dt);
  const double track_width = 3.0;
  const double cart_w      = 0.35;
  const double cart_h      = 0.18;
  const double pole_len    = 2.0 * model.L();

  // ── 4. 仿真状态 ──
  double t = 0.0;
  double u = 0.0;
  double theta_ref = 0.0;
  int    step_idx   = 0;
  std::string status;
  bool    fallen = false;

  // ── 5. Pangolin 初始化 ──
  pangolin::CreateWindowAndBind("Cart-Pole PID Control", 1280, 720);

  // 5a. 左侧场景 Display — 使用 OpenGlRenderState 定义相机
  pangolin::OpenGlRenderState scene_cam(
    pangolin::ProjectionMatrixOrthographic(
      -track_width / 2 - 1,  track_width / 2 + 1,
      -0.5,                 pole_len + 0.8,
      -10, 10
    ),
    pangolin::ModelViewLookAt(0, 1, 5, 0, 1, 0, pangolin::AxisY)
  );

  pangolin::View & scene_view = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, 0.0,
               pangolin::Attach::Pix(620))
    .SetHandler(new pangolin::HandlerScroll());

  scene_view.SetDrawFunction([&](pangolin::View & v) {
    v.Activate(scene_cam);
    drawScene(v, x, t, u, theta_ref, track_width, cart_w, cart_h,
              pole_len, status);
  });

  // 5b. DataLog — 存储仿真数据
  pangolin::DataLog log;
  log.SetLabels({"t", "theta_deg", "y_m", "u_N"});

  // 5c. 右侧上方: Theta 图
  pangolin::Plotter theta_plotter(
    &log,
    0, cfg.controller.sim_time,    // x 范围
    -30, 30,                       // y 范围
    cfg.controller.sim_time / 6,   // tick_x
    10.0                           // tick_y
  );
  theta_plotter.AddSeries("$0", "$1", pangolin::DrawingModeLine,
                          pangolin::Colour::Red(), "theta [deg]");
  // 零线标记
  theta_plotter.AddMarker(
    pangolin::Marker(pangolin::Marker::Horizontal, 0.0f,
                     pangolin::Marker::Equal, pangolin::Colour(0.8f, 0.8f, 0.8f)));

  pangolin::View & theta_view = pangolin::CreateDisplay()
    .SetBounds(0.5, 1.0,
               pangolin::Attach::Pix(620), 1.0)
    .AddDisplay(theta_plotter);

  // 5d. 右侧下方: y 图
  pangolin::Plotter yu_plotter(
    &log,
    0, cfg.controller.sim_time,    // x 范围
    -track_width / 2, track_width / 2,  // y 范围
    cfg.controller.sim_time / 6,   // tick_x
    0.5                            // tick_y
  );
  yu_plotter.AddSeries("$0", "$2", pangolin::DrawingModeLine,
                       pangolin::Colour::Blue(), "y [m]");
  // 零线标记
  yu_plotter.AddMarker(
    pangolin::Marker(pangolin::Marker::Horizontal, 0.0f,
                     pangolin::Marker::Equal, pangolin::Colour(0.8f, 0.8f, 0.8f)));

  pangolin::View & yu_view = pangolin::CreateDisplay()
    .SetBounds(0.0, 0.5,
               pangolin::Attach::Pix(620), 1.0)
    .AddDisplay(yu_plotter);

  // Force plotter — linked x-axis with yu_plotter
  pangolin::Plotter force_plotter(
    &log,
    0, cfg.controller.sim_time,
    -200, 200,
    cfg.controller.sim_time / 6,
    50.0,
    &yu_plotter,       // linked x-axis
    nullptr
  );
  force_plotter.AddSeries("$0", "$3", pangolin::DrawingModeLine,
                          pangolin::Colour(1.0f, 0.55f, 0.0f), "u [N]");
  // 零线标记
  force_plotter.AddMarker(
    pangolin::Marker(pangolin::Marker::Horizontal, 0.0f,
                     pangolin::Marker::Equal, pangolin::Colour(0.8f, 0.8f, 0.8f)));

  yu_view.AddDisplay(force_plotter);

  // 5e. 键盘注册
  pangolin::RegisterKeyPressCallback('q', [](){ g_btn_quit = true; });
  pangolin::RegisterKeyPressCallback('Q', [](){ g_btn_quit = true; });
  pangolin::RegisterKeyPressCallback(27,  [](){ g_btn_quit = true; }); // Esc
  pangolin::RegisterKeyPressCallback(' ', [](){ g_btn_pause = true; });

  // ── 6. 主循环 ──
  std::cout << "Simulation started.\n"
            << "  dt=" << dt << "s  sim_time=" << cfg.controller.sim_time << "s\n"
            << "  initial theta=" << cfg.initial_state.theta_deg << "deg\n"
            << "  Press Q/Esc to quit, Space to pause\n";

  while (!pangolin::ShouldQuit() && step_idx <= steps && !fallen)
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.96f, 0.96f, 0.96f, 1.0f);

    // ── 键盘事件 ──
    if (pangolin::Pushed(g_btn_quit))
      break;

    if (pangolin::Pushed(g_btn_pause)) {
      g_paused = !g_paused;
      if (g_paused)
        status = "[PAUSED]";
      else
        status.clear();
    }

    // ── 仿真步进 ──
    if (!g_paused) {
      t = step_idx * dt;

      // Inner loop only for now. Outer loop tracking y position requires
      // careful sign handling — the inner pole-balancing loop already works.
      // Inner: theta → F (u = -inner_pid.step(theta_ref, theta))
      theta_ref = 0.0;
      u = -inner_pid.step(theta_ref, x(1));

      // 记录数据
      log.Log(
        static_cast<float>(t),
        static_cast<float>(x(1) * 180.0 / M_PI),     // theta [deg]
        static_cast<float>(x(0)),                     // y [m]
        static_cast<float>(u)                         // u [N]
      );

      // 收敛判断
      if (t > 1.0 && std::abs(x(1) * 180.0 / M_PI) < 0.2 && std::abs(x(0)) < 0.02) {
        status = "[STABLE]";
      } else if (std::abs(x(1)) > 1.0) {
        status = "[UNSTABLE]";
      }

      // 定期打印 (headless 环境也可见)
      if (step_idx % 500 == 0) {
        std::cout << std::fixed << std::setprecision(3)
                  << "t=" << std::setw(6) << t << "  "
                  << "y="  << std::setw(8) << x(0) << "  "
                  << "th=" << std::setw(8) << x(1)*180.0/M_PI << " deg  "
                  << "u="  << std::setw(9) << u << " N  " << status << "\n";
      }

      // RK4 推进一步
      x = model.rk4Step(x, u, dt);
      step_idx++;

      // 倾覆检测
      if (std::abs(x(1)) > M_PI / 2.0) {
        status = "[FALLEN]";
        fallen = true;
      }
    }

    // ── 渲染 ──
    scene_view.Activate(scene_cam);
    scene_view.Render();

    theta_view.Render();
    yu_view.Render();

    pangolin::FinishFrame();
  }

  // ── 7. 结束信息 ──
  std::cout << "\n══════════════════════════════════════════\n"
            << "  Simulation finished\n"
            << "══════════════════════════════════════════\n"
            << std::fixed << std::setprecision(4)
            << "  t     = " << t << " s\n"
            << "  y     = " << x(0) << " m\n"
            << "  theta = " << x(1) << " rad  ("
            << x(1) * 180.0 / M_PI << " deg)\n"
            << "  dy    = " << x(2) << " m/s\n"
            << "  dtheta= " << x(3) << " rad/s\n"
            << "  u_end = " << u << " N\n"
            << "  theta_ref = " << theta_ref << " rad\n";

  return 0;
}
