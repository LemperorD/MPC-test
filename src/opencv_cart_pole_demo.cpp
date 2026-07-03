/**
 * @file opencv_cart_pole_demo.cpp
 * @brief 倒立摆 Cascaded PID 控制 + OpenCV 可视化。
 *
 *     外环: y → theta_ref       (PidController, 小车位置)
 *     内环: theta → F           (PidController, 摆杆角度)
 *
 * 用法:
 *   ./opencv_cart_pole_demo [config.yaml]
 *
 *   默认读取 config/cart_pole.yaml
 *
 * 按键:
 *   Q / Esc  — 退出
 *   空格     — 暂停 / 继续
 */

#include "cart_pole.hpp"
#include "pid_controller.hpp"
#include "config_loader.hpp"

#include <opencv2/opencv.hpp>

#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <algorithm>

using namespace cart_pole_controller;

// ────────────────────────────────────────────────────────
//  将世界坐标映射到 OpenCV 画布像素列
// ────────────────────────────────────────────────────────
static int world_to_px(double val, double world_min, double world_max,
                       int img_min, int img_max)
{
  double ratio = (val - world_min) / (world_max - world_min);
  int px = img_min + static_cast<int>(ratio * (img_max - img_min));
  if (px < img_min) px = img_min;
  if (px > img_max) px = img_max;
  return px;
}

// ────────────────────────────────────────────────────────
//  绘制一帧场景到 OpenCV Mat
// ────────────────────────────────────────────────────────
static void drawScene(cv::Mat & img,
                      const Eigen::Vector4d & x,
                      double t, double u,
                      double theta_ref,
                      double track_width,
                      double cart_w_px, double cart_h_px,
                      double pole_len_px,
                      const std::string & status)
{
  const int W = img.cols;
  const int H = img.rows;

  const double y   = x(0);
  const double th  = x(1);

  // 画布坐标约定: 下方是轨道, 上方是摆杆
  const int ground_y = static_cast<int>(H * 0.78);
  const int cart_top = ground_y - static_cast<int>(cart_h_px);

  const int left_margin  = static_cast<int>(W * 0.05);
  const int right_margin = static_cast<int>(W * 0.75);

  // 清空背景
  img.setTo(cv::Scalar(245, 245, 245));

  // ---- 轨道 ----
  cv::line(img,
           cv::Point(left_margin,     ground_y),
           cv::Point(right_margin,    ground_y),
           cv::Scalar(90, 90, 90), 3, cv::LINE_AA);

  // 轨道刻度
  for (double tk = -track_width / 2; tk <= track_width / 2 + 1e-6; tk += 0.5) {
    int tx = world_to_px(tk, -track_width / 2, track_width / 2,
                         left_margin, right_margin);
    cv::line(img, cv::Point(tx, ground_y - 6), cv::Point(tx, ground_y + 2),
             cv::Scalar(130, 130, 130), 1, cv::LINE_AA);
  }

  // 原点标记
  int ox = world_to_px(0.0, -track_width / 2, track_width / 2,
                       left_margin, right_margin);
  cv::line(img, cv::Point(ox, ground_y - 12), cv::Point(ox, ground_y + 4),
           cv::Scalar(60, 60, 200), 2, cv::LINE_AA);

  // ---- 小车 (矩形) ----
  int cart_left = world_to_px(y, -track_width / 2, track_width / 2,
                              left_margin, right_margin)
                  - static_cast<int>(cart_w_px / 2);
  cv::Rect cart_rect(cart_left, cart_top,
                     static_cast<int>(cart_w_px), static_cast<int>(cart_h_px));
  cv::rectangle(img, cart_rect, cv::Scalar(46, 130, 255), -1, cv::LINE_AA);  // blue fill
  cv::rectangle(img, cart_rect, cv::Scalar(30, 100, 200), 2, cv::LINE_AA);   // border

  // 支点像素
  int pivot_x = world_to_px(y, -track_width / 2, track_width / 2,
                            left_margin, right_margin);
  int pivot_y = cart_top;

  // ---- 摆杆 ----
  int tip_x = pivot_x + static_cast<int>(pole_len_px * std::sin(th));
  int tip_y = pivot_y - static_cast<int>(pole_len_px * std::cos(th));

  cv::line(img, cv::Point(pivot_x, pivot_y), cv::Point(tip_x, tip_y),
           cv::Scalar(60, 60, 235), 5, cv::LINE_AA);
  cv::circle(img, cv::Point(tip_x, tip_y), 8,
             cv::Scalar(50, 50, 230), -1, cv::LINE_AA);     // tip
  cv::circle(img, cv::Point(pivot_x, pivot_y), 5,
             cv::Scalar(40, 40, 40), -1, cv::LINE_AA);       // pivot

  // ---- 目标摆角参考线 (半透明绿色) ----
  {
    int ref_tip_x = pivot_x + static_cast<int>(pole_len_px * std::sin(theta_ref));
    int ref_tip_y = pivot_y - static_cast<int>(pole_len_px * std::cos(theta_ref));
    cv::line(img, cv::Point(pivot_x, pivot_y), cv::Point(ref_tip_x, ref_tip_y),
             cv::Scalar(100, 210, 100), 2, cv::LINE_AA);
    cv::circle(img, cv::Point(ref_tip_x, ref_tip_y), 4,
               cv::Scalar(100, 210, 100), -1, cv::LINE_AA);
  }

  // ── 右侧: θ-t 和 y/u 曲线区域 ──
  const int plot_left = right_margin + 20;
  const int plot_right = W - 20;
  const int plot_w = plot_right - plot_left;
  if (plot_w < 80) return;  // 窗口太窄不画曲线

  const int theta_plot_bottom = ground_y;
  const int theta_plot_top    = ground_y - static_cast<int>(H * 0.37);
  const int yu_plot_bottom      = theta_plot_top - 30;
  const int yu_plot_top         = static_cast<int>(H * 0.05);

  // θ 曲线区域背景
  cv::rectangle(img,
                cv::Rect(plot_left, theta_plot_top,
                         plot_w, theta_plot_bottom - theta_plot_top),
                cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
  cv::rectangle(img,
                cv::Rect(plot_left, theta_plot_top,
                         plot_w, theta_plot_bottom - theta_plot_top),
                cv::Scalar(180, 180, 180), 2, cv::LINE_AA);

  // y/u 曲线区域背景
  cv::rectangle(img,
                cv::Rect(plot_left, yu_plot_top,
                         plot_w, yu_plot_bottom - yu_plot_top),
                cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
  cv::rectangle(img,
                cv::Rect(plot_left, yu_plot_top,
                         plot_w, yu_plot_bottom - yu_plot_top),
                cv::Scalar(180, 180, 180), 2, cv::LINE_AA);

  // ---- 文字 HUD (左上角) ----
  int tx = 12, ty = 22, lh = 20;
  auto put = [&](const std::string & s, cv::Scalar color = cv::Scalar(30, 30, 30)) {
    cv::putText(img, s, cv::Point(tx, ty), cv::FONT_HERSHEY_SIMPLEX,
                0.50, color, 1, cv::LINE_AA);
    ty += lh;
  };

  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "t = " << t << " s";  put(oss.str());

    oss.str(""); oss << std::setprecision(3);
    oss << "y = " << std::showpos << y << " m";  put(oss.str(), cv::Scalar(30, 130, 230));

    oss.str(""); oss << std::noshowpos;
    oss << "theta     = " << std::showpos << std::setprecision(2)
        << th * 180.0 / M_PI << " deg";  put(oss.str(), cv::Scalar(60, 60, 235));

    oss.str(""); oss << std::noshowpos;
    oss << "theta_ref = " << std::showpos << std::setprecision(2)
        << theta_ref * 180.0 / M_PI << " deg";  put(oss.str(), cv::Scalar(100, 180, 100));

    oss.str(""); oss << std::noshowpos;
    oss << "u = " << std::showpos << std::setprecision(1) << u << " N";
    put(oss.str(), cv::Scalar(220, 140, 60));
  }

  // 状态标签
  if (!status.empty()) {
    cv::Scalar sc = (status.find("STABLE") != std::string::npos)
                    ? cv::Scalar(40, 160, 40)
                    : cv::Scalar(200, 60, 60);
    cv::putText(img, status, cv::Point(tx, ty),
                cv::FONT_HERSHEY_SIMPLEX, 0.60, sc, 2, cv::LINE_AA);
  }

  // 操作提示
  cv::putText(img, "Q/ESC:quit  SPACE:pause",
              cv::Point(W - 260, H - 12),
              cv::FONT_HERSHEY_SIMPLEX, 0.40, cv::Scalar(150, 150, 150),
              1, cv::LINE_AA);
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

  const double dt       = cfg.controller.dt;
  const int    steps    = static_cast<int>(cfg.controller.sim_time / dt);
  const double track_width = 3.0;

  // 可视化几何 (像素)
  const int img_w = 1920, img_h = 1080;
  const double cart_w_px   = 140.0;
  const double cart_h_px   = 55.0;
  const double pole_len_px = 320.0;

  // ── 4. 仿真状态 ──
  double t = 0.0;
  double u = 0.0;
  double theta_ref = 0.0;
  int    step_idx   = 0;
  std::string status;
  bool    fallen = false;
  bool    paused  = false;

  cv::Mat frame(img_h, img_w, CV_8UC3);

  // ── 5. 历史数据 (用于右侧曲线) ──
  std::vector<float> t_hist, th_hist, y_hist, u_hist;
  t_hist.reserve(steps + 2);
  th_hist.reserve(steps + 2);
  y_hist.reserve(steps + 2);
  u_hist.reserve(steps + 2);

  // ── 6. 主循环 ──
  std::cout << "Simulation started.\n"
            << "  dt=" << dt << "s  sim_time=" << cfg.controller.sim_time << "s\n"
            << "  initial theta=" << cfg.initial_state.theta_deg << "deg\n"
            << "  Press Q/Esc to quit, Space to pause\n";

  while (step_idx <= steps && !fallen)
  {
    // 键盘
    int key = cv::waitKey(1);
    if (key == 'q' || key == 'Q' || key == 27)  // Q / Esc
      break;
    if (key == ' ') {
      paused = !paused;
      status = paused ? "[PAUSED]" : "";
    }

    // 仿真步进
    if (!paused) {
      t = step_idx * dt;

      // 级联 PID 控制
      theta_ref = outer_pid.step(0.0, x(0));
      // theta_ref = 0.0;
      u = -inner_pid.step(theta_ref, x(1));

      // 记录
      t_hist.push_back(static_cast<float>(t));
      th_hist.push_back(static_cast<float>(x(1) * 180.0 / M_PI));
      y_hist.push_back(static_cast<float>(x(0)));
      u_hist.push_back(static_cast<float>(u));

      // 收敛判断
      if (t > 1.0 && std::abs(x(1) * 180.0 / M_PI) < 0.15 && std::abs(x(0)) < 0.02)
        status = "[STABLE]";
      else if (std::abs(x(1)) > 1.0)
        status = "[UNSTABLE]";

      // 打印
      if (step_idx % 500 == 0) {
        std::cout << std::fixed << std::setprecision(3)
                  << "t=" << std::setw(6) << t << "  "
                  << "y="  << std::setw(8) << x(0) << "  "
                  << "th=" << std::setw(8) << x(1)*180.0/M_PI << " deg  "
                  << "u="  << std::setw(9) << u << " N  " << status << "\n";
      }

      // RK4
      x = model.rk4Step(x, u, dt);
      step_idx++;

      // 倾覆
      if (std::abs(x(1)) > M_PI / 2.0) {
        status = "[FALLEN]";
        fallen = true;
      }
    }

    // ── 绘制场景 ──
    drawScene(frame, x, t, u, theta_ref, track_width,
              cart_w_px, cart_h_px, pole_len_px, status);

    // ── 右侧实时曲线 (叠加在场景右边) ──
    if (t_hist.size() > 1) {
      int left_margin  = static_cast<int>(img_w * 0.05);
      int right_margin = static_cast<int>(img_w * 0.75);
      int plot_left = right_margin + 20;
      int plot_right = img_w - 20;
      int plot_w = plot_right - plot_left;
      if (plot_w >= 80) {
        int ground_y = static_cast<int>(img_h * 0.78);
        int theta_bot = ground_y;
        int theta_top = ground_y - static_cast<int>(img_h * 0.37);
        int yu_bot    = theta_top - 30;
        int yu_top    = static_cast<int>(img_h * 0.05);

        float t_max = static_cast<float>(cfg.controller.sim_time);

        // θ 曲线
        {
          float th_abs_max = 0.01f;
          for (auto v : th_hist) th_abs_max = std::max(th_abs_max, std::abs(v));
          th_abs_max = std::max(th_abs_max, 5.0f);
          float th_range = th_abs_max * 1.2f;

          for (size_t i = 1; i < t_hist.size(); ++i) {
            int x0 = plot_left + static_cast<int>((t_hist[i-1] / t_max) * plot_w);
            int x1 = plot_left + static_cast<int>((t_hist[i]   / t_max) * plot_w);
            int y0 = theta_bot - 10 - static_cast<int>(((th_hist[i-1] + th_range) / (2*th_range))
                         * (theta_bot - theta_top - 20));
            int y1 = theta_bot - 10 - static_cast<int>(((th_hist[i]   + th_range) / (2*th_range))
                         * (theta_bot - theta_top - 20));
            cv::line(frame, cv::Point(x0, y0), cv::Point(x1, y1),
                     cv::Scalar(60, 60, 235), 2, cv::LINE_AA);
          }
          // 零线
          int zy = theta_bot - 10 - static_cast<int>((th_range / (2*th_range))
                     * (theta_bot - theta_top - 20));
          cv::line(frame, cv::Point(plot_left, zy), cv::Point(plot_right, zy),
                   cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
          cv::putText(frame, "theta [deg]",
                      cv::Point(plot_left + 5, theta_top + 16),
                      cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(60, 60, 235),
                      1, cv::LINE_AA);
        }

        // y 曲线
        {
          float y_range = std::max(static_cast<float>(track_width / 2 * 1.2), 0.1f);

          for (size_t i = 1; i < t_hist.size(); ++i) {
            int x0 = plot_left + static_cast<int>((t_hist[i-1] / t_max) * plot_w);
            int x1 = plot_left + static_cast<int>((t_hist[i]   / t_max) * plot_w);
            int py0 = yu_bot - 10 - static_cast<int>(((y_hist[i-1] + y_range) / (2*y_range))
                       * (yu_bot - yu_top - 20));
            int py1 = yu_bot - 10 - static_cast<int>(((y_hist[i]   + y_range) / (2*y_range))
                       * (yu_bot - yu_top - 20));
            cv::line(frame, cv::Point(x0, py0), cv::Point(x1, py1),
                     cv::Scalar(230, 140, 60), 2, cv::LINE_AA);
          }
          // 零线
          int zy = yu_bot - 10 - static_cast<int>((y_range / (2*y_range))
                   * (yu_bot - yu_top - 20));
          cv::line(frame, cv::Point(plot_left, zy), cv::Point(plot_right, zy),
                   cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
          cv::putText(frame, "y [m] / u [N]",
                      cv::Point(plot_left + 5, yu_top + 16),
                      cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(230, 140, 60),
                      1, cv::LINE_AA);
        }
      }
    }

    cv::imshow("Cart-Pole Control", frame);
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
            << "  u_end = " << u << " N\n";

  cv::destroyAllWindows();
  return 0;
}
