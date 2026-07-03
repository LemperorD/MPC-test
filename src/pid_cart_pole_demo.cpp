/**
 * @file pid_cart_pole_demo.cpp
 * @brief PID 控制器控制倒立摆 + 终端 ASCII 可视化。
 *
 * 使用方法：
 *   ./pid_cart_pole_demo [dt] [sim_time]
 *
 *   默认 dt = 0.01 s，仿真 10 s。
 *
 * 按 Enter 退出。
 */

#include "cart_pole.hpp"
#include "pid_controller.hpp"

#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <vector>

#ifndef _WIN32
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#endif

using namespace cart_pole_controller;

// ── 终端的行、列 ──
static int term_rows() {
#ifdef _WIN32
  return 30;
#else
  struct winsize w;
  if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &w) == 0)
    return w.ws_row;
  return 30;
#endif
}

static int term_cols() {
#ifdef _WIN32
  return 80;
#else
  struct winsize w;
  if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &w) == 0)
    return w.ws_col;
  return 80;
#endif
}

// ── 键盘非阻塞检测 ──
static bool kbhit() {
#ifdef _WIN32
  return false; // Windows 略
#else
  struct termios oldt, newt;
  int ch, oldf;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  if (ch != EOF) {
    ungetc(ch, stdin);
    return true;
  }
  return false;
#endif
}

// ── 将世界坐标映射到终端列 ──
// y ∈ [-track_width/2, +track_width/2] → 列号 [left_margin, right_margin]
static int world_to_col(double y, double track_width, int left, int right) {
  double ratio = (y + track_width / 2.0) / track_width;  // [0, 1]
  int col = left + static_cast<int>(ratio * (right - left));
  if (col < left) col = left;
  if (col > right) col = right;
  return col;
}

// ── 绘制一帧 ──
static void draw_frame(const Eigen::Vector4d & x,
                       double t, double u,
                       double track_width, const std::string &tip)
{
  const double y   = x(0);
  const double th  = x(1);  // rad

  int rows = term_rows();
  int cols = term_cols();

  // 布局分配
  const int cart_row = std::max(4, rows / 2);  // 小车所在行
  const int info_row = 2;                      // 信息区起始行

  int left  = 5;
  int right = cols - 5;

  // 摆杆长度（字符单位）
  int pole_len = std::min(std::max(static_cast<int>(rows / 4), 5),
                          static_cast<int>(rows * 0.6));

  int cart_col = world_to_col(y, track_width, left, right);

  // ── 构建画面缓冲区（用空格填充） ──
  std::vector<std::string> screen(rows, std::string(cols, ' '));

  // ---- 轨道 ----
  std::string track(right - left + 1, '-');
  // 标记中点
  int mid_col = (left + right) / 2;
  if (mid_col - left >= 0 && mid_col - left < static_cast<int>(track.size()))
    track[mid_col - left] = '+';
  screen[cart_row].replace(left, track.size(), track);

  // ---- 摆杆 ----
  for (int i = 1; i <= pole_len; ++i) {
    int r = cart_row - i;
    if (r < 0) break;

    // 摆杆顶端偏移 = L * sinθ 映射到列
    double tip_dy = i * std::sin(th) * 0.5;  // 缩放因子
    int tip_col = cart_col + static_cast<int>(tip_dy);
    if (tip_col < 0) tip_col = 0;
    if (tip_col >= cols) tip_col = cols - 1;

    screen[r][tip_col] = (i == pole_len) ? 'o' : '|';
  }

  // ---- 小车 ----
  // 画成 "[O]" 样式
  if (cart_col >= 1 && cart_col + 1 < cols) {
    screen[cart_row][cart_col - 1] = '[';
    screen[cart_row][cart_col]     = 'O';
    screen[cart_row][cart_col + 1] = ']';
  }

  // ---- 信息区 ----
  auto put_str = [&](int row, int col, const std::string & s) {
    for (size_t i = 0; i < s.size(); ++i)
      if (col + static_cast<int>(i) < cols && col + static_cast<int>(i) >= 0)
        screen[row][col + static_cast<int>(i)] = s[i];
  };

  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "t = " << std::setw(6) << t << " s";
    put_str(info_row, 2, oss.str());
  }
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "y = " << std::setw(7) << y << " m";
    put_str(info_row + 1, 2, oss.str());
  }
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    double th_deg = th * 180.0 / M_PI;
    oss << "theta = " << std::setw(7) << th_deg << " deg";
    put_str(info_row + 2, 2, oss.str());
  }
  {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "u = " << std::setw(7) << u << " N";
    put_str(info_row + 3, 2, oss.str());
  }
  put_str(info_row + 5, 2, tip);

  // ---- 清屏并输出 ----
  std::cout << "\033[2J\033[H";  // 清屏 + 光标回左上
  for (const auto & line : screen)
    std::cout << line << '\n';
  std::cout << std::flush;
}

// ─────────────────────────────────────────────────────────
// 主程序
// ─────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  // ── 参数解析 ──
  double dt       = (argc > 1) ? std::stod(argv[1]) : 0.01;
  double sim_time = (argc > 2) ? std::stod(argv[2]) : 10.0;
  int    steps    = static_cast<int>(sim_time / dt);

  std::cout << "倒立摆 PID 控制仿真\n"
            << "  dt = " << dt << " s,  sim_time = " << sim_time << " s\n"
            << "  按 Enter 提前退出 ...\n"
            << std::flush;
  std::this_thread::sleep_for(std::chrono::milliseconds(800));

  // ── 模型参数 ──
  const double M_c = 1.0;    // 小车质量  kg
  const double m_p = 0.1;    // 摆杆质量  kg
  const double L   = 0.5;    // 摆杆半长  m
  const double I   = (1.0 / 3.0) * m_p * L * L;  // 均质杆绕端点 I
  const double k   = 0.1;    // 小车阻尼  N·s/m
  const double g   = 9.81;

  CartPoleModel model(M_c, m_p, L, I, k, g);

  // ── PID 控制器（控制 θ → 0）──
  // 对于倒立摆，需要较高的比例+微分增益来克服重力的失稳效应。
  // 此处增益通过线性化模型极点配置整定。
  PidController pid(
    120.0,  // Kp
    5.0,    // Ki
    25.0,   // Kd
    dt,
    -150.0, 150.0,  // 输出限幅 ±150 N
    50.0            // 积分限幅
  );

  // ── 初始状态 ──
  // 摆杆略微倾斜，偏离直立点
  Eigen::Vector4d x;
  x << 0.0,          // y
       0.05,         // θ ≈ 2.87° 初始偏离
       0.0,          // ẏ
       0.0;          // θ̇

  const double track_width = 3.0;  // 显示范围 ±1.5 m
  const double ref_theta   = 0.0;  // 目标摆角（直立）

  // ── 数据记录 ──
  std::vector<double> t_hist, y_hist, th_hist, u_hist;

  // ── 主循环 ──
  for (int k = 0; k <= steps; ++k) {
    double t = k * dt;

    // PID 根据 θ 计算控制力
    // 物理：θ>0（摆杆右倾）→ 需要 F>0（右推小车）来"接住"
    // 标准 PID 的 error = ref - θ = -θ，输出方向相反 → 取负
    double u = -pid.step(ref_theta, x(1));

    // 记录
    t_hist.push_back(t);
    y_hist.push_back(x(0));
    th_hist.push_back(x(1));
    u_hist.push_back(u);

    // ── 可视化（每 5 帧刷新一次以减少闪烁）──
    if (k % 5 == 0) {
      std::string tip;
      // 收敛判断
      if (std::abs(x(1)) < 0.005 && t > 0.5)
        tip = "[稳定 ✓]";
      else if (std::abs(x(1)) > 0.8)
        tip = "[倾覆 ↑]";

      draw_frame(x, t, u, track_width, tip);

      // 按键检测
      if (kbhit()) {
        std::cin.ignore(1024, '\n');
        std::cout << "\n用户中断。\n";
        break;
      }
    }

    // 更新状态 (RK4)
    x = model.rk4Step(x, u, dt);

    // 倾覆检测
    if (std::abs(x(1)) > M_PI / 2.0) {
      draw_frame(x, t, u, track_width, "[已倾覆]");
      std::cout << "\n摆杆倾覆 (|θ| > 90°)，仿真结束。\n";
      break;
    }
  }

  // ── 残余记录 ──
  if (t_hist.size() > 0) {
    double t_end = t_hist.back();
    std::cout << "\n══════════════════════════════════════════\n"
              << "  仿真完成 — 末端状态\n"
              << "══════════════════════════════════════════\n"
              << std::fixed << std::setprecision(4)
              << "  t     = " << t_end << " s\n"
              << "  y     = " << x(0) << " m\n"
              << "  θ     = " << x(1) << " rad  ("
              << x(1) * 180.0 / M_PI << " deg)\n"
              << "  ẏ     = " << x(2) << " m/s\n"
              << "  θ̇    = " << x(3) << " rad/s\n"
              << "  u_end = " << u_hist.back() << " N\n";
  }

  return 0;
}
