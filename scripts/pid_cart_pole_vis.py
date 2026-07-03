#!/usr/bin/env python3
"""
倒立摆 PID 控制 + matplotlib 动画可视化。

用法:
    python3 pid_cart_pole_vis.py [初始角度(deg)] [仿真时间(s)]

默认: 初始角度=5°, 仿真时间=5s

按 Q 或关闭窗口提前退出。
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, Circle
from matplotlib.lines import Line2D


# ═══════════════════════════════════════════════════════════════════════════════
#  倒立摆动力学模型
# ═══════════════════════════════════════════════════════════════════════════════

class CartPoleModel:
    """
    非线性倒立摆模型，联立求解耦合 ODE:

        (I + m*L²) θ̈ = m g L sinθ - m L cosθ·ÿ
        (M+m) ÿ + m L cosθ·θ̈ - m L sinθ·θ̇² + k ẏ = F

    状态: [y, θ, ẏ, θ̇]
    """

    def __init__(self, M_c=1.0, m_p=0.1, L=0.5, I=None, k=0.1, g=9.81):
        self.M_c = M_c   # 小车质量
        self.m_p = m_p   # 摆杆质量
        self.L   = L     # 摆杆质心到支点距离
        self.I   = I if I is not None else (1.0/3.0) * m_p * L * L
        self.k   = k     # 阻尼系数
        self.g   = g

    def f_cont(self, x, u):
        """连续动力学右端, 返回 ẋ = [ẏ, θ̇, ÿ, θ̈]"""
        y, th, dy, dth = x
        S = np.sin(th)
        C = np.cos(th)

        A = self.I + self.m_p * self.L**2    # I + mL²
        B = self.m_p * self.L                # mL

        # 联立解 ÿ, θ̈
        den = (self.M_c + self.m_p) - (B**2 * C**2) / A

        rhs_ydd = (u + B * S * dth**2 - self.k * dy
                   - (B * C * self.m_p * self.g * self.L * S) / A)

        ydd = rhs_ydd / den
        thdd = (self.m_p * self.g * self.L * S - B * C * ydd) / A

        return np.array([dy, dth, ydd, thdd])

    def rk4_step(self, x, u, dt):
        """4 阶 Runge-Kutta 一步"""
        k1 = self.f_cont(x, u)
        k2 = self.f_cont(x + 0.5 * dt * k1, u)
        k3 = self.f_cont(x + 0.5 * dt * k2, u)
        k4 = self.f_cont(x + dt * k3, u)
        return x + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)


# ═══════════════════════════════════════════════════════════════════════════════
#  PID 控制器
# ═══════════════════════════════════════════════════════════════════════════════

class PidController:
    """离散位置式 PID, 带积分限幅和输出限幅"""

    def __init__(self, Kp, Ki, Kd, dt, out_lim=(-150, 150), int_lim=50):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt = dt
        self.out_min, self.out_max = out_lim
        self.int_max = int_lim
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0
        self.first = True

    def step(self, ref, meas):
        err = ref - meas
        P = self.Kp * err
        if not self.first:
            self.integral += err * self.dt
        self.integral = np.clip(self.integral, -self.int_max, self.int_max)
        I = self.Ki * self.integral
        D = 0.0 if self.first else self.Kd * (err - self.prev_err) / self.dt
        self.prev_err = err
        self.first = False
        return np.clip(P + I + D, self.out_min, self.out_max)


# ═══════════════════════════════════════════════════════════════════════════════
#  可视化
# ═══════════════════════════════════════════════════════════════════════════════

class CartPoleVisualizer:
    """
    matplotlib 动画可视化:
      - 左侧: 倒立摆动画 (小车 + 摆杆 + 轨道)
      - 右侧: θ(t) 和 y(t) / u(t) 的实时曲线
    """

    def __init__(self, model, dt, sim_time, track_width=3.0):
        self.model = model
        self.dt = dt
        self.sim_time = sim_time
        self.track_width = track_width

        # 绘图 geometry
        self.cart_w, self.cart_h = 0.3, 0.15      # 小车尺寸
        self.pole_len = 2.0 * model.L              # 摆杆全长 (可视化用)

        # ── 创建 figure ──
        plt.rcParams.update({
            'figure.facecolor': '#f8f8f8',
            'axes.edgecolor': '#444',
            'font.size': 9,
        })
        self.fig = plt.figure("Cart-Pole PID Control", figsize=(14, 6))

        # 左侧: 场景动画
        gs = self.fig.add_gridspec(1, 2, width_ratios=[1.0, 1.4],
                                   left=0.04, right=0.98,
                                   top=0.93, bottom=0.08, wspace=0.25)
        self.ax_scene = self.fig.add_subplot(gs[0])
        self._setup_scene_axes()

        # 右侧: 数据曲线 — 两个子图上下排列
        self.ax_th = self.fig.add_subplot(gs[1].subgridspec(2, 1)[0])
        self.ax_yu = self.fig.add_subplot(gs[1].subgridspec(2, 1)[1])
        self._setup_plot_axes()

        # 历史数据 (用于曲线)
        max_points = int(sim_time / dt) + 2
        self.t_hist  = np.full(max_points, np.nan)
        self.th_hist = np.full(max_points, np.nan)
        self.y_hist  = np.full(max_points, np.nan)
        self.u_hist  = np.full(max_points, np.nan)
        self._idx = 0

        # 动画
        total_steps = int(sim_time / dt) + 1
        self.ani = animation.FuncAnimation(
            self.fig, self._animate, frames=total_steps,
            interval=dt * 1000, blit=False, repeat=False, cache_frame_data=False
        )

        self._paused = False
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)

    # ── 搭建左侧静态元素 ──
    def _setup_scene_axes(self):
        half = self.track_width / 2 + 0.5
        self.ax_scene.set_xlim(-half, half)
        self.ax_scene.set_ylim(-1.0, self.pole_len + 0.5)
        self.ax_scene.set_aspect('equal')
        self.ax_scene.set_title("Cart-Pole Scene", fontsize=11, weight='bold')
        self.ax_scene.set_xlabel("y [m]")
        self.ax_scene.set_ylabel("z [m]")
        self.ax_scene.grid(True, alpha=0.3)

        # 地面 / 轨道
        self.ax_scene.axhline(y=0, color='#555', linewidth=2, zorder=0)
        # 轨道标记
        for tick in np.linspace(-self.track_width/2, self.track_width/2, 7):
            self.ax_scene.plot(tick, -0.05, 'k|', markersize=6, zorder=0)

        # 摆杆 (Line2D, 动态更新)
        self.pole_line, = self.ax_scene.plot([], [], '#e74c3c',
                                              linewidth=4, zorder=3,
                                              solid_capstyle='round')
        # 摆杆端点球
        self.tip_ball = Circle((0, 0), 0.06, fc='#e74c3c', ec='#c0392b',
                               zorder=4, visible=False)
        self.ax_scene.add_patch(self.tip_ball)

        # 小车 (Rectangle, 动态更新位置)
        self.cart = Rectangle((0, 0), self.cart_w, self.cart_h,
                              fc='#3498db', ec='#2980b9', linewidth=1.5,
                              zorder=2)
        self.ax_scene.add_patch(self.cart)

        # 小车中心支点圆
        self.pivot = Circle((0, 0), 0.04, fc='#2c3e50', zorder=5)
        self.ax_scene.add_patch(self.pivot)

        # 信息文字
        self.info_text = self.ax_scene.text(
            0.02, 0.97, '', transform=self.ax_scene.transAxes,
            fontsize=8, fontfamily='monospace', va='top',
            bbox=dict(boxstyle='round', facecolor='#fff', alpha=0.85)
        )

    # ── 搭建右侧曲线轴 ──
    def _setup_plot_axes(self):
        t_end = self.sim_time + 0.001

        # θ 图
        self.ax_th.set_title("θ (Pole Angle) [deg]", fontsize=11, weight='bold')
        self.ax_th.set_xlim(0, t_end)
        self.ax_th.set_ylim(-10, 10)
        self.ax_th.axhline(y=0, color='gray', linestyle='--', linewidth=0.7)
        self.ax_th.set_ylabel("θ [deg]")
        self.ax_th.grid(True, alpha=0.3)
        self.line_th, = self.ax_th.plot([], [], '#e74c3c', linewidth=1.5,
                                        label='θ')
        self.ax_th.legend(loc='upper right', fontsize=8)

        # y / u 图 (双 Y 轴)
        self.ax_yu.set_title("y (Cart Position) & u (Control Force)", fontsize=11,
                             weight='bold')
        self.ax_yu.set_xlim(0, t_end)
        self.ax_yu.set_ylim(-self.track_width/2, self.track_width/2)
        self.ax_yu.axhline(y=0, color='gray', linestyle='--', linewidth=0.7)
        self.ax_yu.set_ylabel("y [m]", color='#3498db')
        self.ax_yu.grid(True, alpha=0.3)
        self.ax_u = self.ax_yu.twinx()
        self.ax_u.set_ylabel("u [N]", color='#e67e22')
        self.ax_u.set_ylim(-200, 200)
        self.line_y, = self.ax_yu.plot([], [], '#3498db', linewidth=1.5,
                                       label='y')
        self.line_u, = self.ax_u.plot([], [], '#e67e22', linewidth=1.0,
                                      alpha=0.7, label='u')
        # 合并图例
        lines = [self.line_y, self.line_u]
        labels = ['y', 'u']
        self.ax_yu.legend(lines, labels, loc='upper right', fontsize=8)

        # 末端数值标注
        self.text_th = self.ax_th.text(
            0.98, 0.92, '', transform=self.ax_th.transAxes,
            fontsize=9, fontfamily='monospace', ha='right', va='top',
            bbox=dict(boxstyle='round', facecolor='#fff', alpha=0.8)
        )
        self.text_yu = self.ax_yu.text(
            0.98, 0.92, '', transform=self.ax_yu.transAxes,
            fontsize=9, fontfamily='monospace', ha='right', va='top',
            bbox=dict(boxstyle='round', facecolor='#fff', alpha=0.8)
        )

    # ── 每帧回调 ──
    def _animate(self, frame_idx):
        if self._paused:
            return

        t = frame_idx * self.dt
        x = self._sim_state
        u = self._sim_u

        y, th, dy, dth = x

        # ── 更新场景 ──
        # 小车位置
        cart_left = y - self.cart_w / 2
        self.cart.set_xy((cart_left, 0))
        # 支点
        self.pivot.center = (y, self.cart_h)
        # 摆杆端点
        tip_x = y + self.pole_len * np.sin(th)
        tip_y = self.cart_h + self.pole_len * np.cos(th)
        self.pole_line.set_data(
            [y, tip_x],
            [self.cart_h, tip_y]
        )
        self.tip_ball.center = (tip_x, tip_y)
        self.tip_ball.set_visible(True)

        # 场景信息
        status = ""
        if t > 0.3 and abs(th * 180/np.pi) < 0.1:
            status = " [STABLE]"
        elif abs(th) > 1.2:
            status = " [FALLING]"

        self.info_text.set_text(
            f"t = {t:6.2f} s\n"
            f"y = {y:+7.3f} m\n"
            f"theta = {np.degrees(th):+7.2f} deg\n"
            f"u = {u:+8.2f} N{status}"
        )

        # ── 记录数据 ──
        self.t_hist[self._idx] = t
        self.th_hist[self._idx] = np.degrees(th)
        self.y_hist[self._idx] = y
        self.u_hist[self._idx] = u
        self._idx += 1

        # ── 更新曲线 ──
        mask = ~np.isnan(self.t_hist)
        t_vals = self.t_hist[mask]

        self.line_th.set_data(t_vals, self.th_hist[mask])
        self.line_y.set_data(t_vals, self.y_hist[mask])
        self.line_u.set_data(t_vals, self.u_hist[mask])

        # 动态调整 θ 轴
        th_abs = np.nanmax(np.abs(self.th_hist[:self._idx])) + 2 if self._idx > 0 else 10
        self.ax_th.set_ylim(-max(th_abs, 5), max(th_abs, 5))

        # 末端标注
        if self._idx > 0:
            self.text_th.set_text(
                f"theta = {self.th_hist[self._idx-1]:+.2f} deg"
            )
            self.text_yu.set_text(
                f"y = {self.y_hist[self._idx-1]:+.3f} m\n"
                f"u = {self.u_hist[self._idx-1]:+.1f} N"
            )

        # ── 推进一步 ──
        self._sim_u = -self._pid.step(0.0, th)
        self._sim_state = self._model.rk4_step(self._sim_state, self._sim_u,
                                               self.dt)

        # 倾覆检测: 动画停止并标记
        if abs(th) > np.pi / 2:
            self.ani.event_source.stop()
            self.ax_scene.text(0.5, 0.5, 'FALLEN', transform=self.ax_scene.transAxes,
                               fontsize=28, color='red', weight='bold', ha='center',
                               va='center', alpha=0.7)
            self.fig.canvas.draw_idle()
            return

        return (self.cart, self.pivot, self.pole_line, self.tip_ball,
                self.line_th, self.line_y, self.line_u,
                self.info_text, self.text_th, self.text_yu)

    # ── 键盘事件 ──
    def _on_key(self, event):
        if event.key in ('q', 'Q', 'escape'):
            plt.close(self.fig)
        elif event.key == ' ':
            self._paused = not self._paused
            if not self._paused:
                # 恢复时从下一帧继续
                pass

    def run(self,
            model: CartPoleModel,
            pid: PidController,
            x0: np.ndarray):
        """启动仿真 + 动画, 阻塞直到窗口关闭。"""
        self._model = model
        self._pid = pid
        self._sim_state = x0.copy()
        self._sim_u = 0.0

        plt.show()


# ═══════════════════════════════════════════════════════════════════════════════
#  main
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    # ── 参数 ──
    dt = 0.005                          # 仿真步长 [s]
    sim_time = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0

    # 模型
    model = CartPoleModel(
        M_c=1.0, m_p=0.1, L=0.5,
        I=(1.0/3.0) * 0.1 * 0.5**2,    # 均质杆绕端点
        k=0.1, g=9.81
    )

    # PID
    pid = PidController(
        Kp=120.0, Ki=5.0, Kd=25.0,
        dt=dt, out_lim=(-150, 150), int_lim=50
    )

    # 初始状态
    th0_deg = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    x0 = np.array([0.0, np.radians(th0_deg), 0.0, 0.0])

    print(f"Cart-Pole PID Control Simulation")
    print(f"  dt={dt}s  sim_time={sim_time}s")
    print(f"  Initial angle = {th0_deg:.1f} deg")
    print(f"  Press Q to quit, Space to pause\n")

    # 启动可视化
    vis = CartPoleVisualizer(model, dt, sim_time, track_width=3.0)
    vis.run(model, pid, x0)

    print("Simulation ended.")


if __name__ == "__main__":
    main()
