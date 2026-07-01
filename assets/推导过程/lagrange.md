# 倒立摆动力学推导（基于拉格朗日方程）

## 1. 建立坐标系

设：

- 小车质量为 $M$
- 摆杆质量为 $m$
- 摆杆质心距离转轴为 $L$
- 摆杆绕质心转动惯量为 $I$
- 小车水平位移为 $y$
- 摆杆与竖直方向夹角为 $\theta$
- 外部输入力为 $F$
- 小车阻尼系数为 $k$

系统广义坐标为

$$
q=
\begin{bmatrix}
y\\
\theta
\end{bmatrix}.
$$

---

## 2. 摆杆质心坐标

摆杆质心坐标为

$$
\begin{aligned}
x &= y + L\sin\theta,\\
z &= L\cos\theta.
\end{aligned}
$$

对时间求导得到速度

$$
\begin{aligned}
\dot{x}
&=
\dot{y}
+
L\dot{\theta}\cos\theta,\\
\dot{z}
&=
-
L\dot{\theta}\sin\theta.
\end{aligned}
$$

因此质心速度平方为

$$
\begin{aligned}
v^2
&=
\dot{x}^2+\dot{z}^2\\
&=
(\dot{y}+L\dot{\theta}\cos\theta)^2
+
(L\dot{\theta}\sin\theta)^2\\
&=
\dot{y}^2
+
2L\dot{y}\dot{\theta}\cos\theta
+
L^2\dot{\theta}^2.
\end{aligned}
$$

---

## 3. 动能

小车动能

$$
T_c
=
\frac12 M\dot{y}^2.
$$

摆杆平动动能

$$
T_t
=
\frac12 m v^2.
$$

摆杆转动动能

$$
T_r
=
\frac12 I\dot{\theta}^2.
$$

因此系统总动能

$$
\begin{aligned}
T
&=
\frac12 M\dot{y}^2
+
\frac12 m
\left(
\dot{y}^2
+
2L\dot{y}\dot{\theta}\cos\theta
+
L^2\dot{\theta}^2
\right)
+
\frac12 I\dot{\theta}^2\\
&=
\frac12(M+m)\dot{y}^2
+
mL\dot{y}\dot{\theta}\cos\theta
+
\frac12(I+mL^2)\dot{\theta}^2.
\end{aligned}
$$

---

## 4. 势能

设竖直向上为势能正方向，则

$$
V
=
mgL\cos\theta.
$$

---

## 5. 拉格朗日函数

拉格朗日函数定义为

$$
\mathcal{L}
=
T-V.
$$

即

$$
\mathcal{L}
=
\frac12(M+m)\dot{y}^2
+
mL\dot{y}\dot{\theta}\cos\theta
+
\frac12(I+mL^2)\dot{\theta}^2
-
mgL\cos\theta.
$$

---

## 6. 广义力

对应两个广义坐标

$$
Q_y
=
F-k\dot{y},
$$

$$
Q_\theta
=
0.
$$

---

## 7. 对坐标 $y$ 应用拉格朗日方程

根据

$$
\frac{d}{dt}
\left(
\frac{\partial\mathcal{L}}
{\partial\dot{y}}
\right)
-
\frac{\partial\mathcal{L}}
{\partial y}
=
Q_y,
$$

有

$$
\frac{\partial\mathcal{L}}
{\partial\dot{y}}
=
(M+m)\dot{y}
+
mL\dot{\theta}\cos\theta.
$$

时间求导得到

$$
(M+m)\ddot{y}
+
mL\ddot{\theta}\cos\theta
-
mL\dot{\theta}^2\sin\theta.
$$

因此

$$
(M+m)\ddot{y}
+
mL\ddot{\theta}\cos\theta
-
mL\dot{\theta}^2\sin\theta
=
F-k\dot{y}.
$$

整理得

$$
(M+m)\ddot{y}
+
mL\cos\theta\,\ddot{\theta}
-
mL\sin\theta\,\dot{\theta}^2
+
k\dot{y}
=
F.
$$

---

## 8. 对坐标 $\theta$ 应用拉格朗日方程

根据

$$
\frac{d}{dt}
\left(
\frac{\partial\mathcal{L}}
{\partial\dot{\theta}}
\right)
-
\frac{\partial\mathcal{L}}
{\partial\theta}
=
0,
$$

有

$$
\frac{\partial\mathcal{L}}
{\partial\dot{\theta}}
=
mL\dot{y}\cos\theta
+
(I+mL^2)\dot{\theta}.
$$

时间求导得到

$$
mL\ddot{y}\cos\theta
-
mL\dot{y}\dot{\theta}\sin\theta
+
(I+mL^2)\ddot{\theta}.
$$

另一方面

$$
\frac{\partial\mathcal{L}}
{\partial\theta}
=
-mL\dot{y}\dot{\theta}\sin\theta
+
mgL\sin\theta.
$$

代入拉格朗日方程可得

$$
(I+mL^2)\ddot{\theta}
+
mL\ddot{y}\cos\theta
-
mgL\sin\theta
=
0.
$$

整理为

$$
(I+mL^2)\ddot{\theta}
=
mgL\sin\theta
-
mL\ddot{y}\cos\theta.
$$

---

## 9. 最终动力学方程

系统动力学方程可写为

$$
\boxed{
\begin{cases}
(I+mL^2)\ddot{\theta}
=
mgL\sin\theta
-
mL\cos\theta\,\ddot{y},\\[8pt]

(M+m)\ddot{y}
+
mL\cos\theta\,\ddot{\theta}
-
mL\sin\theta\,\dot{\theta}^{2}
+
k\dot{y}
=
F.
\end{cases}
}
$$

该方程为倒立摆系统的非线性动力学模型，可进一步在线性化后用于状态空间建模、LQR 控制、MPC 控制及其他现代控制方法的设计。
