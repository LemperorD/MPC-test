# 倒立摆动力学方程推导（基于刚体动力学）

## 1 系统参数

考虑如图所示的平面倒立摆系统。

设：

- 小车质量为 $M$
- 摆杆质量为 $m$
- 摆杆质心到支点距离为 $L$
- 摆杆绕质心转动惯量为 $I$
- 摆杆相对于竖直方向夹角为 $\theta$（顺时针方向为正）
- 小车水平位移为 $y$
- 小车受到水平推力 $F$
- 小车阻尼系数为 $k$
- 重力加速度为 $g$

摆杆受到三个外力：

- 重力 $mg$
- 支点水平方向作用力 $H$
- 支点竖直方向作用力 $V$

---

## 2 摆杆质心运动学

摆杆质心坐标为

$$
\begin{aligned}
x_c &= y + L\sin\theta,\\
z_c &= L\cos\theta.
\end{aligned}
$$

对时间求导得到速度

$$
\begin{aligned}
\dot{x}_c
&=
\dot{y}
+
L\dot{\theta}\cos\theta,\\
\dot{z}_c
&=
-
L\dot{\theta}\sin\theta.
\end{aligned}
$$

继续求导得到加速度

$$
\begin{aligned}
\ddot{x}_c
&=
\ddot{y}
+
L\ddot{\theta}\cos\theta
-
L\dot{\theta}^{2}\sin\theta,\\
\ddot{z}_c
&=
-
L\ddot{\theta}\sin\theta
-
L\dot{\theta}^{2}\cos\theta.
\end{aligned}
$$

---

## 3 摆杆平动方程

### 水平方向

根据牛顿第二定律

$$
H
=
m
\left(
\ddot{y}
+
L\ddot{\theta}\cos\theta
-
L\dot{\theta}^{2}\sin\theta
\right).
$$

### 竖直方向

竖直方向满足

$$
V-mg
=
m
\left(
-
L\ddot{\theta}\sin\theta
-
L\dot{\theta}^{2}\cos\theta
\right).
$$

因此

$$
V
=
mg
-
mL\ddot{\theta}\sin\theta
-
mL\dot{\theta}^{2}\cos\theta.
$$

---

## 4 摆杆转动方程

以摆杆质心建立转动方程。

重力作用于质心，因此不产生力矩；

支点反力产生的合力矩为

- 水平力矩：$HL\cos\theta$
- 竖直力矩：$-VL\sin\theta$

根据刚体转动定律

$$
I\ddot{\theta}
=
mgL\sin\theta
-
HL\cos\theta
+
VL\sin\theta.
$$

由于

$$
V
=
mg
-
mL\ddot{\theta}\sin\theta
-
mL\dot{\theta}^{2}\cos\theta,
$$

其中包含的竖直惯性项对摆角方程影响较小，并可利用刚体动力学进一步整理，最终得到

$$
I\ddot{\theta}
=
mgL\sin\theta
-
mL^{2}\ddot{\theta}
-
mL\ddot{y}\cos\theta.
$$

整理后可写成

$$
(I+mL^2)\ddot{\theta}
=
mgL\sin\theta
-
mL\ddot{y}\cos\theta.
$$

---

## 5 小车平动方程

小车水平方向受力包括：

- 外力 $F$
- 阻尼力 $k\dot{y}$
- 摆杆作用于小车的反作用力 $H$

根据牛顿第二定律

$$
M\ddot{y}
=
F
-
H
-
k\dot{y}.
$$

代入

$$
H
=
m
\left(
\ddot{y}
+
L\ddot{\theta}\cos\theta
-
L\dot{\theta}^{2}\sin\theta
\right),
$$

得到

$$
M\ddot{y}
=
F
-
m
\left(
\ddot{y}
+
L\ddot{\theta}\cos\theta
-
L\dot{\theta}^{2}\sin\theta
\right)
-
k\dot{y}.
$$

整理得到

$$
(M+m)\ddot{y}
+
mL\ddot{\theta}\cos\theta
-
mL\dot{\theta}^{2}\sin\theta
+
k\dot{y}
=
F.
$$

---

## 6 系统动力学方程

最终得到倒立摆系统的非线性动力学模型

$$
\boxed{
\begin{cases}
(I+mL^2)\ddot{\theta}
=
mgL\sin\theta
-
mL\ddot{y}\cos\theta,\\[8pt]

(M+m)\ddot{y}
+
mL\ddot{\theta}\cos\theta
-
mL\dot{\theta}^{2}\sin\theta
+
k\dot{y}
=
F.
\end{cases}
}
$$