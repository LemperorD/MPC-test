# MPC-test

使用MPC完成倒立摆控制问题

## 1. 建模

考虑如下图所示的平面倒立摆系统。摆杆的支点固定于可以水平移动的小车上。

![alt text](assets/image.png)

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

则系统方程最终可提取为如下内容：

$$
\begin{cases}
(I+mL^2)\ddot{\theta}
=
mgL\sin\theta
-
mL\cos\theta\,\ddot{y}, \\[8pt]

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
$$

> 推导过程详见assets

## 2. 具体任务
