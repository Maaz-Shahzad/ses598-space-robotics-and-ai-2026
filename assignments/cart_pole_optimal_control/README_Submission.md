# Cart-Pole Optimal Control using LQR

## Overview

The **cart-pole system** is a classic benchmark problem in dynamics and control. It consists of a cart that can move horizontally and a pendulum (pole) attached to the cart via a frictionless pivot. The control objective is to apply a horizontal force to the cart such that the pole remains balanced in the upright position while maintaining acceptable cart motion.

This problem is inherently unstable and nonlinear. However, around the upright equilibrium, the system can be linearized and controlled using state-feedback methods such as the **Linear Quadratic Regulator (LQR)**.

This repository implements an LQR controller for the cart-pole system in simulation and documents the systematic tuning process used to achieve stable and efficient performance.

---

## System Model

### State Vector

The cart-pole system is represented using the state vector:

\[
x =
\begin{bmatrix}
x \\
\dot{x} \\
\theta \\
\dot{\theta}
\end{bmatrix}
\]

where:

- \(x\) = cart position (m) 
- \(\dot{x}\) = cart velocity (m/s) 
- \(\theta\) = pole angle from upright (rad) 
- \(\dot{\theta}\) = pole angular velocity (rad/s) 

---

### State-Space Representation

The linearized dynamics are represented as:

\[
\dot{x} = Ax + Bu
\]

where:

- \(A\) = system matrix 
- \(B\) = input matrix 
- \(u\) = control force applied to the cart 

---
### Physical Setup

- Inverted pendulum mounted on a cart
- Cart traversal range: ±2.5m (total range: 5m)
- Pole length: 1m
- Cart mass: 1.0 kg
- Pole mass: 1.0 kg

### Disturbance Generator

The system includes an earthquake force generator that introduces external disturbances:

- Generates continuous, earthquake-like forces using superposition of sine waves
- Base amplitude: 15.0N (default setting)
- Frequency range: 0.5-4.0 Hz (default setting)
- Random variations in amplitude and phase
- Additional Gaussian noise

---

### Baseline Performance with Default Parameters


---

### Initialization Benchmark Table

| Environment | Initial Cart Pos (m) | Initial Pole Angle (deg) | Initial Cart Vel (m/s) | Initial Pole Ang Vel (deg/s) | Stable Before Control | Notes |
|-------------|----------------------|---------------------------|-------------------------|-------------------------------|-----------------------|-------|
| Native PC   |                      |                           |                         |                               |                       |       |
| Docker      |                      |                           |                         |                               |                       |       |
| Test Run 1  |                      |                           |                         |                               |                       |       |
| Test Run 2  |                      |                           |                         |                               |                       |       |

---

## LQR Controller Design

The LQR controller computes the optimal feedback gain matrix \(K\) such that:

\[
u = -Kx
\]

The gain matrix is obtained by minimizing the quadratic cost function:

\[
J = \int_0^\infty (x^T Q x + u^T R u) dt
\]

where:

- \(Q\) = state penalty matrix 
- \(R\) = control effort penalty 

---

## Tuning Methodology

### 1. Initial Controller Design

A baseline controller was designed with moderate penalties on pole angle and angular velocity:

\[
Q = \text{diag}(q_x,\ q_{\dot{x}},\ q_\theta,\ q_{\dot{\theta}})
\]

\[
R = r
\]

Initial values were chosen to prioritize pole stabilization while allowing reasonable cart motion.

---

### 2. Iterative Parameter Adjustment

Parameters were tuned systematically based on observed performance:

- Increasing \(q_\theta\): improves pole stability 
- Increasing \(q_{\dot{\theta}}\): reduces oscillations
- Increasing \(q_x\): limits cart displacement
- Increasing \(q_{\dot{x}}\): reduces cart velocity
- Increasing \(R\): reduces aggressive control effort

---

### 3. Performance Evaluation Metrics

The following metrics were used:

- Maximum cart displacement 
- Maximum pole angular deviation 
- Maximum cart velocity 
- Maximum pole angular velocity 
- Settling time 
- Stability score (qualitative) 

---

## Simulation Results

### Cart Position vs Time

<!-- Insert cart position plot below -->
![Cart Position](docs/images/cart_position.png)

---

### Pole Angle vs Time

<!-- Insert pole angle plot below -->
![Pole Angle](docs/images/pole_angle.png)

---

### Cart Velocity vs Time

<!-- Insert cart velocity plot below -->
![Cart Velocity](docs/images/cart_velocity.png)

---

### Pole Angular Velocity vs Time

<!-- Insert pole angular velocity plot below -->
![Pole Angular Velocity](docs/images/pole_angular_velocity.png)

---

### Control Input vs Time

<!-- Insert control input plot below -->
![Control Input](docs/images/control_input.png)

---

## Performance Comparison Table

| Test ID | Q Matrix (diag) | R Value | Max Cart Disp (m) | Max Pole Angle (deg) | Max Cart Vel (m/s) | Max Pole Ang Vel (deg/s) | Stability Score | Notes |
|---------|----------------|---------|-------------------|----------------------|--------------------|---------------------------|-----------------|-------|
| 1       | [ , , , ]     |         |                   |                      |                    |                           |                 |       |
| 2       | [ , , , ]     |         |                   |                      |                    |                           |                 |       |
| 3       | [ , , , ]     |         |                   |                      |                    |                           |                 |       |
| 4       | [ , , , ]     |         |                   |                      |                    |                           |                 |       |
| 5       | [ , , , ]     |         |                   |                      |                    |                           |                 |       |

---

## Observations

- Increasing pole angle weight significantly improves stabilization. 
- Excessively high cart position weight restricts necessary cart motion. 
- Very low \(R\) produces aggressive and oscillatory control. 
- Balanced tuning results in fast stabilization with minimal cart displacement. 

(Add detailed observations here)

---

## Conclusion

The LQR controller successfully stabilizes the cart-pole system when properly tuned. The tuning process highlights the trade-off between cart motion and pole stability. Optimal selection of Q and R matrices results in fast stabilization, minimal oscillations, and efficient control effort.

Future work may include:

- Nonlinear control methods 
- Model Predictive Control (MPC) 
- Robust control under disturbances 

