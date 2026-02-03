# First-Order Boustrophedon Navigator — PD Controller Tuning (ROS 2 + turtlesim)

This repository contains the implementation and tuning of a **PD controller** for a **First-Order Boustrophedon Navigator** using **ROS 2** and **turtlesim**. The goal is to achieve stable, smooth, and accurate coverage motion while respecting the constraints of a first‑order kinematic model.

The environment (ROS 2 workspace, nodes, and turtlesim setup) is provided as part of the assignment. This README focuses on **how the controller was tuned**, **why specific gains were selected**, and **how performance was evaluated**.

## 1. Problem Overview

* **Platform:** turtlesim (unicycle-like kinematics)
* **Control Objective:**

  * Track straight-line segments in a boustrophedon (back-and-forth) coverage pattern
  * Perform smooth turns at lane boundaries
  * Minimize lateral tracking error and heading error
* **Controller Type:** PD controller

---

## 2. Controller Formulation

The PD control law is defined as:

velocity(t) = Kp * e(t) + Kd * de(t)/dt```

where:
- `e(t)` is the error 
- `de(t)/dt` is the time derivative of the error
- `Kp` is the proportional gain
- `Kd` is the derivative gain

A similar formulation is used for angular control where *velocity(t)* is replaced by *omega(t)* and respective errors along with their derivatives are used.

---

## 3. Tuning Methodology

The tuning process followed a structured approach which is defined below:

### 3.1 Initial Gain Selection

1. **Start with Proportional control**
   - Set `Kd_linear = 0` and `Kd_angular = 0`
   - Increase `Kp_linear` and `Kp_angular` gradually until the turtle reliably follows the straight and turning segments

2. **Add derivative control**
   - Add a small `Kd_linear` and `Kd_angular` to damp oscillations
   - Increase their values until oscillations are noticeably reduced

### 3.2 Incremental Refinement

After obtaining a stable baseline, gains were incrementally adjusted to improve cornering behavior and to reduce the average cross tracking error. 

---

## 4. Performance Metrics

Metrics used to evaluate controller performance:

- **Maximum cross-track error (m)**
- **Average tracking error (m)**
- **Qualitative smoothness** (visual inspection)

---

## 5. Performance Plots

Plots generated for each gain set:

1. **Trajectory Plot**: Desired boustrophedon path vs. actual turtle trajectory
2. **Tracking Error vs. Time**: Cross-track or heading error
3. **Linear and Angular Velocities vs. Time**: Used to detect aggressive or noisy control action

---

## 6. Comparison of Different Parameter Sets

| Gain Set | Kp_linear | Kd_linear | Kp_angular | Kd_angular | Comments                          |
|----------|-----------|-----------|------------|------------|-----------------------------------|
| A        | 2.0       | 0.0       | 8.5        | 0.0        | Good tracking, inconsistent turns |
| B        | 1.5       | 0.0       | 9.0        | 0.1        | Smoother turns with good tracking |
| C        | 1.25      | 0.4       | 9.0        | 0.04       | Improved cross-track error        |

### Observations

- Increasing `Kp` reduces steady-state error but increases oscillations
- Adding `Kd` improves damping and turn behavior
- Excessively high `Kp` leads to sharp, non-smooth motion unsuitable for coverage tasks

### Figures

#### Gain set A

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/first_order_boustrophedon_navigator/Figures/setA/cross_track_error.png" />

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/first_order_boustrophedon_navigator/Figures/setA/trajectory.png" />

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/first_order_boustrophedon_navigator/Figures/setA/velocity_profiles.png" />

#### Gain set B

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/first_order_boustrophedon_navigator/Figures/setB/cross_track_error.png" />

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/first_order_boustrophedon_navigator/Figures/setB/trajectory.png" />

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/first_order_boustrophedon_navigator/Figures/setB/velocity_profiles.png" />

#### Gain set C

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/first_order_boustrophedon_navigator/Figures/setC/cross_track_error.png" />

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/first_order_boustrophedon_navigator/Figures/setC/trajectory.png" />

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/first_order_boustrophedon_navigator/Figures/setC/velocity_profiles.png" />

---

## 7. Analysis

### 7.1 Gain Set A

Gain set A relies on proportional control only and achieves acceptable tracking on straight segments. However, the lack of derivative damping leads to oscillations, especially during turns and lane transitions. Angular velocity commands are aggressive, resulting in sharp and non-smooth motion. While stable, this behavior is not ideal for coverage tasks.

### 7.2 Gain Set B

Introducing derivative action in the angular channel improves damping and significantly smooths turning behavior. Oscillations in heading and cross-track error are reduced, and trajectory tracking during lane transitions is more consistent. Linear motion remains stable, though small tracking errors persist on straight segments. Overall smoothness is improved compared to gain set A.

### 7.3 Gain Set C

Gain set C adds derivative control to both linear and angular channels, yielding the best overall performance. Cross-track error is reduced and converges faster after disturbances, with minimal oscillations. Velocity profiles are smooth and well-bounded across straight segments and turns. This set provides the best balance between accuracy and smoothness.

---
## 8. Challenges Encountered and Solutions

### 8.1 Tracking Oscillations
- **Issue:** Sustained oscillations with P-only control
- **Solution:** Introduced derivative control to damp oscillations, reduced `Kp_linear` and increased `Kp_angular`. 

### 8.2 Overshoot at Lane Transitions
- **Issue:** Heading overshoot during 180° turns
- **Solution:** Increased `Kd_angular` to slow down heading changes and slightly reduced linear velocity during turns

### 8.3 Noise Amplification in Derivative Term
- **Issue:** Noisy angular velocity commands when `Kd_angular` was too high
- **Solution:** Reduced `Kd_angular`

---

## 9. Final Tuned Parameters

- Kp_linear = 1.25
- Kd_linear = 0.4
- Kp_angular = 9.0
- Kd_angular = 0.04

These values provide the best balance between accuracy, smoothness, and robustness across multiple runs.

---

## 10. Conclusion

A simple PD controller can achieve reliable boustrophedon coverage when tuned systematically. Derivative control is essential for smooth motion and stable turning in such problems.

---

## Author

**Maaz Shahzad**, 
M.S. Robotics and Autonomous Systems, Arizona State University

