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

---

## 7. Challenges Encountered and Solutions

### 7.1 Oscillations During Straight-Line Tracking
- **Issue:** Sustained oscillations with P-only control
- **Solution:** Introduced derivative control to damp oscillations and reduced `Kp_linear`

### 7.2 Overshoot at Lane Transitions
- **Issue:** Heading overshoot during 180° turns
- **Solution:** Increased `Kd_angular` to slow down heading changes and slightly reduced linear velocity during turns

### 7.3 Noise Amplification in Derivative Term
- **Issue:** Noisy angular velocity commands when `Kd_angular` was too high
- **Solution:** Reduced `Kd_angular`

---

## 8. Final Tuned Parameters

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

