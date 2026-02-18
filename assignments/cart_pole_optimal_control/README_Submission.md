# Cart-Pole Optimal Control using LQR

## 1. Problem Overview

The **cart-pole system** is a classic benchmark problem in dynamics and control. It consists of a cart that can move horizontally and a pendulum (pole) attached to the cart via a frictionless pivot. The control objective is to apply a horizontal force to the cart such that the pole remains balanced in the upright position while maintaining acceptable cart motion.

This problem is inherently unstable and nonlinear. However, around the upright equilibrium, the system can be linearized and controlled using state-feedback methods such as the **Linear Quadratic Regulator (LQR)**.

This repository implements an LQR controller for the cart-pole system in simulation and documents the systematic tuning process used to achieve stable and efficient performance.

---

## 2. System Model

### 2.1 State Vector

The cart-pole system is represented using the state vector (x):


`x = [x, x_dot, theta, theta_dot]`

where:

- x           = cart position (m) 
- x_dot       = cart velocity (m/s) 
- theta      = pole angle from the upright position (rad) 
- theta_dot   = pole angular velocity (rad/s) 

Note that the **state vector** is represented by **`x`** and the **cart position** is also a scalar represented by `x`.

---

### 2.2 State-Space Representation

The linearized dynamics are represented as:

**x_dot** = A * **x** + B * **u**

where:

- **A** = system matrix 

        A = [
            [0, 1, 0, 0],
            [0, 0, (m * g)/M, 0],
            [0, 0, 0, 1],
            [0, 0, ((M+m)*g)/(M*L), 0]
            ]
- **B** = input matrix 

        B = [
            [0],
            [1/M],
            [0],
            [-1/(M*L)]
        ]
- **u** = control force applied to the cart 

---
### 2.3 Physical Setup

- Inverted pendulum mounted on a cart
- Cart traversal range: ±2.5m (total range: 5m)
- Pole length (L) : 1 m
- Cart mass (M)   : 1.0 kg
- Pole mass (m)   : 1.0 kg
---
### 3. Disturbance Generator

The system includes an earthquake force generator that introduces external disturbances:

- Generates continuous, earthquake-like forces using superposition of sine waves
- Base amplitude: 15.0 N (default setting)
- Frequency range: 0.5-4.0 Hz (default setting)
- Random variations in amplitude and phase
- Additional Gaussian noise

---

## 4. Baseline Performance with Default Parameters

The following default parameters were used for baseline performance determination:
```python
# State cost matrix Q (default values)
Q = np.diag([1.0, 1.0, 1.0, 1.0])  # [x, x_dot, theta, theta_dot]

# Control cost R (default value)
R = np.array([[1.0]])  # Control effort cost
```

| Run ID   | Duration of Operation (sec) | Max cart displacement (m) | Max Pole angle (deg)  | Avg Control Effort (N) | Stability Score |
| -------- |--------|-------- | -------- |---------|---------|
| 1 | 23.24 | 2.5 | 12.33 | 1.79 | 2.46 | 
| 2 | 23.18 | 2.5 | 5.37 | 1.03 | 3.87 | 

**Run 1**:

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/cart_pole_optimal_control/Figures/Baseline/Figure_1.png" />

**Run 2**:

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/cart_pole_optimal_control/Figures/Baseline/Figure_2.png" />

---

## 5. LQR Controller Formulation

The LQR controller computes the optimal feedback gain matrix \(K\) such that:

**u** = -K * **x**

The gain matrix is obtained by minimizing the quadratic cost function:

J = ∫₀<sup>∞</sup> (xᵀ Q x + uᵀ R u) dt

where:

- **Q** = state penalty matrix 
- **R** = control effort penalty 

---

## 6. Tuning Methodology

### 1. Initial Controller Design

After the performance evaluation of the baseline controller, an initial controller was designed with moderate penalties on pole angle and angular velocity:

`Q = diag(qₓ, qdotₓ, q_θ, qdot_θ)`

`R = r`

The initial values were chosen to prioritize pole stabilization while allowing reasonable cart motion.

---

### 2. Iterative Parameter Adjustment

Parameters were tuned systematically based on observed performance:

- Increasing **q_θ**: improves pole stability 
- Increasing **qdot_θ**: reduces oscillations
- Increasing **qₓ**: limits cart displacement
- Increasing **qdotₓ**: reduces cart velocity
- Increasing **R**: reduces aggressive control effort

---

## 7. Simulation Results

Multiple simulations were performed with three distinct sets of weights in Q and R matrices. The performance comparison, time histories and observations made using these simulations are provided below.

### Performance Comparison Table

|Parameter Set| Test ID | Q Matrix (diag) | R Value | Max Cart Disp (m) | Max Pole Angle (deg) | Avg Control Effort (N) | Stability Score |Duration of Operation (sec)|
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
|A| 1 | [ 2.0,2.0 ,8.0 ,6.0] | 0.5 | 2.5 | 2.705 | 1.108 | 4.40 | 33.69 |
|A| 2 | [ 2.0,2.0 ,8.0 ,6.0] | 0.5 | 2.5 | 5.172 | 2.008 | 3.87 | 23.10 |  
|B| 3 | [ 5.0,5.0 ,10.0 ,8.0] | 0.25 | 2.5 | 7.196 | 3.977 | 3.36 | 25.74| 
|B| 4 | [ 5.0,5.0 ,10.0 ,8.0] | 0.25 | 0.091 | 0.997 | 0.744 | 9.58 | 120.0| 
|B| 5 | [ 5.0,5.0 ,10.0 ,8.0] | 0.25 | 2.5 | 8.018 | 4.608 | 3.17 | 30.54|  
|C| 6 | [ 10.0,10.0 ,20.0 ,15.0] | 0.1 | 0.039 | 1.498 | 1.510 | 9.55 | 120.0| 
|C| 7 | [ 10.0,10.0 ,20.0 ,15.0] | 0.1 | 0.092 | 3.072 | 3.436 | 9.03 | 120.0| 
|C| 8 | [ 10.0,10.0 ,20.0 ,15.0] | 0.1 | 0.087 | 2.808 | 3.325 | 9.10 | 120.0| 

---

### Simulation Time History Plots

#### Parameter Set A
Parameter set A represents the first improvement over the baseline controller, with increased penalties on pole angle and angular velocity while maintaining moderate penalties on cart position and velocity. This configuration resulted in improved pole stabilization compared to the default parameters, with reduced maximum pole angle deviations (2.7°–5.2°). However, the cart still reached its displacement limits (±2.5 m), indicating that cart motion was not sufficiently constrained. The controller applied moderate control effort, resulting in smoother response compared to aggressive configurations. Stability duration improved in one test but remained limited in another, suggesting that while pole stabilization was improved, robustness to disturbances was still inconsistent.

**Test 1**:

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/cart_pole_optimal_control/Figures/setA/Figure_1.png" />

**Test 2**:

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/cart_pole_optimal_control/Figures/setA/Figure_2.png" />

#### Parameter Set B

This set of parameters increased the penalties on all state variables and reduced the control penalty, allowing the controller to respond more aggressively to disturbances. This resulted in further improved stabilization performance under favorable disturbance conditions, as demonstrated in Test 4, where the cart displacement remained extremely small (0.091 m) and the pole angle deviation was limited to less than 1°. This configuration achieved excellent stability, maintaining equilibrium for the full simulation duration (120 s). However, performance varied across runs, with some tests still reaching displacement limits and exhibiting higher control effort. This variability indicates improved responsiveness but reduced consistency under certain disturbance conditions. 

**Test 3**:

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/cart_pole_optimal_control/Figures/setB/Figure_1.png" />

**Test 4**:

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/cart_pole_optimal_control/Figures/setB/Figure_2.png" />

**Test 5**:

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/cart_pole_optimal_control/Figures/setB/Figure_3.png" />

#### Parameter Set C

Parameter Set C applied the highest penalties on all state variables while significantly reducing the control penalty, allowing the controller to prioritize stabilization aggressively. This configuration produced the best overall performance among all tested parameter sets. Cart displacement was minimized to less than 0.1 m in all tests, representing a substantial improvement over previous configurations. Pole angle deviations were also kept low (approximately 1.5°–3°), indicating excellent stabilization performance. Importantly, the controller maintained stability for the entire simulation duration (120 s) in all tests, demonstrating strong robustness to continuous disturbances. Although the control effort increased compared to the baseline and set A configurations, it remained within reasonable limits and contributed to faster stabilization and minimal oscillations. This parameter set achieved the optimal balance between stability, disturbance rejection, and cart motion constraint.

**Test 6**:

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/cart_pole_optimal_control/Figures/setC/Figure_1.png" />

**Test 7**:

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/cart_pole_optimal_control/Figures/setC/Figure_2.png" />

**Test 8**:

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/cart_pole_optimal_control/Figures/setC/Figure_3.png" />

---

## Observations

- Increasing cart position and velocity weights significantly reduces cart displacement and improves positional regulation.
- Increasing pole angle weight significantly improves stabilization. 
- Increasing angular velocity weight reduces oscillations and improves transient response smoothness.
- Excessively high cart position weight restricts necessary cart motion. 
- Lower control penalty (R) improves disturbance rejection by allowing stronger corrective action.
- Balanced tuning results in fast stabilization with minimal cart displacement. 

---

## Conclusion

The LQR controller successfully stabilizes the cart-pole system when properly tuned. The tuning process highlights the trade-off between cart motion and pole stability. Optimal selection of Q and R matrices results in fast stabilization, minimal oscillations, and efficient control effort.


