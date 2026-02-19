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
| 1 | 7.70 | 2.5 | 5.136 | 1.615 | 3.89 | 
| 2 | 5.66 | 2.5 | 5.316 | 0.955 | 3.89 | 


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
|A| 1 | [ 2.0,2.0 ,8.0 ,6.0] | 0.5 | 2.5 | 6.969 | 3.298 | 3.44 | 14.85 |
|A| 2 | [ 2.0,2.0 ,8.0 ,6.0] | 0.5 | 2.5 | 3.789 | 2.033 | 4.14 | 14.69 |  
|B| 3 | [ 5.0,5.0 ,10.0 ,8.0] | 0.25 | 2.5 | 8.240 | 4.947 | 3.10 | 27.60| 
|B| 4 | [ 5.0,5.0 ,10.0 ,8.0] | 0.25 | 2.5 | 7.393 | 3.135 | 3.36 | 72.33| 
|B| 5 | [ 5.0,5.0 ,10.0 ,8.0] | 0.25 | 2.5 | 7.419 | 3.787 | 3.33 | 36.45|  
|C| 6 | [ 10.0,10.0 ,20.0 ,15.0] | 0.1 | 0.025 | 1.038 | 1.113 | 9.69 | 120.0| 
|C| 7 | [ 10.0,10.0 ,20.0 ,15.0] | 0.1 | 0.038 | 1.440 | 1.892 | 9.54 | 120.0| 
|C| 8 | [ 10.0,10.0 ,20.0 ,15.0] | 0.1 | 0.029 | 1.446 | 2.051 | 9.55 | 120.0| 

---

### Simulation Time History Plots

#### Parameter Set A

Parameter set A represents the first iterative improvement over the baseline, doubling the state penalties and halving the control cost (R=0.5). While the stability duration doubled compared to the baseline (reaching ~14.8 seconds), the configuration was still unable to prevent the cart from hitting the ±2.5m displacement limit. Furthermore, the pole angle deviations actually increased compared to the baseline, reaching as high as 6.969°, indicating that the modest increase in penalties was insufficient to counter the disturbance forces within the cart's physical constraints.

**Test 1**:

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/cart_pole_optimal_control/Figures/setA/Figure_1.png" />

**Test 2**:

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/cart_pole_optimal_control/Figures/setA/Figure_2.png" />

#### Parameter Set B

This set further increased state penalties and reduced R to 0.25 to allow for more aggressive control action. This configuration significantly extended the duration of operation, with Test 4 surviving for 72.33 seconds. However, the higher average control effort (~3.1–4.9 N) and continued reliance on the full cart range (2.5m displacement) resulted in even larger pole angle deviations, peaking at 8.24°. While the system stayed upright longer, the "Stability Scores" remained low (~3.10–3.36) due to large maximum cart displacement values.


**Test 3**:

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/cart_pole_optimal_control/Figures/setB/Figure_1.png" />

**Test 4**:

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/cart_pole_optimal_control/Figures/setB/Figure_2.png" />

**Test 5**:

<img width="640" height="480" alt="Example terminal debug outputs for controller" src="https://raw.githubusercontent.com/Maaz-Shahzad/ses598-space-robotics-and-ai-2026/refs/heads/main/assignments/cart_pole_optimal_control/Figures/setB/Figure_3.png" />

#### Parameter Set C

Parameter Set C provided a definitive solution by applying high penalties to both position and angle while minimizing control cost (R=0.1). This configuration achieved perfect stability, maintaining operation for the full 120-second simulation duration in all tests. Most notably, the cart displacement was virtually eliminated, dropping from 2.5m in previous sets to a maximum of only 0.038m across three simulations. Pole angle deviations were also reduced drastically to nearly 1°, and Stability Scores jumped to an excellent range of 9.54–9.69. This set demonstrates that high-gain state feedback is required to maintain the cart at its home position while successfully rejecting earthquake-like disturbances.

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


