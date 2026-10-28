# Mathematical Models: Vehicle Dynamics & Steering Control

> **Status (2026-10-16): background theory, partly out of date.** The code has moved on: the
> Stanley law now damps the yaw rate relative to the path curvature (`stanley_steer_control.m`,
> `lka_stanley_2025.m`), pure pursuit uses a speed lookup table for the look-ahead and the curvature
> form κ = 2·y_T/(L_d² + y_T²) (`lka_pure_pursuit.m`), and the test-bench vehicle uses saturating
> tyres with a kinematic blend below 3 m/s (`lka_single_track_dynamics.m`). The current equations and
> numbers are in [`lka_controller.md`](lka_controller.md) §2.

This document outlines the theoretical models used for the Lane Keeping Assist (LKA) system.

---

## 1. 2-DOF Lateral Bicycle Model

Following the formulation in **Rajamani (2012)** (*Vehicle Dynamics and Control*, Chapter 2), the lateral dynamics of an all-terrain vehicle are represented by the standard linear two-degree-of-freedom bicycle model.

### Coordinate System & Assumptions
- **Longitudinal speed ($V_x$)**: Assumed piecewise constant.
- **States**: Lateral velocity $V_y$ and yaw rate $r = \dot{\psi}$.
- **Input**: Front road-wheel steering angle $\delta$.

### Equations of Motion
Applying Newton's Second Law for lateral translation and yaw rotation:

$$m(\dot{V}_y + V_x r) = F_{yf} + F_{yr}$$

$$I_z \dot{r} = a F_{yf} - b F_{yr}$$

where:
- $m$: Total vehicle mass ($240\text{ kg}$)
- $I_z$: Yaw moment of inertia ($180\text{ kg}\cdot\text{m}^2$)
- $a, b$: Distance from Center of Gravity (C.G.) to front and rear axles ($a = 0.75\text{ m}, b = 0.70\text{ m}$)

### Linear Tire Formulation
For small slip angles ($\alpha \le 5^\circ$):

$$\alpha_f = \delta - \frac{V_y + a r}{V_x}, \quad \alpha_r = - \frac{V_y - b r}{V_x}$$

$$F_{yf} = C_f \alpha_f, \quad F_{yr} = C_r \alpha_r$$

where $C_f, C_r$ are the lumped cornering stiffnesses of the front and rear axles.

### State-Space Representation
$$\begin{bmatrix} \dot{V}_y \\ \dot{r} \end{bmatrix} = \begin{bmatrix} -\frac{C_f + C_r}{m V_x} & -V_x - \frac{a C_f - b C_r}{m V_x} \\ -\frac{a C_f - b C_r}{I_z V_x} & -\frac{a^2 C_f + b^2 C_r}{I_z V_x} \end{bmatrix} \begin{bmatrix} V_y \\ r \end{bmatrix} + \begin{bmatrix} \frac{C_f}{m} \\ \frac{a C_f}{I_z} \end{bmatrix} \delta$$

---

## 2. Stanley Steering Controller

Developed by Stanford Racing for the DARPA Grand Challenge and formalized by **Snider (2009)**:

### Control Law
$$\delta(t) = \theta_e(t) + \arctan\left( \frac{k_{\text{stanley}} \cdot (-e_{fa}(t))}{V_x(t) + k_{\text{soft}}} \right) - k_{\text{yaw}} \cdot r(t)$$

where:
- $e_{fa}(t)$: Signed cross-track error measured from the nearest lane waypoint to the **front axle center**.
- $\theta_e(t) = \psi_{\text{path}} - \psi_{\text{veh}}$: Orientation heading error.
- $k_{\text{stanley}}$: Cross-track gain (determines aggressiveness of lane recovery).
- $k_{\text{soft}}$: Softening constant preventing infinite gains at near-zero velocities.
- $k_{\text{yaw}}$: Yaw rate damping gain reducing overshoot under fast transitions.

### Stability
When heading error is small, $\sin(\theta_e) \approx \theta_e$ and $\dot{e}_{fa} \approx -V_x \sin(\delta - \theta_e)$, which yields an exponentially stable first-order error differential equation:

$$\dot{e}_{fa}(t) + \frac{k_{\text{stanley}}}{\sqrt{1 + \left(\frac{k_{\text{stanley}} e_{fa}}{V_x}\right)^2}} e_{fa}(t) = 0$$

---

## 3. Pure Pursuit Path Tracking

Based on **Coulter (1992)** and **Snider (2009)**, Pure Pursuit fits a circular arc from the vehicle's rear axle to a lookahead goal point on the path at distance $L_d$:

$$\delta = \arctan\left( \frac{2 L \sin(\alpha)}{L_d} \right)$$

where:
- $L = a + b$: Wheelbase.
- $\alpha$: Angle between current vehicle heading and the line-of-sight vector to the lookahead point.
- $L_d = L_{d0} + k_v V_x$: Dynamic velocity-scheduled lookahead distance.

---

## References
1. Rajamani, R. (2012). *Vehicle Dynamics and Control*. 2nd Edition, Springer.
2. Snider, J. M. (2009). *Automatic Steering Methods for Autonomous Automobile Path Tracking*. CMU-RI-TR-09-08.
