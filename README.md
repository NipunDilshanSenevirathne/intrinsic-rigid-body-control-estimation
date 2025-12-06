# Intrinsic rigid-body PID Control, Extended Kalman Filter, and Symmetric systems on Lie Groups

This is a compilation of a set of interactive notes and supplementary material for intrinsic PID control, DEKF and Symmetric systems on LIe groups with application to rigid body motion.

As a supplement the following repo contains a Lie-Group oriented treatment of classical mechanics and a set of python tools for simulating roigid body motion and the DEKF.

https://github.com/mugalan/classical-mechanics-from-a-geometric-point-of-view

**D. H. S. Maithripala, PhD.**  
smaithri@eng.pdn.ac.lk  
https://eng.pdn.ac.lk/ME/People/FacultyProfiles.php?id=6  
https://orcid.org/0000-0002-8200-2696


---
## 🌀 Intrinsic AGLES PID Controller for mechanical system on Lie groups

This following notebook implements a **geometric AGLES PID controller** for fully actuated rigid body systems evolving on the nonlinear configuration space of a general finite dimensional Lie group. The controller operates in the **momentum space**, enabling coordinate-free tracking of configuration trajectories with provable convergence properties.

https://github.com/mugalan/intrinsic-rigid-body-control-estimation/blob/main/rigid-body-control/Intrinsic_PID_on_Lie_Groups.ipynb

### ✒️ Controller Description

The control law is derived by lifting classical PID control for a double integrator to the Lie group $G$. The key components include:

* **Right-invariant configuration error**: Defined as $e = g_r g^{-1}$, capturing the discrepancies between the current and reference trajectories.

* **Momentum-based error dynamics**: The controller is expressed in momentum errors $\pi_e$, which evolve linearly under the influence of control inputs.

* **Geometric integral terms**: Integral errors are defined on the configuration space itself, preserving geometric consistency without resorting to local coordinates or quaternions.

* **Quadratic-like error function**: The gradient of a polar Morse function on $G$, defines the proportional action in a group-consistent way.

### 🚀 Significance

This controller achieves **almos global coordinate-free tracking** of desired rigid body trajectories, avoiding singularities and ambiguities associated with parameterizations of manifolds.

By working directly in the **momentum space** and respecting the geometric structure of the configuration space, the design naturally accommodates:

* **Feedforward compensation** of desired momentum trajectories
* **Proportional-derivative-integral feedback** in a globally consistent way
* **Stability and convergence** guarantees under standard Lyapunov analysis on manifolds

### 🌍 Almost-Global Convergence

Due to the topological properties of most of the Lie groups such as the rotation group $SO(3)$, **global asymptotic stabilization** is impossible using continuous state-feedback. However, this controller achieves **almost-global locally exponential (AGLE) convergence**, meaning:

A key feature of this controller is that it leverages the **linearity of the momentum equations**. By formulating the error dynamics in terms of momentum, the control design reduces to applying a **standard PID structure** on a linear system — despite the nonlinear configuration space. This dramatically simplifies the controller implementation while preserving geometric correctness.


* The desired configuration is **asymptotically stable** from almost all initial conditions.
* The only exceptions are a measure-zero set of initial attitudes — these are **unstable saddle points** of the error function.
* The convergence is locally exponential.

This is the best possible result achievable with smooth feedback on general Lie groups, making the controller **theoretically optimal** under the constraints of continuous control.

### Summary Derivation

#### Right–Invariant Dynamics

In momentum space a mechanical system on a Lie group $G$ with Lie algebra $\mathfrak{g}$, dual $\mathfrak{g}^*$ and left invarariant kinetic energy takes the form
$$
\begin{align}
\dot{g} = \omega \cdot g, \qquad
\dot{\pi} = f^e + f^u,
\end{align}
$$
where
- $g\in G$ is the configuration of the mechanical system
- $\omega\in \mathfrak{g}$ is the right velocity
- $\pi=\operatorname{Ad}_{g}^* \mathbb{I}\operatorname{Ad}_{g^{-1}}\omega \in \mathfrak{g}^*$ is the generalized momentum 
- $\mathbb{I}$ is the constant inertial tensor resulting from the left invariant kinetic energy
- $ f^e \in \mathfrak{g}^* $ is an external (disturbance) force, and
- $ f^u \in \mathfrak{g}^* $ is the fully actuated control input.

Note that

$$
\begin{align}
\pi_r &= \operatorname{Ad}_{g}^* \mathbb{I}\operatorname{Ad}_{g_r^{-1}}\omega_r
\end{align}
$$
#### Error Dynamics
Consider the problem of finidng $f^u\in\mathfrak{g}^*$ such that $\lim_{t\to\infty} g(t)=g_r(t)$ for some sufficiently smooth refernce trajectory $g_r(t)\in G$. Let $\omega_r\in \mathfrak{g}$ as $\omega_r=\dot{g}_rg_r^{-1}$.

Define the **right–invariant tracking error**
$$
\begin{align}
e = g_r g^{-1}.
\end{align}
$$
Then:
$$
\begin{align}
\dot{e} &= \omega_e \cdot e, \qquad
\omega_e &= \omega_r - \operatorname{Ad}_e \omega.
\end{align}
$$

Define the **momentum error**
$$
\begin{align}
\pi_e = \operatorname{Ad}_{g}^* \mathbb{I}\operatorname{Ad}_{g_r^{-1}}\omega_e
       = \operatorname{Ad}_{e^{-1}}^*\pi_r - \pi.
\end{align}
$$

The **momentum error dynamics** (with right–invariant kinematics) are:
$$
\begin{align}
\boxed{
\dot{\pi}_e
= \operatorname{Ad}_{e^{-1}}^*\dot{\pi}_r
- \dot{\pi}_i
+ \operatorname{ad}_{\omega_e}^*\operatorname{Ad}_{e^{-1}}^*\pi_r.
}
\end{align}
$$

---

### AGLES–PID Control Law

Let $ f_e : G \to \mathbb{R} $ be a polar Morse function with unique minimum at the identity.  
Define

$$\begin{align}
df_e = \pi_e \cdot e, \qquad
\dot{\pi}_I = \pi_e.
\end{align}
$$

The **control law** is:
$$
\begin{align}
f_i^u =
\left(
\operatorname{Ad}_{e^{-1}}^*\dot{\pi}_r
+ \operatorname{ad}_{\omega_e}^*\operatorname{Ad}_{e^{-1}}^*\pi_r
- f_i^e
\right)
- k_p\pi_e - k_d\pi_e - k_I\pi_I.
\end{align}
$$

The **closed-loop error dynamics** are:
$$
\begin{align}
\dot{e} = \omega_e \cdot e,\\
\dot{\pi}_I = \pi_e,\\
\dot{\pi}_e = -k_p\pi_e - k_d\pi_e - k_I\pi_I.
\end{align}
$$
> “The error dynamics do not get any simpler or more straightforward than this.”



### Mathematical Conventions

**Coadjoint action:**  
$$
\begin{align}
\langle \operatorname{Ad}_g^*\mu, \zeta \rangle = \langle \mu, \operatorname{Ad}_{g^{-1}}\zeta \rangle
\end{align}
$$

**Derivative rule:**  
$$
\begin{align}
\frac{d}{dt}(\operatorname{Ad}_{e^{-1}}^*\pi)
= \operatorname{Ad}_{e^{-1}}^*\dot{\pi}
+ \operatorname{ad}_{\omega_e}^*\operatorname{Ad}_{e^{-1}}^*\pi
\end{align}
$$


### References

* D.H.S. Maithripala, Jordan M. Berg,
An intrinsic PID controller for mechanical systems on Lie groups, Automatica, Volume 54, 2015, Pages 189-200, ISSN 0005-1098,
[PDF](https://www.sciencedirect.com/science/article/pii/S0005109815000060)

* Rama Seshan Chandrasekaran, Ravi N. Banavar, Arun D. Mahindrakar, D.H.S. Maithripala,
Geometric PID controller for stabilization of nonholonomic mechanical systems on Lie groups, Automatica, Volume 165, 2024, 111658, ISSN 0005-1098, [PDF](https://www.sciencedirect.com/science/article/pii/S0005109824001511)

* D. H. S. Maithripala, J. M. Berg and W. P. Dayawansa, "Almost-global tracking of simple mechanical systems on a general class of Lie Groups," in IEEE Transactions on Automatic Control, vol. 51, no. 2, pp. 216-225, Feb. 2006, doi: 10.1109/TAC.2005.862219. [PDF](https://ieeexplore.ieee.org/abstract/document/1593897)

---

## 🌀 Discrete Invariant Extended Kalman Filter (DEKF) on Lie Groups

This repository contains the **full derivation and simulation of a discrete-time invariant Extended Kalman Filter (EKF)** on Lie groups, implemented and demonstrated in Python/Colab.
The project develops the theory step by step — from stochastic rigid-body kinematics to the final discrete filter equations — and validates the results through Monte Carlo simulations on $SO(3)$)$.

---

### 📘 Overview

The following notebook presents a **geometrically consistent formulation** of the EKF directly on a Lie group $G$, following the framework of [Barrau & Bonnabel (2017–2020)](https://arxiv.org/abs/1410.1465).

https://github.com/mugalan/intrinsic-rigid-body-control-estimation/blob/main/intrinsic-DEKF/RigidBodyIntinsicEKF_DHSM.ipynb

Unlike classical EKFs in Euclidean space, this approach respects the **group structure**, yielding **invariant error dynamics** and improved consistency.

The derivation starts from the discrete Euler form of the kinematic equation:
$$
\begin{align}
g_k &= g_{k-1}\exp(\Delta T\zeta_{k-1}), \quad\\
y_k &= \phi_{g_k^{-1}}(\gamma) + z_k, \qquad p(z_k)=\mathscr{N}(0,\Sigma_m)
\end{align}
$$


**The discrete invariant error recursion** on the Lie algebra:

Prediction (state on the group and covariance):
$$
\begin{align}
\widetilde g_k^- &= \widetilde g_{k-1}\exp{\left(\Delta T\left(\zeta_{k-1}+w_{k-1}\right)\right)},\qquad p(w_{k-1})=\mathscr{N}(0,\Sigma_q),\\
P_k^- &= A_{k-1} P_{k-1} A_{k-1}^{\top} + G_{k-1}\Sigma_qG_{k-1}^{\top}. 
\end{align}
$$
Here $\Sigma_q$ denotes the process noise covariance (same role as $\Sigma_p$ defined above).

Predicted output: consistent with the continuous-time relation $\widetilde y=\phi_{\widetilde g^{-1}}(\gamma)$, we use
$$
\begin{align}
\widetilde y_k^- = \phi_{{\widetilde{g}_k^-}^{-1}}(\gamma).
\end{align}
$$

Correction (group update and covariance update):
$$
\begin{align}
K_k &= P_k^- H_k^{\top}\big(H_k P_k^- H_k^{\top} + \Sigma_m\big)^{-1}, \\
P_k &= \big(I - \Delta TK_k H_k\big)P_k^-, \tag{52}\\
\widetilde g_k &= \exp{\big(L(y_k,\widetilde y_k^-)\big)}\widetilde g_k^-,
\end{align}
$$
$$
\begin{align}
A_{k-1}&= I\\
G_{k-1}&= \Delta T\,\operatorname{Ad}_{\widetilde{g}_{k-1}}\Phi(-\Delta T \zeta_{k-1})
\end{align}
$$
These equations mirror the **standard linear Kalman filter** form but are expressed intrinsically on the Lie group.

---

### 🧠 Features

* **Full symbolic derivation** of the discrete invariant error dynamics.
* **Implementation of the DEKF** for attitude estimation on (SO(3)).
* **Consistent handling of gyro and direction sensor noise**, with correct scaling between continuous- and discrete-time models.
* **Automatic tuning sweep** (`auto_tune_EKF`) to explore the effect of process and measurement noise ratios.
* **Diagnostic plots** for trace misalignment, covariance evolution, and steady-state uncertainty.
* **Covariance inflation / filter rejuvenation** to prevent overconfidence and maintain long-term stability.

---

### ⚙️ Implementation on $SO(3)$

#### **Sensor model**

The simulated IMU provides:
$$
\begin{align}
\Omega_\text{meas} &= \Omega_\text{true} + \mathcal{N}(0,\sigma_\omega^2I), \qquad
A_i^\text{meas} = R^\top e_i + \mathcal{N}(0,\sigma_\text{dir}^2I),
\end{align}
$$
with $\sigma_\omega$ the per-sample gyro noise and $\sigma_\text{dir}$ the unit-vector noise.

#### **Filter propagation**

$$
\begin{align}
\widetilde R_k^- &= \widetilde R_{k-1}\exp(\Delta T\Omega_{k-1}),
\quad
P_k^- = A_{k-1} P_{k-1}A_{k-1}^\top + G_{k-1}\Sigma_qG_{k-1}^\top,
\end{align}
$$
$$
\begin{align}
A_{k-1} = I_{3\times 3},\qquad
G_{k-1} = \Delta T\,\widetilde{R}_{k-1}\Phi(\Delta \widehat{\Omega}_{k-1}),\qquad
H_k =
\begin{bmatrix}
-\widehat{{\widetilde{R}_k^-}^{\top}e_1}\\
-\widehat{{\widetilde{R}_k^-}^{\top}e_2}
\end{bmatrix}.
\end{align}
$$

#### **Correction**

$$
\begin{align}
K_k &= P_k^-H_k^\top(H_kP_k^-H_k^\top+\Sigma_m)^{-1}, \qquad
\widetilde R_k = \exp(L(y_k, \widetilde y_k^-))\widetilde R_k^-.
\end{align}
$$


---

### 📊 Simulation Results

The notebook includes diagnostic plots showing:

* **Trace error** (3 - \mathrm{tr}(R_\text{true}^\top R_\text{est})) — a scalar measure of attitude misalignment.
* **Covariance magnitude** ((\lambda_{\max}(P))) and equivalent 1σ angular uncertainty.
* **Parameter sweeps** for (\Sigma_q) and (\Sigma_m), illustrating the trade-off between smoothness and responsiveness.

A well-tuned configuration (e.g. `Σ_q_factor=5`, `Σ_m_factor=0.1`) yields:

* Steady-state 1σ ≈ 1°,
* Trace error < 0.05 → ≈ 10–12° misalignment,
* Stable and consistent performance under stochastic excitation.

---

### 🧩 References

* A. Barrau and S. Bonnabel, *The Invariant Extended Kalman Filter as a Stable Observer*, IEEE TAC, 2017.
* A. Barrau and S. Bonnabel, *Intrinsic Filtering on Lie Groups with Applications to Attitude Estimation*, SIAM Review, 2020.


---


## 🧭 Summary

This repository provides a **complete derivation-to-simulation pipeline** for the Discrete Invariant EKF on Lie groups, including:

* rigorous Lie-algebra-based derivations,
* correct stochastic discretization,
* practical tuning and stability enhancements (covariance inflation).

It bridges theoretical filtering on manifolds with implementable, numerically robust code — ready for use in **robotics, UAV attitude estimation, and navigation research**.


---

## 🔖 License

MIT © DHS Maithripala
