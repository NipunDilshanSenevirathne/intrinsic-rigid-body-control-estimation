# 🌀 Intrinsic Rigid-Body PID Control, Extended Kalman Filter, and Symmetric Systems on Lie Groups

This repository compiles a series of interactive notebooks and Python simulations covering:

* **Intrinsic PID control** for mechanical systems on Lie groups,
* **Discrete Invariant Extended Kalman Filters (DEKF)**, and
* **Symmetric systems** arising from Lie group actions and reduction.

The material is presented from a **coordinate-free geometric perspective** with applications to **rigid-body motion**, **estimation**, and **mechanical system symmetry reduction**.

---

### 📘 Related Repository

For a complementary Lie-group–based treatment of classical mechanics and geometric simulation tools, see:
👉 [classical-mechanics-from-a-geometric-point-of-view](https://github.com/mugalan/classical-mechanics-from-a-geometric-point-of-view)

---

**Author:**
**D. H. S. Maithripala, Ph.D.**
University of Peradeniya, Sri Lanka
📧 [smaithri@eng.pdn.ac.lk](mailto:smaithri@eng.pdn.ac.lk)
🌐 [Faculty Profile](https://eng.pdn.ac.lk/ME/People/FacultyProfiles.php?id=6) | [ORCID](https://orcid.org/0000-0002-8200-2696)

---

## ⚙️ Intrinsic AGLES PID Controller for Mechanical Systems on Lie Groups

**Notebook:** [Intrinsic PID on Lie Groups](https://github.com/mugalan/intrinsic-rigid-body-control-estimation/blob/main/rigid-body-control/Intrinsic_PID_on_Lie_Groups.ipynb)

This notebook implements the **geometric AGLES PID controller** (Almost Globally Locally Exponentially Stable) for fully actuated rigid-body systems evolving on nonlinear configuration manifolds modeled by Lie groups.

The controller operates in **momentum space**, enabling coordinate-free trajectory tracking with provable stability.

---

### ✒️ Controller Overview

The control law lifts the classical PID structure for a double integrator to a Lie group $G$.
Key components include:

* **Right-invariant configuration error**
  $e = g_r g^{-1}$, capturing configuration mismatch.
* **Momentum-based error dynamics**
  Linearized in the cotangent (momentum) space.
* **Geometric integral term**
  Defined intrinsically on $G$, ensuring coordinate independence.
* **Morse-type error function**
  Ensures almost-global stability on compact Lie groups such as $SO(3)$.

---

### 🚀 Significance

This controller achieves **almost-global coordinate-free tracking** of rigid-body trajectories — avoiding Euler angle singularities and quaternion ambiguities.

It:

* Exploits **Lie group structure** for intrinsic error definitions,
* Provides **PID-like tuning** in momentum space,
* Guarantees **local exponential convergence** for almost all initial conditions.

---

### 🌍 Almost-Global Convergence

Due to topological constraints (e.g., non-contractibility of $SO(3)$), smooth controllers cannot achieve global asymptotic stability.
However, this controller attains **AGLE** behavior:

* Desired configuration is asymptotically stable from almost all initial conditions.
* Only a measure-zero set corresponds to unstable equilibria (antipodal points).
* Convergence is **locally exponential** near the desired trajectory.

---

### 🧮 Right–Invariant Dynamics and Control Law

The momentum-space dynamics of a mechanical system on a Lie group $G$ are:

$$
\dot{g} = \omega \cdot g, \quad \dot{\pi} = f^e + f^u,
$$

with $\pi = \mathrm{Ad}*g^* \mathbb{I} \mathrm{Ad}*{g^{-1}}\omega$.
Using the right-invariant error $e = g_r g^{-1}$, we define the **momentum error**:

$$
\pi_e = \mathrm{Ad}_{e^{-1}}^* \pi_r - \pi.
$$

The AGLES–PID control law is then:

$$
f^u =
\left(\mathrm{Ad}*{e^{-1}}^*\dot{\pi}*r + \mathrm{ad}*{\omega_e}^*\mathrm{Ad}*{e^{-1}}^*\pi_r - f^e\right)-k_p\pi_e - k_d\pi_e - k_I\pi_I,
$$

with integral error $\dot{\pi}_I = \pi_e$.

The resulting closed-loop dynamics are linear in the momentum errors.

---

### 📚 References

* D.H.S. Maithripala & J.M. Berg, *An intrinsic PID controller for mechanical systems on Lie groups*, **Automatica**, 54:189–200, 2015.
* R.S. Chandrasekaran, R.N. Banavar, A.D. Mahindrakar, D.H.S. Maithripala, *Geometric PID controller for nonholonomic systems on Lie groups*, **Automatica**, 165, 111658, 2024.
* D.H.S. Maithripala, J.M. Berg, W.P. Dayawansa, *Almost-global tracking of simple mechanical systems on Lie groups*, **IEEE TAC**, 51(2):216–225, 2006.

---

## 🔄 Symmetric Systems and Lie Group Reduction

Many mechanical systems exhibit **symmetries** — invariances under the smooth action of a Lie group $H$ on a configuration space $G$.
Such systems can be **reduced** via symmetry to a lower-dimensional quotient manifold $G/H$, simplifying analysis and control.

### 🧩 Key Concepts

* **Principal bundle:**
  $H \hookrightarrow G \xrightarrow{\pi} G/H$, where $G/H$ is the reduced (shape) space.

* **Connection:**
  A smooth distribution that defines horizontal and vertical motions — separating “shape change” from “internal rotation”.

* **Reduction:**
  Dynamics on $G$ project to reduced equations on $G/H$ via the connection, leading to elegant geometric interpretations such as **holonomy** and **geometric phase**.

---

### ⚙️ Example — $SO(3) \to S^2$

In the rotation group $SO(3)$ with right $SO(2)$-symmetry about a fixed axis:

$$
SO(2) \hookrightarrow SO(3) \xrightarrow{\pi} S^2, \quad \pi(R) = Re_3.
$$

The base variable $r = Re_3$ represents the body’s symmetry axis on the sphere, while the fibre variable corresponds to spin about it.
When $r(t)$ completes a closed loop on $S^2$, the total attitude $R(t)$ accumulates a **geometric phase** — equal to the negative solid angle subtended by the loop.

This geometric structure underlies phenomena like **precession**, **Berry phase**, and **Hannay–Berry holonomy** in mechanics.

---

## 🧮 Discrete Invariant Extended Kalman Filter (DEKF) on Lie Groups

**Notebook:** [Rigid Body Intrinsic EKF](https://github.com/mugalan/intrinsic-rigid-body-control-estimation/blob/main/intrinsic-DEKF/RigidBodyIntinsicEKF_DHSM.ipynb)

The DEKF provides an **intrinsic, structure-preserving state estimator** for systems evolving on Lie groups (e.g., $SO(3)$).
Unlike standard EKFs, it maintains **group consistency**, ensuring unbiased and stable performance.

---

### 🧠 Highlights

* Derived directly from **Lie algebra error dynamics**.
* Works for **attitude and pose estimation** on $SO(3)$ and $SE(3)$.
* Uses **invariant linearization** — prediction and correction equations remain consistent regardless of the reference frame.
* Demonstrated through Monte Carlo simulation with IMU-style measurements.

---

### 📊 Simulation Features

* Covariance evolution and 1σ uncertainty visualization.
* Automatic tuning via process–measurement noise ratio.
* Covariance inflation and consistency metrics.
* Trace error plots for attitude misalignment.

---

## 🧭 Summary

This repository unifies **control, estimation, and symmetry reduction** for mechanical systems on Lie groups, providing:

* A geometric foundation for **intrinsic control and filtering**,
* Numerically robust simulation tools (Python + Plotly visualizations),
* Clear links between **theory and implementation**.

Ideal for researchers working in:

* Rigid-body dynamics
* Geometric mechanics
* Robotics and spacecraft attitude control
* Nonlinear filtering on manifolds

---

## 🔖 License

MIT License © 2025 D.H.S. Maithripala
