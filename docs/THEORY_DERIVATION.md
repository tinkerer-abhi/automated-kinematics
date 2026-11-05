# Physics & Mathematical Derivations

This document provides the formal mathematical derivations underlying the automated kinematics processing, derivative filtering, parameter extraction, and energy conservation verification.

---

## 1. Equation of Motion: Damped Harmonic Oscillator

Consider a mass $m$ attached to an elastic spring with spring constant $k$, subject to a viscous damping force opposing velocity:
$$F_{\text{net}} = F_{\text{restoring}} + F_{\text{damping}} = -k (x - x_{\text{eq}}) - c \dot{x}$$

Applying Newton's Second Law ($F_{\text{net}} = m \ddot{x}$):
$$m \frac{d^2 x}{dt^2} + c \frac{dx}{dt} + k (x - x_{\text{eq}}) = 0$$

Dividing through by $m$:
$$\frac{d^2 x}{dt^2} + 2\gamma \frac{dx}{dt} + \omega_0^2 (x - x_{\text{eq}}) = 0$$

Where:
- $\omega_0 = \sqrt{\frac{k}{m}}$ is the **undamped natural angular frequency** ($\text{rad/s}$)
- $\gamma = \frac{c}{2m}$ is the **decay / damping attenuation coefficient** ($\text{s}^{-1}$)
- $\zeta = \frac{\gamma}{\omega_0} = \frac{c}{2\sqrt{km}}$ is the **dimensionless damping ratio**

---

## 2. Underdamped Solution ($\zeta < 1 \implies \gamma < \omega_0$)

Substituting the trial solution $x(t) - x_{\text{eq}} = C e^{\lambda t}$ yields the characteristic algebraic equation:
$$\lambda^2 + 2\gamma \lambda + \omega_0^2 = 0$$

Roots of the characteristic equation:
$$\lambda_{1,2} = -\gamma \pm \sqrt{\gamma^2 - \omega_0^2} = -\gamma \pm i \omega_d$$

where the **damped natural angular frequency** $\omega_d$ is:
$$\omega_d = \sqrt{\omega_0^2 - \gamma^2} = \omega_0 \sqrt{1 - \zeta^2}$$

The general real solution takes the form:
$$x(t) = A_0 e^{-\gamma t} \cos(\omega_d t + \phi) + x_{\text{eq}}$$

The damped oscillation period is:
$$T_d = \frac{2\pi}{\omega_d} = \frac{2\pi}{\sqrt{\omega_0^2 - \gamma^2}}$$

The **Quality Factor ($Q$)** measures the sharpness of resonance and energy retention:
$$Q = \frac{\omega_0}{2\gamma} = \frac{1}{2\zeta}$$

---

## 3. Logarithmic Decrement Method

For two successive positive peak displacements $x_n$ and $x_{n+1}$ separated by one full period $T_d$:
$$\frac{x_n - x_{\text{eq}}}{x_{n+1} - x_{\text{eq}}} = \frac{A_0 e^{-\gamma t_n}}{A_0 e^{-\gamma (t_n + T_d)}} = e^{\gamma T_d}$$

Taking the natural logarithm yields the **Logarithmic Decrement ($\delta$)**:
$$\delta = \ln\left(\frac{x_n - x_{\text{eq}}}{x_{n+1} - x_{\text{eq}}}\right) = \gamma T_d = \frac{2\pi \gamma}{\omega_d} = \frac{2\pi \zeta}{\sqrt{1 - \zeta^2}}$$

Solving for the damping ratio $\zeta$:
$$\zeta = \frac{\delta}{\sqrt{4\pi^2 + \delta^2}}$$

---

## 4. Why Raw Numerical Differentiation Fails & The Savitzky-Golay Solution

### The Finite Difference Noise Dilemma
Suppose raw ultrasonic position samples $x[n]$ have discrete measurement noise $\epsilon[n]$ with variance $\sigma_x^2 \approx (1\text{ mm})^2$:
$$\tilde{x}[n] = x[n] + \epsilon[n]$$

Computing velocity by standard central difference:
$$\tilde{v}[n] = \frac{\tilde{x}[n+1] - \tilde{x}[n-1]}{2\Delta t} = v[n] + \frac{\epsilon[n+1] - \epsilon[n-1]}{2\Delta t}$$

The variance of velocity noise is:
$$\sigma_v^2 = \frac{2\sigma_x^2}{(2\Delta t)^2} = \frac{\sigma_x^2}{2(\Delta t)^2} \implies \sigma_v = \frac{\sigma_x}{\sqrt{2}\Delta t}$$

For $\Delta t = 0.025\text{ s}$ ($40\text{ Hz}$) and $\sigma_x = 1.5\text{ mm}$:
$$\sigma_v \approx \frac{0.0015}{\sqrt{2} \times 0.025} \approx 0.042\text{ m/s}$$

For acceleration, differentiating again:
$$\sigma_a \approx \frac{\sqrt{6}\sigma_x}{(\Delta t)^2} = \frac{\sqrt{6} \times 0.0015}{(0.025)^2} \approx 5.88\text{ m/s}^2$$

Because maximum physical acceleration during a small spring oscillation is only $2 \text{ to } 4\text{ m/s}^2$, the noise is **larger than the entire physical signal**!

### The Savitzky-Golay Digital Filter Solution
A Savitzky-Golay filter fits a local polynomial of order $p$ (e.g. $p=3$) to a sliding window of $2m+1$ points using weighted least squares:
$$p(\tau) = c_0 + c_1 \tau + c_2 \tau^2 + c_3 \tau^3$$

By analytically differentiating the fitted polynomial:
- $\dot{p}(0) = c_1 \equiv v(t)$
- $\ddot{p}(0) = 2 c_2 \equiv a(t)$

This acts as a low-pass differentiator that filters out ultrasonic sensor quantization noise while strictly preserving peak amplitude and avoiding phase distortion.

---

## 5. Conservation and Dissipation of Mechanical Energy

The instantaneous total mechanical energy of the spring oscillator is:
$$E(t) = T(t) + V(t) = \frac{1}{2} m v(t)^2 + \frac{1}{2} k (x(t) - x_{\text{eq}})^2$$

Differentiating total mechanical energy with respect to time:
$$\frac{dE}{dt} = m v \frac{dv}{dt} + k (x - x_{\text{eq}}) \frac{dx}{dt} = v \left[ m \ddot{x} + k(x - x_{\text{eq}}) \right]$$

From the equation of motion, $m \ddot{x} + k(x - x_{\text{eq}}) = -c v$:
$$\frac{dE}{dt} = v (-c v) = -c v(t)^2 \le 0$$

### Key Physical Insights:
1. Since $-c v^2 \le 0$ at all moments, **mechanical energy is strictly monotonically non-increasing**.
2. Kinetic and potential energy continuously trade back and forth at twice the oscillator frequency ($2\omega_d$).
3. The smoothed envelope of total mechanical energy decays exponentially:
   $$E_{\text{envelope}}(t) = E_0 e^{-2\gamma t}$$
4. Measuring $E(t)$ over time directly verifies the viscous loss relation $\Delta E = -\int c v^2 dt$.
