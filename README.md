# Advanced Control and Robotics Studies

This repository consolidates a series of simulation-based studies in nonlinear control, hybrid locomotion modeling, constrained optimization, and multibody robotic simulation.

The objective of this repository is educational and research-oriented: to explore mathematical modeling, numerical methods, and control strategies for robotic systems.

---

## Repository Structure

### 1. Locomotion

**ALIP – Angular Momentum Linear Inverted Pendulum**

- Closed-form discrete dynamics
- Hybrid step-to-step transitions
- Momentum-based foot placement control
- Multi-step walking simulation
- 2D animation and video export

Folder:
locomotion/alip


---

### 2. Manipulation

#### IK via Quadratic Programming (IK-QP)

- Differential inverse kinematics
- QP formulation with constraints
- Regularization
- Smooth trajectory generation

#### IK via Nonlinear Programming (IK-NLP)

- Full nonlinear optimization
- Obstacle avoidance constraints
- End-effector trajectory shaping

Folder:
manipulation/


---

### 3. Optimization

**Direct Collocation**

- Discretized nonlinear dynamics
- Equality constraints for system evolution
- Nonlinear programming formulation
- Numerical optimal control solution

Folder:
optimization/direct_collocation


---

### 4. Multibody Simulation

**Self-Balancing Robot**

- Dynamic model simulation (ODE45)
- Simscape Multibody implementation
- PID stabilization

Folder:
simulation/self_balancing


---

## Technical Topics Covered

- Hybrid dynamical systems
- Reduced-order locomotion models
- Constrained optimization
- Quadratic programming
- Nonlinear programming
- Trajectory optimization
- Multibody simulation

---

## Notes

All projects are implemented for simulation and educational research purposes.