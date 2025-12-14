# Final Project: Omni-Wheel Robot Odometry with UKF Sensor Fusion

A ROS 2 project implementing odometry for a three-wheeled omni-directional robot (equilateral triangle configuration) by fusing wheel encoder data with IMU measurements using an Unscented Kalman Filter.

## Overview

This is a final project for EEME6911 Topics in Control:Probabilistic Robotics. Our primary contributions are:

1. **Deriving a motion model** for the omni-wheel drivetrain (three wheels in an equilateral triangle configuration)
2. **Implementing UKF-based odometry** by fusing this motion model with IMU sensor data

The project includes a complete simulation environment built on ROS 2 Humble and Gazebo, with the robot model, control interfaces, and kinematics nodes.

---

## Table of Contents

- [Robot Platform](#robot-platform)
- [Repository Structure](#repository-structure)
- [Simulation Environment Setup](#simulation-environment-setup)
- [Motion Model Derivation](#motion-model-derivation)
- [UKF Odometry Implementation](#ukf-odometry-implementation)
- [Results](#results)
- [Installation & Usage](#installation--usage)
- [Team Members](#team-members)
- [References](#references)

---

## Robot Platform
The LeKiwi robot uses three omni-directional wheels arranged in an equilateral triangle, with the URDF adapted from the original [SIGRobotics-UIUC/LeKiwi](https://github.com/SIGRobotics-UIUC/LeKiwi) repository.

**insert diagram of lekiwi top down**

- **Wheel 1 (back):**
- **Wheel 2 (front right):**
- **Wheel 3 (front left):**

Each wheel is driven by an ST3215 servo motor.

---

## Repository Structure

```
src/
├── model_description/          # Robot URDF/meshes
│   ├── urdf/
│   │   └── LeKiwi_simplified.urdf
│   ├── meshes/
│
└── model_gazebo/               # Gazebo simulation & control
    ├── launch/
    │   └── model_in_gazebo.launch.py
    ├── config/
    │   └── controllers.yaml
    ├── model_gazebo/
    │   └── vel_cmd_to_wheel.py     # Inverse kinematics and low level controller node
    |   └── ukf_odometry.py    # ukf odometry implementation
    |   └── ground_truth_pub.py     
    └── worlds/
```

---

## Simulation Environment Setup

Getting an omni-wheel robot to simulate correctly in Gazebo required substantial integration work. This section documents the effort involved.

### URDF Configuration

The robot model was converted from CAD to URDF with the following considerations:

- **Inertial properties:** We ran a script to prune collision elements and inertias from the urdf tree so that the robot had a neutral center of mass and inertia tensors for stable simulation
- **ros2_control integration:** Added `<ros2_control>` tags to define the hardware interface for Gazebo
- **Collision geometry:** Simplified collision meshes for performance. Wheels themselves treated as cylinders 
- **Omni-wheel friction:** Configured asymmetric friction (low lateral, normal longitudinal) to approximate omni-wheel behavior in Gazebo

### ros2_control Integration

We used `ros2_control` with `gazebo_ros2_control` to interface with the simulated robot:

| Component | Purpose |
|-----------|---------|
| `JointStateBroadcaster` | Publishes joint positions/velocities to `/joint_states` |
| `JointGroupVelocityController` | Accepts velocity commands for all three wheels |

**Controller configuration** (`controllers.yaml`):
```yaml
omni_wheel_controller:
  ros__parameters:
    joints:
      - ST3215_Servo_Motor-v1-2_Revolute-60   # Wheel 1
      - ST3215_Servo_Motor-v1-1_Revolute-62   # Wheel 2
      - ST3215_Servo_Motor-v1_Revolute-64     # Wheel 3

gazebo_ros2_control:
  ros__parameters:
    gains:
      ST3215_Servo_Motor-v1-2_Revolute-60: {p: 1.0, i: 0.0, d: 0.5}
      ST3215_Servo_Motor-v1-1_Revolute-62: {p: 1.0, i: 0.0, d: 0.5}
      ST3215_Servo_Motor-v1_Revolute-64: {p: 1.0, i: 0.0, d: 0.5}
```
this ensures that published wheel velocities are not instantaneously achieved (more physical)

### Inverse Kinematics Low-level control Node

The `vel_cmd_to_wheel` node subscribes to `/cmd_vel` (body-frame velocity) and publishes individual wheel angular velocities to the controller.

**Inverse kinematics equations (also see [Motion Model](#motion-model-derivation)):**

$$
\begin{aligned}
\omega_1 &= \frac{-v_x - \omega L}{r} \\
\omega_2 &= \frac{\cos(60^\circ)v_x - \cos(30^\circ)v_y - \omega L}{r} \\
\omega_3 &= \frac{\cos(60^\circ)v_x + \cos(30^\circ)v_y - \omega L}{r}
\end{aligned}
$$

Where:
- $v_x, v_y$: Linear velocity components in the body frame
- $\omega$: Angular velocity (yaw rate)
- $L$: Distance from wheel to robot center ($0.132$ m)
- $r$: Wheel radius ($0.1$ m)

**ROS 2 topics:**
- Subscribes: `/cmd_vel` (`geometry_msgs/Twist`)
- Publishes: `/omni_wheel_controller/commands` (`std_msgs/Float64MultiArray`)

<!-- 
### Screenshots / Videos
TODO: Add simulation screenshots or video links here
- Robot spawning in Gazebo
- Driving with teleop
- Verifying motion in different directions
-->

---

## Motion Model Derivation

For our robot in Gazebo, the environment is perfectly flat and we assume that the only degrees of freedom of interest are planar translation and rotation about the vertical axis.

<div align="center">
<img src="diagram.png" alt="robot schematic" width="50%">
</div>

### 1. State Vector ($\mathbf{x}$) and Input Vector ($\mathbf{u}$)
The state vector ($6 \times 1$) tracks global pose and body-frame velocities. The control input comes strictly from the **IMU**.
(Note: $a_x, a_y$ are proper accelerations in the Body Frame).

$$
\mathbf{x} = \begin{bmatrix}
p_x \\
p_y \\
\theta \\
v_x \\
v_y \\
\omega
\end{bmatrix}
\quad\quad, \quad \quad
\mathbf{u} = \begin{bmatrix}
a_x \\
a_y \\
\omega_{gyro}
\end{bmatrix}
$$

- $p_x$: Global X Position (m)  
- $p_y$: Global Y Position (m)  
- $\theta$: Global Yaw (rad)  
- $v_x$: Body-frame Velocity X (m/s)  
- $v_y$: Body-frame Velocity Y (m/s)  
- $\omega$: Angular Velocity (rad/s)

$\theta$ is defined to be counterclockwise-positive from the +x axis.

### 2. Motion Model ($f$)
This is the system dynamics function $\mathbf{x}_{k+1} = f(\mathbf{x}_k, \mathbf{u}_k)$.
We integrate the accelerometer inputs to update linear velocity, and we use the gyro input to drive the angular velocity state directly.

$$
\mathbf{x}_{k+1} = \begin{bmatrix}
p_{x, k} + (v_{x, k} \cos \theta_k - v_{y, k} \sin \theta_k) \Delta t \\
p_{y, k} + (v_{x, k} \sin \theta_k + v_{y, k} \cos \theta_k) \Delta t \\
\theta_k + \left( \frac{\omega_k + \omega_{gyro}}{2} \right) \Delta t \\
v_{x, k} + a_x \Delta t \\
v_{y, k} + a_y \Delta t \\
\omega_{gyro}
\end{bmatrix}
$$

### 3. Measurement model ($z$)
The measurement comes strictly from the **Wheel Encoders**.
(Note: $r$ is each wheel radius)

$$
\mathbf{z} = \begin{bmatrix}
\omega_1 \\
\omega_2 \\
\omega_3
\end{bmatrix}
$$

The function $\hat{\mathbf{z}} = h(\mathbf{x})$ maps the current predicted state to the expected sensor readings using the Forward Kinematics matrix $J$.

$$
h(\mathbf{x}) = \frac{1}{r} J \cdot
\begin{bmatrix}
p_x \\
p_y \\
\theta \\
v_x \\
v_y \\
\omega
\end{bmatrix}
$$

$$
h(\mathbf{x}) = \frac{1}{r}\begin{bmatrix}
0 & 0 & 0 & -1 & 0 & -L_0 \\
0 & 0 & 0 & \sin{\frac{\pi}{6}} & -\cos{\frac{\pi}{6}} & -L_0 \\
0 & 0 & 0 & \sin{\frac{\pi}{6}} & \cos{\frac{\pi}{6}} & -L_0
\end{bmatrix}
\begin{bmatrix}
p_x \\
p_y \\
\theta \\
v_x \\
v_y \\
\omega
\end{bmatrix}
$$

$$
h(\mathbf{x}) = \frac{1}{r} \begin{bmatrix}
-v_x + L_0 \omega \\
0.5 v_x - \frac{\sqrt{3}}{2} v_y - L_0 \omega \\
0.5 v_x + \frac{\sqrt{3}}{2} v_y - L_0 \omega
\end{bmatrix}
$$

The measurement model is actually linear w.r.t the state. This means we won't need to sample from it to build a gaussian for the UKF, we can directly combine it with the prediction and measurement uncertainties (in measurement space) to build the Kalman Gain.

---

## UKF Odometry Implementation

Implements a hybrid **Unscented Kalman Filter (Prediction)** and **Linear Kalman Filter (Correction)** for 3-wheel omnidirectional state estimation ($x \in \mathbb{R}^6$: Pose + Twist).

* **Prediction (IMU):** Propagates nonlinear dynamics $x_{k+1} = g(x_k, u_{imu}, \Delta t)$ using the Unscented Transform to handle process noise.
* **Correction (Encoders):** Performs standard linear updates via observation model $z = C\mathbf{x}$ using wheel velocities from `/joint_states`.
* **Interfaces:** Subscribes to `/imu`, `/joint_states`; publishes `/ukf_odometry` and broadcasts `odom` $\to$ `base_link` TF.


---

## Results

<!--
TODO: Add experimental results. Suggested content:

### Test Scenarios
- Straight line motion
- Circular motion
- Figure-8 pattern

### Comparison
- Wheel-only odometry vs fused odometry
- Ground truth comparison (if available)

### Plots
- Position error over time
- Orientation error over time
- Covariance evolution

### Videos
- Embedded videos or links to demonstrations
-->

*Section to be completed*

---

## Installation & Usage

### Dependencies

- ROS 2 Humble
- Gazebo Classic
- `gazebo_ros_pkgs`
- `ros2_control`
- `gazebo_ros2_control`
- `turtlebot3_gazebo` (world file integration)

*these should be included in the installation script from class*

### Building

```bash
cd /path/to/Prob_Project_ws
colcon build
source install/setup.bash
```

### Running the Simulation

Launch Gazebo with the robot:
```bash
ros2 launch model_gazebo model_in_gazebo.launch.py
```

---

## Known Issues / Future Work

This project has shown that the LeKiwi omniwheel robot can be effectively simulated in Gazebo, and Kalman filter-based odometry can be implemented to will improve the accuracy of raw encoder-based odometry. However, odometry alone accumulates substantial error, and in particular odometry on the omniwheel robot exhibits noticeably more drift when compared to a standard differential drive rover like the Turtlebot. To correct this drift one would need to incorporate information from exteroceptive sensors.   

For future work one could consider utilizing the camera to recognize landmarks and perform localization, or return the arm joints to `revolute` and attempt manipulation tasks.

---

## Team Members

- Darren Biskup (db3728)
- Han He (hh3102)

---

## References

* Lynch, K. M., & Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and Control* (Chapter 13: Wheeled Mobile Robots). Cambridge University Press.
* Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.
* Hu, S., Chen, H., & Shao, Y. (2020). Triangular Omnidirectional Wheel Motion Control System. *Open Access Library Journal*, 7: e6677. [https://doi.org/10.4236/oalib.1106677](https://www.scirp.org/journal/paperinformation?paperid=102349)

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
