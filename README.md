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
│   └── launch/
│       └── display.launch.py
│
└── model_gazebo/               # Gazebo simulation & control
    ├── launch/
    │   └── model_in_gazebo.launch.py
    ├── config/
    │   └── controllers.yaml
    ├── model_gazebo/
    │   └── vel_cmd_to_wheel.py     # Inverse kinematics node
    └── worlds/
```

---

## Simulation Environment Setup

Getting an omni-wheel robot to drive correctly in Gazebo required substantial integration work. This section documents the effort involved.

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

<!-- 
TODO: Add the full motion model derivation here. Suggested content:

### Coordinate Frames
- World frame vs body frame
- Wheel placement angles (α₁ = 180°, α₂ = -60°, α₃ = 60°)

### Forward Kinematics
Given wheel angular velocities (ω₁, ω₂, ω₃), compute body velocity (vₓ, vᵧ, ω):
- Matrix formulation
- Pseudoinverse solution (over-constrained system)

### State Equations
- State vector: [x, y, θ]ᵀ or [x, y, θ, vₓ, vᵧ, ω]ᵀ
- Discrete-time state transition equations
- Process noise model

### Figures
- Diagram showing wheel orientations and velocity vectors
- Coordinate frame definitions
-->

*Section to be completed*

---

## UKF Odometry Implementation

<!--
TODO: Document the UKF implementation. Suggested content:

### State Vector
- What states are estimated?
- Why UKF over EKF? (non-linear process/measurement models)

### Process Model
- How wheel encoder readings predict the next state
- Process noise covariance Q

### Measurement Model
- IMU measurements used (orientation, angular velocity, linear acceleration)
- Measurement noise covariance R

### Sensor Fusion Approach
- Block diagram of the fusion architecture
- Update rates for each sensor

### Tuning
- How noise parameters were determined
- Any challenges encountered

### Code Structure
- Key ROS nodes/files
- Topic interfaces
-->

*Section to be completed*

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

<!--
TODO: Document any limitations or planned improvements
- Omni-wheel friction approximation limitations in Gazebo
- Wheel slip at high accelerations
- Plans for hardware deployment
-->

*Section to be completed*

---

## Team Members

- Darren Biskup (db3728)
- Han He (hh3102)

---

## References

<!--
TODO: Add references
- Papers on omni-wheel kinematics
- UKF/sensor fusion references
- ROS 2 documentation links
-->

*To be added*

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
