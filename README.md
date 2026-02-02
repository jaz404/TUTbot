# TUTbot

<div align="center">
  <img src="docs/assets/assembled1.jpg" width="400" height="400"/>
  <img src="docs/assets/assembled2.jpg" width="400" height="400"/>
  <br/>
  <img src="docs/assets/ros2_control_test.gif" width="400" height="400"/>
  <img src="docs/assets/joint_pub_crop.gif" width="400" height="400"/>
  <br/>
  <em>TUTbot</em>
</div>

*TUTbot* or short for tutorial robot is a mobile robot that I am building to advance my understanding of robotic teleoperation, autonomous navigation, perception and simulation.

This repository contains the documentation, code, and resources for building and programming the TUTbot robot.


<div align="center">

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![Python](https://img.shields.io/badge/Python-3.10+-green)
![Raspberry Pi](https://img.shields.io/badge/Raspberry%20Pi-5-blue)

</div>

## Index
- [Assembly Guide](docs/assembly.md)
- [Visualization Setup](docs/setup_vis.md)
- [ROS2 Control Setup](docs/ros2_control.md)
- [Kinematics](docs/kinematics.md)
- [Odometry](docs/odometry.md)
- [Sensor Fusion](docs/sensor_fusion.md)
- [Mapping and Localization](docs/mapping_and_localization.md)
- [Arduino to ROS](docs/arduino_to_ros.md)

## Progress Table

| Task | Status |
|------|--------|
| Robot Assembly and Setup | Done |
| ROS setup | Done  |
| Setup PID control for motors | In Progress |
| Teleoperation | Done |
| URDF & Visualization | Done |
| Control | Done |
| Kinematics | Done |
| Odometry | Done |
| Probability & Sensor Fusion | Done |
| Mapping and Localization | In Progress |
| Navigation | In Progress |

## Some Visuals

### Noisy vs Clean odometry
<p align="center">
<img src="docs/assets/noisy_odom_2.gif" />
<br>
Noisy vs Clean odometry
</p>

### Monocular Kalman Filter
<p align="center">
<img src="docs/assets/odom_kalman.gif" />
<br>
Monocular Kalman Filter
</p>

### EKF based localization
<p align="center">
<img src="docs/assets/ekf.gif" />
<br>
EKF
</p>

### Teleoperation
<p align="center">
<img src="docs/assets/teleop_ps4.gif" />
<br>
Teleoperation with Joystick
</p>

<p align="center">
<img src="docs/assets/control_teleop.gif" />
<br>
Teleoperation with Turtlesim
</p>

### Trajectory mapper node
<p align="center">
<img src="docs/assets/odometry_trajectory.png" width="800"/>
<br>
Trajectory mapper node
</p>


## Acknowledgements
This project was created with help from Antonio Brandi's course on ROS2. 

## License
This project is licensed under the MIT License - see the LICENSE file for details.