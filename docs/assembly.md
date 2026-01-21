# TUTbot Assembly Guide
This is a walkthrough on how I designed and assembled the TUTbot robot.
<div align="center">
<img src="assets/assembled1.jpg" width="500px" />
<img src="assets/assembled2.jpg" width="500px" />
</div>

## Overview
The TUTbot is built using off-the-shelf components and 3D printed parts. The idea is to keep it simple and modular for easy upgrades. 

## Components List

| Component | Quantity | Notes |
|-----------|----------|-------|
| Chassis plates (base, middle and top) | 1x each | 3D printed structural frame (download [STL*](../src/bumperbot_description/meshes/))|
| Geared Motor with Quadrature Encoder and Mounting Bracket | 2x | Main drive motors |
| Wheels | 2x | Compatible with geared motors |
| Raspberry pi 5 (with active cooling and case) | 1x | Main compute unit |
| rplidar C1 | 1x | 360° 2D LiDAR sensor |
| MPU6050 IMU | 1x | 6-axis IMU sensor |
| LIPO 3S Battery pack (7.4V) | 1x | Power source (using 5V 10A PSU + power bank instead) |
| Wemos d1 mini | 1x | Motor control and encoder reading |
| L298N motor driver | 1x | Motor controller |
| 1.25" castor wheel | 1x | Front support wheel |
| Prototyping components | - | Perfboard, breadboard, terminal block, jumper wires |
| Standoffs M3 | Various lengths | For component mounting |
| Screws and nuts (M2, M3, M4) | Various lengths | Fasteners |
| Wire | - | 26AWG (low power) and 18AWG (high power) |

<div align="center">
<img src="assets/components_rotated.jpg" width="500px"/>
<br>
All the components required for the assembly
</div>

*the STL files for the chassis were sourced from AntoBrandi 
## Assembly Process

### Base Assembly

<div align="center">
<img src="assets/base_1.jpg" width="300px" height="300px"/>
<img src="assets/base_2.jpg"height="300px"/>
</div>

The motor came with mounting brackets, wheel and encoder cable. These need to be attached to the base plate using some M2 screws. I also mounted the 1.25" castor and to level the wheels with the castors, I used 6mm M3 standoffs. 

<div align="center">
<img src="assets/encoder_motor_kit.jpg" height="300px"/>
<img src="assets/encoder_wiring.png"  height="300px"/>
</div>
These motors use a quadrature encoder which is used to determine the angular position and direction of a rotating shaft by generating two square wave signals that are 90 degrees out of phase. I am using the Wemos D1 Mini to read these signals and control the motors via the L298N motor driver. I created a custom PCB to interface between the Wemos D1 Mini and the L298N motor driver (see the images below). Any microcontroller can be used here. I am using Wemos D1 Mini since I already have a few spares. Also make sure to use boot-safe pins with encoder connections, otherwise it may cause unexpected behavior during boot.

<div align="center">
<img src="assets/wemos_1.jpg" width="300px" />
<img src="assets/wemos_2.jpg" width="300px" />
<img src="assets/wemos_connections.jpg" width="300px" />
</div>

### Middle Assembly

<div align="center">
<img src="assets/middle_1.jpg" width="500px" />
</div>

The middle plate holds the Raspberry Pi, L298N motor driver. The raspberry pi is mounted using M3 standoffs and screws. The L298N motor driver is mounted using M3 screws. I also used a short right-angle micro USB cable to connect the Raspberry Pi to the Wemos D1 Mini.

<div align="center">
<img src="assets/micro_b.png" width="500px"/>
</div>

### Top Assembly

The top plate holds rplidar C1 and MPU6050 IMU. The rplidar C1 is mounted using M2.5 screws. The MPU6050 IMU is mounted using M3 screws. 
> additional care must be given to avoid damaging the rather closely located capacitor on MPU6050
<div align="center">
<img src="assets/mpu6050.jpg" height="300px" />
<img src="assets/assembled1.jpg" width="300px" />
</div>

# RPLIDAR C1 Setup Guide

This guide will walk you through setting up the RPLIDAR C1 sensor for the TUTbot robot.

RPLIDAR C1 uses Time-of-Flight (ToF) and triangulation for distance measurement.
<div align="center">
<img src="assets/rplidar_c1.png" width="300px"/>
</div>

For installation and usage instructions, refer to the official repository:
[https://github.com/Slamtec/sllidar_ros2](https://github.com/Slamtec/sllidar_ros2)

## Features:
- 12m ranging radius
- 0.05m blind zone 
- 10Hz (600RPM) scanning frequency
- 0.72° angular resolution
- 5000 sampling rate
- UART communication (460800 baud)
- 30mm ranging accuracy
- 230mA power consumption

## Testing

To verify the RPLIDAR is working correctly:
1. Connect the RPLIDAR to UART to type-C converter
2. Connect the converter to the robot's USB port

    The lidar should be detected at `/dev/ttyUSB0`
    Give additional permissions: `sudo chmod 777 /dev/ttyUSB0`

3. Run `ros2 launch sllidar_ros2 view_sllidar_c1_launch.py` to start the RPLIDAR driver

### Rviz Display
Once the driver is running, open Rviz to visualize the LIDAR data

<img src="assets/ros2_node_test.png" width="1000px"/>
<img src="assets/rplidar_test.jpg" width="1000px"/>

I created a 30x30x10" enclosure to perform basic tests. The Lidar measured distances accurately within the expected range (~74cm).

# Wemos D1 Mini Setup

<div align="center">
<img src="assets/encoder_test.gif" width="300px"/>
</div>