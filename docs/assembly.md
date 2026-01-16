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
| Chassis plates (base, middle and top) | 1x each | 3D printed structural frame |
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
</div>

> the STL files for the chassis were sourced from AntoBrandi 
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