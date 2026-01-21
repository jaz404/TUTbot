# Sensor Fusion
## 1. Adding noise to robot motion
### Why add noise?
Real world sensors and actuators are never perfect. Adding noise to the robot's motion models helps to simulate real world conditions and test the robustness of the robot's control algorithms. 

We will add errors in 4 places:
1. (2) left and right odom calculations - gaussian noise
2. Wheel radius - error in measurement/wear
3. Wheel separation - error in measurement

### Implementing [noisy_controller.py](../src/bumperbot_controller/bumperbot_controller/noisy_controller.py) 

We do want to re-implement the code for conversion of the wheel commands to velocity commands of wheels this will still be done by the simple_controller. In the noisy controller, we only want to introduce noise and errors in the odometry calculation and so in the robot localization. So, we will be required to run both. For true values, we will use the simple_controller and for noisy values, we will use the noisy_controller. 
This means both controllers can run simultaneously:
- **`simple_controller`** → `/bumperbot_controller/odom` (clean odometry)
- **`noisy_controller`** → `/bumperbot_controller/odom_noisy` (noisy odometry)

#### 1. Remove the additional logic:
##### We no longer need to publish wheel commands or subscribe to velocity commands. The following lines can be deleted. 
```python
self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
self.vel_sub_ = self.create_subscription(TwistStamped, "bumperbot_controller/cmd_vel", self.vel_callback, 10)

self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2], 
[self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
```
#### 2. Update the name of the odom topic where we will be publishing noisy odometry messages. 
```python
self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom_noisy", 10)
```
#### 3. Change the name of the child frame:
```python
self.odom_msg_ = Odometry()
self.odom_msg_.header.frame_id = "odom"
self.odom_msg_.child_frame_id = "base_footprint_ekf"
```
#### 4. Change the transform frame:
```python
self.tf_msg_.child_frame_id = "base_footprint_noisy"
```
#### 5. Add gaussian noise to the readings:
```python
# Add random gaussian noise to the wheel encoders
wheel_encoder_left += np.random.normal(0, 0.005) # 0 mean and 0.005 std dev
wheel_encoder_right += np.random.normal(0, 0.005)
```
#### 6. Adding noisy controller to the [controller.launch.py](../src/bumperbot_controller/launch/controller.launch.py)
So far we have added noise to the wheel encoders, but we also need to consider the error in measurements: wheel radius and wheel separation. Also, the wheel radius might change with wear and tear over time. Since we have defined these parameters in the [noisy_controller.py](../src/bumperbot_controller/bumperbot_controller/noisy_controller.py) file, we need to modify the [controller.launch.py](../src/bumperbot_controller/launch/controller.launch.py) file to pass these parameters with noise to the noisy controller.

```python
wheel_radius_error_arg = DeclareLaunchArgument(
    "wheel_radius_error",
    default_value="0.005",
)
wheel_separation_error_arg = DeclareLaunchArgument(
    "wheel_separation_error",
    default_value="0.02",
)
```
To be able to use these real time values when the launch file is used, we need to use the Opaque Function from the launch module. An OpaqueFunction in ROS2 launch is a way to pass complex data or functions between launch files without exposing implementation details.
```python
noisy_controller_launch = OpaqueFunction(function=noisy_controller)
```
In the `noisy_controller(context, *args, **kwargs)` function, we can access the values of the arguments using `LaunchConfiguration` and perform operations on them. We will get the values of wheel_radius and wheel_radius error and add them and pass it to the noisy controller node. Same with the wheel separation. 

So, when our launch file is launched, it always launches the noisy controller as well. 

### Visualizing the difference between clean and noisy odometry

#### 1. Launch gazebo and simple controller using the launch files
```bash
ros2 launch bumperbot_description gazebo.launch.py
ros2 launch bumperbot_controller controller.launch.py
```
Should see all these being published on listing all topics:
```bash
/bumperbot_controller/cmd_vel
/bumperbot_controller/odom
/bumperbot_controller/odom_noisy
```
#### 2. Install PlotJuggler
We will be using this tool to visualize the odometry data.
```bash
sudo apt install ros-jazzy-plotjuggler*
```
Run plot juggler using the command
```bash
ros2 run plotjuggler plotjuggler
```
<p align="center">
<img src="assets/noisy_odom_1.gif" />
<br>
Noisy vs Clean odometry visualization 
</p>

1. Click on Start in the streaming section
2. Select topics to be visualized 
3. Drag and drop the sub message from the topics list to the plot area

> On the left window, plotjuggler is plotting the linear.x for both clean and noisy odometry and on the right, rviz is showing optimal odometry(odom) and base_footprint_noisy transforms. The  base_footprint_noisy can be seen wavering due to the gaussian noise we added. 

### Using joy_teleop to control the robot to visualize the difference better
Along with Gazebo, simple_controller, RViz and plotjuggler, launch the joy_teleop launch file.
```bash
ros2 launch bumperbot_controller joystick_teleop.launch.py
```
Now we can visualize the difference between clean and noisy data better. The `base_footprint` is the clean odometry and the `base_footprint_noisy` is the noisy odometry. 

    <p align="center">
    <img src="assets/noisy_odom_2.gif" />
    <br>
    Noisy vs Clean odometry visualization 
    </p>

## 2. Implementing Kalman Filter (monocular)

### What is a kalman filter?
The Kalman Filter is a recursive state estimation algorithm that combines a system’s motion model with noisy sensor measurements to produce an optimal estimate of the system state and its uncertainty. At each time step, it predicts the next state using the model and then updates this prediction using measurements, weighting each by their respective uncertainties. It is optimal for linear systems with Gaussian noise and is widely used in robotics, control, and sensor fusion. It is a subset of bayesian filter and the simplest bayesian filter. 

### Create package `bumperbot_localization` and create [kalman_filter.py](../src/bumperbot_localization/bumperbot_localization/kalman_filter.py)

In this example, we will be creating a monocular kalman filter which only deals with the imu_angular_z_.

<p align="center">
<img src="assets/odom_kalman.gif" />
<br>
Kalman filter odom
</p>

#### Subscribe to odom(noisy) and imu topics and publish to odom_kalman topic
```python
self.odom_sub_ = self.create_subscription(Odometry, "bumperbot_controller/odom_noisy", self.odomCallback, 10)
self.imu_sub_ = self.create_subscription(Imu, "imu/out", self.imuCallback, 10)
self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom_kalman", 10)
```

#### Initialize mean and variance and first measurement detection
##### distinguishing first measurement is necessary for the setup since we do not know the mean and variance at the start. 
```python
self.mean_ = 0.0
self.variance_ = 1000.0
self.motion_variance_ = 4.0
self.measurement_variance_ = 0.5
self.imu_angular_z_ = 0.0
self.is_first_odom_ = True
self.last_angular_z_ = 0.0
self.motion_ = 0.0
```
#### odom_callback
##### For first message, setup the mean and last angular z values and set flag to false
```python
self.kalman_odom_ = odom
if self.is_first_odom_:
    self.last_angular_z_ = odom.twist.twist.angular.z
    self.is_first_odom_ = False
    self.mean_ = odom.twist.twist.angular.z
    return
```
##### For any following message, calculate the motion, run the state prediction and measurement update functions, update the odom_kalman and publish it.
```python
self.motion_ = odom.twist.twist.angular.z - self.last_angular_z_
self.statePrediction()
self.measurementUpdate()
# Update for the next iteration
self.last_angular_z_ = odom.twist.twist.angular.z
# Update and publish the filtered odom message
self.kalman_odom_.twist.twist.angular.z = self.mean_
self.odom_pub_.publish(self.kalman_odom_)
```
#### define the state prediction and measurment update functions
```python
def measurementUpdate(self):
    self.mean_ = (self.measurement_variance_ * self.mean_ + self.variance_ * self.imu_angular_z_) / (self.variance_ + self.measurement_variance_)           
    self.variance_ = (self.variance_ * self.measurement_variance_) / (self.variance_ + self.measurement_variance_)
def statePrediction(self):
    self.mean_ = self.mean_ + self.motion_
    self.variance_ = self.variance_ + self.motion_variance_
```
#### Running the kalman_filter node
Run gazebo, controller, joystick, plotjuggler, kalman_filter node
```bash
ros2 run bumperbot_localization kalman_filter.py
```
## 3. EKF - Extended Kalman Filter