# Kinematics

Mobile robot kinematics describes how robots move through space. For a differential drive robot moving on a 2D plane, we can describe its pose (position and orientation) with respect to the global/world frame using 3 parameters: x, y, and θ (theta).

- **x, y**: Cartesian coordinates representing the robot's position in the world frame
- **θ (theta)**: Orientation angle representing the robot's rotation about the z-axis

## Turtlesim Example

Launch the turtlesim node:
```bash
ros2 run turtlesim turtlesim_node
```
In another window, launch turtle teleoperation:
```bash
ros2 run turtlesim turtle_teleop_key
```
<p align="center">
  <img src="assets/turtlesim_teleop.gif" alt="Turtle Teleop">
  <br>
  <em>Turtle sim teleop.</em>
</p>

Rqt graph:
```bash
ros2 run rqt_graph rqt_graph
```

Run ros2 topic list:
```bash
ros2 topic list
```
Output:
```
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```
<p align="center">
  <img src="assets/rqt_turtlesim.png" alt="Rqt Graph">
  <br>
  <em>Rqt graph of turtlesim node and turtle_teleop_key node.</em>
</p>

> Here, nodes are ovals, topics are rectangles and edges indicate publish/subscribe relationships.

In this rqt_graph view, the running nodes are `/teleop_turtle` and `/turtlesim`, with communication occurring through several topics under the `/turtle1` namespace. The `/teleop_turtle` node publishes velocity commands on the `/turtle1/cmd_vel` topic, which is subscribed to by the `/turtlesim` node to move the turtle in the simulator. The `/turtlesim` node also provides the `/turtle1/rotate_absolute` action server, which expands into multiple topics: it publishes feedback on `/turtle1/rotate_absolute/_action/feedback` and status updates on `/turtle1/rotate_absolute/_action/status`, while an action client (implicit in the system) subscribes to these to monitor goal execution.

Since, we now know the velocity commands are published on the `/turtle1/cmd_vel` topic, we can now publish velocity commands to this topic to move the turtle in the simulator.

```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist 'linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
'
```

> Here, we are publishing a Twist message to the `/turtle1/cmd_vel` topic. The Twist message has two fields: linear and angular. The linear field has x, y, and z components, and the angular field has x, y, and z components.

We can also see the pose of the `/turtle1` by running the following command:
```bash
ros2 topic echo /turtle1/pose
```
Output:
```bash
---
x: 6.490523815155029
y: 3.875563144683838
theta: 0.2210351973772049
linear_velocity: 1.0
angular_velocity: 1.0
---
```
We can also spawn another turtle in the sim by:

```bash
ros2 service call /spawn turtlesim/srv/Spawn "x: 0.0
y: 5.0
theta: 20.0
name: 'turtle2'"
```
```bash
waiting for service to become available...
requester: making request: turtlesim.srv.Spawn_Request(x=0.0, y=5.0, theta=20.0, name='turtle2')

response:
turtlesim.srv.Spawn_Response(name='turtle2')
```
Now, both the turtles have separate topics being published and subscribed to. 
```bash
ros2 topic list

/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
/turtle2/cmd_vel
/turtle2/color_sensor
/turtle2/pose
```
To now control turtle2 using teleop:
```bash
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```
### Translation Vector and Rotation Matrix 

The pose topics `turtle1/pose` and `turtle2/pose` provide the position and orientation of each turtle with respect to the global frame (the bottom left corner of the turtlesim window).

#### Translation Vector
The translation vector from turtle1 to turtle2 represents the position of turtle2 expressed in turtle1's coordinate frame:

$$\mathbf{t}_{2}^{1} = \begin{bmatrix} T_x \\ T_y \end{bmatrix} = \begin{bmatrix} x_2 - x_1 \\ y_2 - y_1 \end{bmatrix}$$

#### Rotation Matrix
The rotation matrix defines the orientation of turtle2 with respect to turtle1:

$$
R_{2}^{1} = \begin{bmatrix} 
\cos\theta_{2}^{1} & -\sin\theta_{2}^{1} \\ 
\sin\theta_{2}^{1} & \cos\theta_{2}^{1} 
\end{bmatrix}
$$

where $\theta_{2}^{1} = \theta_2 - \theta_1$ is the relative orientation.

#### Homogeneous Transformation
The complete transformation from turtle1 to turtle2 combines both rotation and translation into a single homogeneous transformation matrix:

$$
T_{2}^{1} = \begin{bmatrix} 
R_{2}^{1} & \mathbf{t}_{2}^{1} \\ 
0_{1\times2} & 1
\end{bmatrix} = \begin{bmatrix} 
\cos\theta_{2}^{1} & -\sin\theta_{2}^{1} & T_x \\ 
\sin\theta_{2}^{1} & \cos\theta_{2}^{1} & T_y \\ 
0 & 0 & 1
\end{bmatrix}
$$

#### Implementation
A practical implementation is provided in [simple_turtlesim_kinematics.py](../src/bumperbot_py_examples/bumperbot_py_examples/simple_turtlesim_kinematics.py). To use it:

1. Spawn two turtles using the methods described above
2. Run the kinematics node:
```bash
ros2 run bumperbot_py_examples simple_turtlesim_kinematics
```

<p align="center">
  <img src="assets/t1_2.png" alt="Turtle Kinematics">
  <br>
  <em>Translation vector and rotation matrix from turtle1 to turtle2 </em>
</p>

# Differential Kinematics 

### Robot Pose
The robot pose in the world frame is

$$
\mathbf{p} =
\begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix}
$$

with time derivative

$$
\dot{\mathbf{p}} =
\begin{bmatrix}
\dot{x} \\
\dot{y} \\
\dot{\theta}
\end{bmatrix}
$$

### Wheel Parameters

- $r$ : wheel radius  
- $l$ : distance between left and right wheels (wheelbase)  
- $\dot{\phi}_R$ : right wheel angular velocity  
- $\dot{\phi}_L$ : left wheel angular velocity  

### Compact Function Form

$$
\dot{\mathbf{p}} = f(l, r, \theta, \dot{\phi}_R, \dot{\phi}_L)
$$
This is what we need to calculate to be able to send velocity commands to the robot.

### Body-Frame Velocities

Linear and angular velocities of the robot body:

$$
v = \frac{r}{2}(\dot{\phi}_R + \dot{\phi}_L)
$$

$$
\omega = \frac{r}{l}(\dot{\phi}_R - \dot{\phi}_L)
$$


### Differential Drive Velocity Mapping

The mapping from wheel velocities to body velocities is

$$
\mathbf{u}
=
\begin{bmatrix}
\frac{r}{2} & \frac{r}{2} \\
\frac{r}{l} & -\frac{r}{l}
\end{bmatrix}
\dot{\boldsymbol{\phi}}
$$

or explicitly,

$$
\begin{bmatrix}
v \\
\omega
\end{bmatrix}
=
\begin{bmatrix}
\frac{r}{2} & \frac{r}{2} \\
\frac{r}{l} & -\frac{r}{l}
\end{bmatrix}
\begin{bmatrix}
\dot{\phi}_R \\
\dot{\phi}_L
\end{bmatrix}
$$
On the left side we have the robot body velocities and on the right side we have the wheel velocities. 

This matrix is often referred to as the **kinematic coupling matrix** for a differential-drive robot.

We can also map it the other way around:

$$
\begin{bmatrix}
\dot{\phi}_R \\
\dot{\phi}_L
\end{bmatrix}
=
\begin{bmatrix}
\frac{1}{r} & \frac{l}{2r} \\
\frac{1}{r} & -\frac{l}{2r}
\end{bmatrix}
\begin{bmatrix}
v \\
\omega
\end{bmatrix} \tag{1}
$$

### Body-to-World Velocity Transformation

Connects the velocity of robot in body frame to the velocity of robot in world 

The robot body-frame velocity vector is

$$
\mathbf{u} =
\begin{bmatrix}
v \\
\omega
\end{bmatrix}
$$

The world-frame pose rate is given by

$$
\begin{bmatrix}
\dot{x} \\
\dot{y} \\
\dot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
\cos\theta & -\sin\theta & 0 \\
\sin\theta & \cos\theta  & 0 \\
0          & 0           & 1
\end{bmatrix}
\begin{bmatrix}
v \\
0 \\
\omega
\end{bmatrix} \tag{2}
$$

or equivalently,

$$
\begin{bmatrix}
\dot{x} \\
\dot{y} \\
\dot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
v\cos\theta \\
v\sin\theta \\
\omega
\end{bmatrix}
$$
    The body-frame velocity of a differential-drive robot consists of forward motion \(v\) and angular motion \(\omega\). The lateral velocity component is zero due to the nonholonomic (no side-slip) constraint of the wheels. The rotation matrix transforms these body-frame velocities into world-frame pose rates.

Using equations (1) and (2), we can calculate the equation which connects the overall velocity of the robot in the world frame to the rotation velocity of the wheels. This is called the **forward differential kinematics**.

On multiplying (1) and (2), we get: 

$$
\begin{bmatrix}
\dot{x} \\
\dot{y} \\
\dot{\theta}
\end{bmatrix}
=
\left(
\begin{bmatrix}
\cos\theta & -\sin\theta & 0 \\
\sin\theta & \cos\theta  & 0 \\
0          & 0           & 1
\end{bmatrix}
\right)
\left(
\begin{bmatrix}
\frac{r}{2} & \frac{r}{2} & 0 \\
0           & 0         & 0 \\
\frac{r}{l} & -\frac{r}{l} & 0
\end{bmatrix}
\right)
\left(
\begin{bmatrix}
\dot{\phi}_R \\
\dot{\phi}_L \\
0
\end{bmatrix}
\right)
$$

$$
\mathbf{\dot{\mathbf{p}}} = f(r, l, \theta, \dot{\phi}_R, \dot{\phi}_L)
$$

On simplifying the kinematic expression, we obtain the **Jacobian matrix** that maps wheel angular velocities to world-frame pose rates:

$$
\begin{bmatrix}
\dot{x} \\
\dot{y} \\
\dot{\theta}
\end{bmatrix}
=
\left(
\begin{bmatrix}
\frac{r}{2}\cos\theta & \frac{r}{2}\cos\theta \\
\frac{r}{2}\sin\theta & \frac{r}{2}\sin\theta \\
\frac{r}{l} & -\frac{r}{l}
\end{bmatrix}
\right)
\begin{bmatrix}
\dot{\phi}_R \\
\dot{\phi}_L
\end{bmatrix}
\tag{3}
$$

## Simple Speed Controller [[simple_controller.py]](../src/bumperbot_controller/bumperbot_controller/simple_controller.py)

Now, we have all the pre-requisites to implement the simple speed controller. 

Create a new file called [simple_controller.py](../src/bumperbot_controller/bumperbot_controller/simple_controller.py) in the `bumperbot_controller` package. 

In the class, we can define wheel radius and wheel separation as parameters and declare them using the `declare_parameter` method. 
```python
self.declare_parameter('wheel_radius', 0.033)
self.declare_parameter('wheel_separation', 0.17)
```
We will publish the wheel speeds to the topic `simple_velocity_controller/commands` and subscribe to the topic `bumperbot_controller/cmd_vel`. In the callback to subscription, we will calculate the wheel speeds using the kinematic expression (3) we calculated above and publish them to the topic `simple_velocity_controller/commands`.

```python
self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2],
                                           [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
```
Next, we can modify the [controller.launch.py](../src/bumperbot_controller/launch/controller.launch.py) file to pass the parameters to the controller node. 
```python
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033",
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17",
    )
```
For more details, refer to the [bumpberbot_controller](../src/bumperbot_controller) package.

The `--show-args` flag can be used to see the arguments that can be passed to the launch file. These can be passed as arguments to the launch file.

```bash
ros2 launch bumperbot_controller controller.launch.py --show-args
```
Output: 
```bash
Arguments (pass arguments as '<name>:=<value>'):

    'use_python':
        Use Python for controller manager
        (default: 'true')

    'wheel_radius':
        Wheel radius
        (default: '0.033')

    'wheel_separation':
        Wheel separation
        (default: '0.17')
```

We should see the topic `/bumperbot_controller/cmd_vel` topic after running the controller launch file and their should be correct setup messages displayed in the terminal for gazebo and simple_controller. 

Run gazebo `ros2 launch bumperbot_gazebo bumperbot.launch.py` and controller `ros2 launch bumperbot_controller controller.launch.py`.
GZ terminal should display something for both simple_velocity_controller and joint_state_broadcaster which we defined in the [bumperbot_controllers.yaml](../src/bumperbot_controller/bumperbot_controller/config/bumperbot_controllers.yaml) file. 
```bash
ros_test_ws/install/bumperbot_controller/share/bumperbot_controller/config/bumperbot_controllers.yaml -p use_sim_time:=true --param use_sim_time:=true 
[gazebo-2] [INFO] [1768547892.862081686] [controller_manager]: Configuring controller: 'simple_velocity_controller'
[gazebo-2] [INFO] [1768547892.863846881] [simple_velocity_controller]: configure successful
[gazebo-2] [INFO] [1768547892.947776603] [controller_manager]: Activating controllers: [ simple_velocity_controller ]
[gazebo-2] [INFO] [1768547892.948426164] [simple_velocity_controller]: activate successful
[gazebo-2] [INFO] [1768547892.948521374] [controller_manager]: Successfully switched controllers!
[gazebo-2] [INFO] [1768547893.168249378] [controller_manager]: Loading controller : 'joint_state_broadcaster' of type 'joint_state_broadcaster/JointStateBroadcaster'
[gazebo-2] [INFO] [1768547893.168382290] [controller_manager]: Loading controller 'joint_state_broadcaster'
[gazebo-2] [INFO] [1768547893.180661910] [controller_manager]: Controller 'joint_state_broadcaster' node arguments: --ros-args --params-file /home/jaspr/Documents/TUTbot/ros_test_ws/install/bumperbot_controller/share/bumperbot_controller/config/bumperbot_controllers.yaml -p use_sim_time:=true --param use_sim_time:=true 
[gazebo-2] [INFO] [1768547893.248758345] [controller_manager]: Configuring controller: 'joint_state_broadcaster'
[gazebo-2] [INFO] [1768547893.248936954] [joint_state_broadcaster]: 'joints' or 'interfaces' parameter is empty. All available state interfaces will be published
[gazebo-2] [INFO] [1768547893.273773149] [controller_manager]: Activating controllers: [ joint_state_broadcaster ]
[gazebo-2] [INFO] [1768547893.281656138] [controller_manager]: Successfully switched controllers!
```
Simple controller terminal should display something like this:
```bash
[INFO] [simple_controller.py-3]: process started with pid [73093]
[simple_controller.py-3] [INFO] [1768547892.366700139] [simple_controller]:  Using wheel radius: 0.033
[simple_controller.py-3] [INFO] [1768547892.367461813] [simple_controller]:  Using wheel separation: 0.17
[simple_controller.py-3] [INFO] [1768547892.375316157] [simple_controller]:  The speed conversion matrix is: [[ 0.0165      0.0165    ]
[simple_controller.py-3]  [ 0.19411765 -0.19411765]]
[spawner-2] [INFO] [1768547892.860621409] [spawner_simple_velocity_controller]: Loaded simple_velocity_controller
[spawner-2] [INFO] [1768547892.960000779] [spawner_simple_velocity_controller]: Configured and activated simple_velocity_controller
[INFO] [spawner-2]: process has finished cleanly [pid 73092]
[spawner-1] [INFO] [1768547893.247381777] [spawner_joint_state_broadcaster]: Loaded joint_state_broadcaster
[spawner-1] [INFO] [1768547893.293466639] [spawner_joint_state_broadcaster]: Configured and activated joint_state_broadcaster
[INFO] [spawner-1]: process has finished cleanly [pid 73091]
```

Next, to actually move the robot, we can publish to `/bumperbot_controller/cmd_vel` topic. 
> Sometimes the tab completion doesn't work directly to get the message template. Just paste the yaml in the terminal with a \\. 
```bash
ros2 topic pub /bumperbot_controller/cmd_vel geometry_msgs/msg/TwistStamped \ "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  twist: {linear: {x: 0.0, y: 0.0, z: 0.0},
          angular: {x: 0.0, y: 0.0, z: 0.5}}}"
```

<!-- include gif -->
<p align="center">
<img src="assets/diff_kinematics.gif" />
<br>
<em>Simple controller demo</em>
</p>

### Using turtlesim teleop with simple controller

Turtlesim teleop publishes messages to `turtle1/cmd_vel`. We can remap the topic to `/bumperbot_controller/cmd_vel` to use it with the simple controller. The teleop uses the `geometry_msgs/msg/Twist` format and not `geometry_msgs/msg/TwistStamped`, so we need to modify the simple controller to accept `geometry_msgs/msg/Twist`.

**Changes needed in `simple_controller.py`:**

- **Line 23**: Change `TwistStamped` to `Twist`
  ```python
  self.vel_sub_ = self.create_subscription(Twist, "bumperbot_controller/cmd_vel", self.vel_callback, 10)
  ```

- **Lines 31-32**: Switch `msg.twist` to `msg`
  ```python
  robot_speed = np.array([[msg.linear.x], 
  [msg.angular.z]])
  ```

<p align="center">
  <img src="assets/control_teleop.gif" alt="Turtle Teleop" />
  <br>
  <em>Teleop with simple controller.</em>
</p>

## Using joystick teleop with simple controller

We can use the joystick teleop to control the robot. We can use the `joy_teleop` package to do this. 

<p align="center">
  <img src="assets/teleop_ps4.gif" alt="Joystick Teleop" />
  <br>
  <em>Teleop with joystick.</em>
</p>

Implementation details are in the [joystick_teleop.launch.py](../src/bumperbot_controller/launch/joystick_teleop.launch.py) file. This file launches the joy_node from the `joy` package and the joy_teleop node from the `joy_teleop` package.
```python
joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory("bumperbot_controller"), "config", "joystick_config.yaml")]
    )
joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(get_package_share_directory("bumperbot_controller"), "config", "joystick_teleop.yaml")]
    )
```
#### joy package
The joy package provides a low-level interface between a physical joystick and ROS 2. Its joy_node reads raw input events from a joystick device exposed by the operating system (typically via /dev/input/js* on Linux) and publishes them as messages of type `sensor_msgs/msg/Joy`. These messages contain an array of axis values(analog sticks and triggers) and an array of button states(pressed or not pressed).

The [joystick_config.yaml](../src/bumperbot_controller/config/joystick_config.yaml) file configures parameters for the joy_node. The parameters are:
- `device_id`: The ID of the joystick device. This is typically 0 for the first joystick.
- `device_name`: The name of the joystick device. This is typically empty.
- `deadzone`: The deadzone of the joystick. This is the range of values where the joystick is considered to be at rest.
- `autorepeat_rate`: The autorepeat rate of the joystick. This is the rate at which the joystick is repeated when held down.
- `sticky_buttons`: This is whether to use sticky buttons or not.
- `coalesce_interval_ms`: This is the interval at which the joystick is coalesced.


#### joy_teleop package
The joy_teleop package operates at a higher semantic level. It subscribes to the `sensor_msgs/msg/Joy` messages published by the joy_node and maps joystick inputs to robot commands based on a user-defined configuration. 

The [joystick_teleop.yaml](../src/bumperbot_controller/config/joystick_teleop.yaml) file configures parameters for the joy_teleop node. The parameters are:
- `move`: This is the move configuration. It has the following parameters:
  - `type`: The type of the move configuration. This is typically `topic`.
  - `interface_type`: The interface type of the move configuration. This is typically `geometry_msgs/msg/TwistStamped`.
  - `topic_name`: The topic name of the move configuration. This is typically `/bumperbot_controller/cmd_vel`.
  - `deadman_buttons`: The deadman buttons of the move configuration. If this button is not pressed, the robot will not move as a safety measure. 
  - `axis_mappings`: The axis mappings of the move configuration. This maps the joystick axis to the robot's linear and angular velocity. 

The button and axis mapping can be found at https://github.com/Ar-Ray-code/ps_ros2_common. 