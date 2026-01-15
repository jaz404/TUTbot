# Kinematics

We can describe the pose of our mobile robot with respect to the global/world frame using 3 parameters: x, y, and theta. 

## Turtlesim Example

Launch the turtlesim node:
```bash
ros2 run turtlesim turtlesim_node
```
In another window, launch turtle teleoperation:
```bash
ros2 run turtlesim turtle_teleop_key
```
<!-- insert a gif here -->
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
