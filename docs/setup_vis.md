# URDF Setup

## Why create a URDF?
It is useful to have a code-independent, human-readable way to describe the geometry of the robot. One of the ways to achieve this is through URDF. The Unified Robot Description Format (URDF) is an XML-based format that describes a robot's structure, links, joints, and visual properties.

This allows us to define the robot's physical components in a standardized way that can be used by various ROS tools and simulation environments like RViz and Gazebo.

```bash
sudo apt-get install ros-jazzy-urdf-tutorial
```
Make sure to source your workspace: `. install/setup.bash`
```bash
ros2 launch urdf_tutorial display.launch.py model:=/home/jaspr/Documents/TUTbot/ros_test_ws/src/bumperbot_description/urdf/bumperbot.urdf.xacro
```
<p align="center">
  <img src="assets/urdf_vis_1.png" alt="URDF Visualization" width="400">
  <br>
  <em>URDF visualization of the bumperbot base footprint and base link. The base_footprint is the origin frame, and the base_link represents the main body of the robot.</em>
</p>

Now let's add the wheel links to our URDF:

We'll add two wheel links to our URDF - one for the left wheel and one for the right wheel. Each wheel will be defined as a link with a visual geometry, and we'll connect them to the base link using joints.

URDF allows defining two origins one at the link level and one at the joint level. The link origin defines the pose of the link's origin relative to the link frame, while the joint origin defines the transformation from the parent link's origin to the child link's origin.

This means we can control where the visual mesh of a link is positioned relative to the link frame, and we can also control where the joint connects between two links. Here is a visual example if we set roll to 90 degrees to rotate the wheel mesh in (i) the link frame, and (ii) the joint frame. Set the joint type to continuous for rotational joints.
<p align="center">
  <img src="assets/link_origin_rotation.png" alt="Link Origin Rotation" width="300">
  <img src="assets/joint_origin_rotation.png" alt="Joint Origin Rotation" width="300">
  <br>
  <em>Left: Link origin rotation. Right: Joint origin rotation.</em>
</p>

Similarly, we can add the caster wheels in the same way, using fixed joints to attach them to the base link.

The complete URDF should now include all the links and joints we've defined, allowing us to visualize the entire robot in RViz.
Using the joint state publisher GUI, we can control the joint angles of the robot and see the changes in real-time in RViz.

<p align="center">
  <img src="assets/joint_pub.gif" alt="Joint State Publisher GUI" width="400">
  <br>
  <em>Joint state publisher GUI for controlling robot joints.</em>
</p>

<hr>
The URDF_tutorial sets up RVIZ and the robot_state_publisher and joint_state_publisher to visualize the robot. We can do this manually by launching the necessary nodes. 

1. Run the robot_state_publisher and provide it with the URDF file in XML format.

  The robot_state_publisher requires the URDF to be in XML format, which is why we use xacro to convert our .xacro file to .xml. We can describe it as a robot_description parameter that is passed as a ROS parameter.

  ```bash
  ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$( xacro /home/jaspr/Documents/TUTbot/ros_test_ws/src/bumperbot_description/urdf/bumperbot.urdf.xacro)"
  ```
  
2. Run the joint_state_publisher_gui to control the joints.

  ```bash
  ros2 run joint_state_publisher_gui joint_state_publisher_gui
  ```

3. Run RViz to visualize the robot.

  ```bash
  ros2 run rviz2 rviz2
  ```

  3.1. For RVIZ to work properly, set the Fixed Frame to "base_footprint" in the Displays panel.\
  3.2. Add TF display to see the robot's coordinate frames.\
  3.3. Add RobotModel display to see the robot mesh. Set the Description topic to `/robot_description`.

This should then be saved into bumperbot_description/launch/ as a RVIZ configuration file (e.g., `display.rviz`).

With all these, we can visualize the robot model as the urdf_tutorial does.

This setup has multiple steps and is quite cumbersome to set up manually. You can use this launch file: [display.launch.py](../ros_test_ws/src/bumperbot_description/launch/display.launch.py) to automate the process.

```bash
ros2 launch bumperbot_description display.launch.py
```

# Gazebo Setup

## Why use Gazebo?

Gazebo is a 3D simulation environment that allows us to simulate the robot's behavior in a virtual world. It provides physics simulation, sensor simulation, and visualization capabilities.

For Gazebo to work with our existing setup, we need to define collision tags for each link in our URDF file. These collision tags define the shape and size of each link for physics simulation purposes. Since a mesh is a very complex geometry, it would be a good idea to use simpler collision shapes like boxes, cylinders, or spheres for better performance. For castors and wheels, I have defined spheres as collision shapes. Look for the `<collision>` tags in the [bumperbot.urdf.xacro](../ros_test_ws/src/bumperbot_description/urdf/bumperbot.urdf.xacro) file.

Next, we also need to define inertia properties for each link. These are defined using `<inertial>` tags in the URDF file. These can be generated using plugins for solidworks. The rest of the parameters required for Gazebo are in the [bumperbot_gazebo.urdf.xacro](../ros_test_ws/src/bumperbot_description/urdf/bumperbot_gazebo.urdf.xacro) file. This file is included within the bumperbot.urdf.xacro file and contains friction coefficients, damping values, and other physical properties needed for simulation. 

These parameters define how each link physically interacts with the ground in Gazebo’s contact model. The parameters mu1 and mu2 are the Coulomb friction coefficients in two orthogonal directions within the contact plane; setting them to extremely large values effectively enforces no slipping in either the rolling or lateral direction, which is a common but unphysical shortcut to guarantee perfect traction. The parameter kp is the contact stiffness, controlling how hard the wheel–ground contact is and how much penetration is allowed before a restoring force is applied, while kd is the contact damping term that dissipates energy and stabilizes the contact response to prevent oscillations or bouncing. The minDepth value specifies the minimum penetration depth required before contact forces are generated, helping to filter numerical noise, and maxVel limits the maximum corrective velocity used to resolve interpenetration, which improves numerical stability. Finally, fdir1 defines the primary friction direction in the link’s local frame.


See the [gazebo.launch.py](../ros_test_ws/src/bumperbot_bringup/launch/gazebo.launch.py) file for the gz launch configuration.

<p align="center">
  <img src="assets/gz_sim_1.png" alt="Gazebo simulation of the robot" width="600">
  <br>
  <em>Gazebo simulation of the robot with visualized joints.</em>
</p>

## Lets setup a Gazebo World 

Create the following new directories in the bumperbot_description package:
- `worlds/` - for Gazebo world files
- `models/` - for custom Gazebo models
- `photos/` - for photos of the world

The models were sourced from [aws-robotics](https://github.com/aws-robotics/aws-robomaker-small-house-world)

### TODO: add changes 

<p align="center">
  <img src="assets/small_house.gif" alt="Gazebo simulation of the robot" width="600">
  <br>
  <em>small house world.</em>
</p>

# Bringing sensors into Gazebo

### How to simulate an imu sensor?  

#### Create a new link `imu_link` in [bumperbot.urdf.xacro](../ros_test_ws/src/bumperbot_description/urdf/bumperbot.urdf.xacro) and add all the relevant tags.

```xml
<visual>
    <link name="imu_link">
        <inertial>
        <origin />
        <mass />
        <inertia  />
        </inertial>
        <visual>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <geometry>
            <mesh filename="package://bumperbot_description/meshes/imu_link.STL" />
        </geometry>
        <material name="">
            <color rgba="0.792156862745098 0.819607843137255 0.933333333333333 1" />
        </material>
        </visual>
        <collision>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <geometry>
            <mesh filename="package://bumperbot_description/meshes/imu_link.STL" />
        </geometry>
        </collision>
  </link>
```
#### Define the joint between imu_link and base_link
```xml
<joint name="imu_joint" type="fixed">
  <origin xyz="0 0 0.0698986241758014" rpy="0 0 0" />
  <parent link="base_link" />
  <child link="imu_link" />
  <axis xyz="0 0 0" />
</joint>
```
#### To similate IMU in gazebo, add the following plugin in [bumperbot_gazebo.urdf.xacro](../src/bumperbot_description/urdf/bumperbot_gazebo.urdf.xacro)
```xml
</plugin>
<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu">
</plugin>
```
#### Next, we need to add the relevant properties of imu 
This can be done in another gazebo tag in the same file. Set the reference to the name of the imu link. In this tag, we can adjust the uncertainities in each datafield of the imu sensor. Typically, the linear velocities have larger errors compared to the angular velocities. We can also adjust the name of topic it publishes at and the hz.
```xml
<gazebo reference="imu_link">
    <sensor name="imu" type="imu">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <gz_frame_id>imu_link</gz_frame_id>
        <topic>imu</topic>
        <imu>
          <angular_velocity>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
              </noise>
            </z>
          </angular_velocity>
          <linear_acceleration>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
              </noise>
            </z>
          </linear_acceleration>
        </imu>
    </sensor>
  </gazebo>
```
If we now launch gazebo, terminal should show that the imu has loaded:
```bash
[gazebo-2] [Dbg] [ImuSensor.cc:150] IMU data for [bumperbot::base_footprint::imu] advertised on [imu]
```
But the `/imu` topic cannot be accessed using ros2 topic list. This will require using the gazebo ros bridge. 

#### Add the gazebo ros2 brdige node to [gazebo.launch.py](../src/bumperbot_description/launch/gazebo.launch.py)
```xml
gz_ros2_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=[
        "/imu@sensor_msgs/msg/Imu[gz.msg.IMU",
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    ],
    remappings=[
        ("/imu", "/imu/out")
    ]
)
```
This node enables communication between Gazebo Transport topics and ROS 2 topics.

Bridged Topics:
- /imu → sensor_msgs/msg/Imu (Gazebo → ROS 2)
- /clock → rosgraph_msgs/msg/Clock (Gazebo → ROS 2)

Remapping:
- /imu is remapped to /imu/out for clearer separation of sensor output topics.

On running ros2 list topic, should see `/imu/out` being published. 

on echo:
```
header:
  stamp:
    sec: 73
    nanosec: 780000000
  frame_id: imu_link
orientation:
  x: 1.9589914616280593e-09
  y: -0.0052737159047662905
  z: 3.7158489066628803e-07
  w: 0.9999860938635186
orientation_covariance:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
angular_velocity:
  x: 9.887519563329777e-05
  y: 0.00020248904024318398
  z: 0.00012589436376863984
angular_velocity_covariance:
- 3.999999975690116e-08
- 0.0
- 0.0
- 0.0
- 3.999999975690116e-08
- 0.0
- 0.0
- 0.0
- 3.999999975690116e-08
linear_acceleration:
  x: 0.08772173886870498
  y: 0.01244104920254562
  z: 9.789098160543372
linear_acceleration_covariance:
- 0.00028899998869746923
- 0.0
- 0.0
- 0.0
- 0.00028899998869746923
- 0.0
- 0.0
- 0.0
- 0.00028899998869746923
```
info:
```
ros2 topic info /imu/out
Type: sensor_msgs/msg/Imu
Publisher count: 1
Subscription count: 0
```
Launch gz, controller, joystick, plotjuggler
<p align="center">
<img src="assets/sim_imu.gif" />
<br>
Simulated imu data on Plotjuggler
</p>
