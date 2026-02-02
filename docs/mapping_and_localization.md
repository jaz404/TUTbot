# Mapping and Localization

In mobile robotics, mapping and localization rely on a hierarchy of coordinate frames to describe the robot’s pose consistently over time. We adopt the standard ROS frame convention:

- `odom` frame: The local odometry frame. This frame provides a smooth, continuous estimate of the robot’s motion based on wheel encoders, IMU integration, or visual odometry. It is locally accurate but drifts over time.
- `base_link` frame: The robot’s body-fixed frame. This frame is rigidly attached to the robot and moves with it. All onboard sensors (LiDAR, camera, IMU) are defined relative to base_link.
- `map` frame: The global, world-fixed frame. This frame is used for long-term consistency, global planning, and loop-closure corrected localization. It does not drift, but estimates relative to it may jump when global corrections are applied.

#### Frame Relationships
- base_link is dynamic (time-varying).
- odom and map are fixed world frames (they do not move relative to the environment).

For example the if the robot starts always at the odom frame (lets say it is the charging dock) then there would be a fixed transformation from map frame to the odom frame. Then, one more transformation from odom frame to the non static base_link frame which moves with the robot. 

## 1. Local Localization / Odometry

Local localization, commonly referred to as **odometry**, estimates the robot’s pose relative to its starting position, without using any global map. It provides the transform between the `odom` and `base_link` frames and is typically computed using proprioceptive sensors such as wheel encoders, IMUs, or visual odometry. Odometry is designed to be smooth and continuous over time, making it well suited for short-term motion estimation and control.

However, odometry is inherently subject to drift due to sensor noise, wheel slip, and integration errors. This drift accumulates unbounded over time, meaning that while the relative motion estimate is locally accurate, the absolute pose of the robot in the environment becomes increasingly incorrect the longer the robot operates without correction.

### 1.1. Odometry Motion Model

The odometry motion model describes how the robot’s pose evolves over time based on incremental motion measurements obtained from odometry sensors (e.g., wheel encoders). The robot pose is represented as a state vector:

x = [x, y, θ]^T

where x and y define the position in the plane, and θ is the robot’s orientation.

Given a previous pose (x, y, θ), the new pose (x', y', θ') is computed by applying a relative motion composed of a translational displacement and rotational changes. The motion is parameterized by:

- δ̂_trans : estimated forward translation  
- δ̂_rot1  : estimated initial rotation  
- δ̂_rot2  : estimated final rotation  

The state update equations are:

x' = x + δ̂_trans · cos(θ + δ̂_rot1)  
y' = y + δ̂_trans · sin(θ + δ̂_rot1)  
θ' = θ + δ̂_rot1 + δ̂_rot2  

This formulation decomposes the robot’s motion into three sequential components: an initial rotation, a translation, and a final rotation. The first rotation (δ̂_rot1) aligns the robot with the direction of motion, allowing the translation to be applied along the correct heading. After translating, the second rotation (δ̂_rot2) accounts for the remaining change in orientation, ensuring that the robot reaches the correct final pose.

Using two rotations allows the model to represent general planar motion, including curved trajectories, using simple primitives. This decomposition closely approximates real robot motion and enables more accurate modeling of uncertainty, since rotational and translational errors can be treated separately.

While this motion model is simple and computationally efficient, it is inherently approximate. Errors in the estimated motion parameters accumulate over time due to wheel slip, uneven terrain, and sensor noise, leading to drift. As a result, the odometry motion model is well suited for short-term pose prediction and local localization, but requires global localization or sensor fusion to maintain long-term accuracy.

### 1.2. [odometry_motion_model.py](../src/bumperbot_localization/bumperbot_localization/odometry_motion_model.py) with Uncertainty

<p align="center">
<!-- gif -->
<img src="assets/odom_model.gif">
<br>
<em>Odometry Motion Model with Uncertainty</em>
</p>

In practice, odometry measurements are corrupted by noise arising from wheel slip, uneven terrain, encoder quantization, and unmodeled dynamics. To account for this uncertainty, the odometry motion model augments the noise-free motion parameters with stochastic error terms.

The true motion parameters are modeled as noisy versions of the ideal increments:

δ̂_rot1  = δ_rot1  − noise(α1 · |δ_rot1| + α2 · |δ_trans|)  
δ̂_trans = δ_trans − noise(α3 · |δ_trans| + α4 · (|δ_rot1| + |δ_rot2|))  
δ̂_rot2  = δ_rot2  − noise(α1 · |δ_rot2| + α2 · |δ_trans|)  

Here, `noise(·)` denotes a zero-mean random variable (typically Gaussian), and the parameters α1 through α4 control how different types of motion contribute to uncertainty.

- α1 models rotational noise proportional to rotation
- α2 models rotational noise induced by translation
- α3 models translational noise proportional to translation
- α4 models translational noise induced by rotation

These are defined as declarable parameters in the file along with number of parameters:
```py
self.declare_parameter('alpha1', 0.1)
self.declare_parameter('alpha2', 0.1)
self.declare_parameter('alpha3', 0.1)
self.declare_parameter('alpha4', 0.1)
self.declare_parameter('nr_samples', 300)
```

This formulation captures the empirical observation that translation and rotation errors are coupled: large translations tend to introduce orientation error, and large rotations tend to introduce position error. As a result, uncertainty grows anisotropically depending on the type of motion executed.

By explicitly modeling this noise, the odometry motion model can be used in probabilistic localization frameworks such as particle filters, where samples are drawn from the motion distribution rather than applying a deterministic state update. This allows the localization system to represent and propagate uncertainty over time, rather than relying on a single drifting pose estimate.

## 2. Global Localization

Global localization estimates the robot’s pose with respect to a known map and is responsible for correcting the drift accumulated by odometry. Rather than directly estimating `map → base_link`, it computes the transform between the `map` and `odom` frames, which indirectly places `base_link` in the global frame through the existing `odom → base_link` transform.

Global localization assumes that local localization is already available and reasonably accurate in the short term. It uses exteroceptive sensors such as LiDAR or cameras to match current observations against the map, producing global pose corrections. These corrections may be non-smooth or discontinuous but ensure long-term consistency and enable reliable global navigation and planning.

## 3. SLAM (Simultaneous Localization and Mapping)
When mapping and localization are performed together, the system is referred to as SLAM.