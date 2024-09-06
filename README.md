# F1/10 Docking using MPC
This ROS2 package enables an F1/10 car to dock autonomously at a docking station using a Model Predictive Control (MPC) algorithm. The package employs a custom objective function with configurable constraints specified in the config.yaml file. The vehicle's kinematics are modeled using a bicycle model.

## Package Architecture
This ROS2 package operates as an action server designed to manage the docking process. Users can send desired positions to the server using a custom Docking message type.<div align="center">
  <image src="https://github.com/user-attachments/assets/b8b59a57-6e72-4494-9a07-53b8e99aa071" controls>
  </image>
</div>

## Docking Preview
The video showcases the F1/10 car performing the docking maneuver using the MPC-based control algorithm at a designated docking station.
<div align="center">
  <video src="https://github.com/user-attachments/assets/914b583d-18ca-4e05-b7f5-82cfd983579f" controls>
    Your browser does not support the video tag.
  </video>
</div>

## Bicycle Model Equations

### State Variables:
- $x_{\text{pos}}$: x-position of the car
- $y_{\text{pos}}$: y-position of the car
- $\theta$: Orientation angle of the car (heading)
- $\delta$: Steering angle

### Control Inputs:
- $v$: Linear velocity of the car
- $\phi$: Steering rate (rate of change of steering angle)

### Parameters:
- $L$: Wheelbase (distance between front and rear wheels)

### State Dynamics:

The model uses the following continuous-time equations to describe the evolution of the state variables:

```math
\frac{dx_{\text{pos}}}{dt} = v \cdot \cos(\theta)
```

```math
\frac{dy_{\text{pos}}}{dt} = v \cdot \sin(\theta)
```

```math
\frac{d\theta}{dt} = \frac{v \cdot \tan(\delta)}{L}
```

```math
\frac{d\delta}{dt} = \phi
```

## Objective Function

The objective function is central to the MPC, guiding the controller to make optimal decisions by balancing multiple factors such as position accuracy, orientation, steering angles, and control efforts.

### Components of the Objective Function

1. **Stage Cost**:

   The stage cost evaluates the performance at each step within the prediction horizon. It consists of the following terms:
   ```math
   lterm = \left( \left( \frac{x_{\text{pos}} - x_{\text{set}}}{x_{\text{upper}} - x_{\text{lower}}} \times G_{\text{pos}} \right)^2 + \left( \frac{y_{\text{pos}} - y_{\text{set}}}{y_{\text{upper}} - y_{\text{lower}}} \times G_{\text{pos}} \right)^2 + \left( (\theta - \theta_{\text{set}}) \times G_{\theta} \right)^2 + \left( (\delta - \delta_{\text{set}}) \times G_{\delta} \right)^2 \right)
   ```
1. **Terminal Cost**:
   
   The terminal cost evaluates the performance at the end of the prediction horizon. In this project, the terminal cost is set to be equal to the stage cost:
   ```math
   mterm = lterm
   ```

1. **Control Effort Penalties**:
   
   To avoid aggressive control actions, penalties are applied to the control inputs:
     ```math
     rterm_v = 1 \times 10^{-2}
     ```
     
     ```math
     rterm_\phi = 1 \times 10^{-2}
     ```

### Combined Objective Function

The overall objective function combines the terminal and stage costs:
```math
\text{Objective} = \sum_{k=0}^{N-1} lterm_k + mterm_N + \sum_{k=0}^{N-1} (rterm_v \cdot v_k + rterm_\phi \cdot \phi_k)
```
where $N$ is the prediction horizon.



## How to test

Edit the config file:
```bash
nvim f1tenth_docking/config/config.yaml
```

Start the docking action server:
```bash
ros2 launch f1tenth_docking f1tenth_docking.launch.py
```

Mock the vesc servo pose:
```bash
ros2 topic pub -r 100 /vesc/core vesc_msgs/VescStateStamped '{state: {servo_pose: 0.0}}'
```

Mock the optitrack pose:
```bash
ros2 topic pub -r 100 /optitrack/rigid_body_0 geometry_msgs/PoseStamped '{pose: {position: {x: 1.0, y: 2.0, z: 3.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```

Set the control loop setpoint:
```bash
ros2 action send_goal docking_action_server f1tenth_docking_interfaces/action/Docking "{setpoint: {x_pos: 497, y_pos: 2.0, theta: 0.0, delta: 0.0}}"
```
