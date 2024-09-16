# Example 1: Basic control of a drone in Gazebo using motion references

### 1. Launch gazebo
To launch the gazebo simulation, execute once the following command:

```bash
ros2 launch as2_gazebo_assets launch_simulation.py simulation_config_file:=world.yaml
```

### 2. Launch aerostack2 nodes
To launch aerostack2 nodes, execute once the following command:

- Aerial platform:
```bash
ros2 launch as2_platform_gazebo platform_gazebo_launch.py namespace:=drone0 platform_config_file:=config.yaml simulation_config_file:=world.yaml
```

- State estimator:
```bash
ros2 launch as2_state_estimator state_estimator_launch.py namespace:=drone0 config_file:=config.yaml
```

- Motion controller:
```bash
ros2 launch as2_motion_controller controller_launch.py namespace:=drone0 config_file:=config.yaml
```

### 3. Move the drone

#### Take Off the drone

It can be done manually on the cli, but here you have a brief script to speed up:
```bash
./takeoff_drone.bash
```

#### Land the drone

It can be done manually on the cli, but here you have a brief script to speed up:
```bash
./land_drone.bash
```

#### Move the drone

Take a look at what control modes are available:
| YAW_MODE |   Value   |
|----------|:---------:|
|   NONE   |     0     |
| YAW_ANGLE |    1     |
| YAW_SPEED |    2     |
    

| CONTROL_MODE |   Value   |
|----------|:---------:|
| UNSET  |  0  |
| HOVER  |  1  |
| POSITION  |  2  |
| SPEED  |  3  |
| SPEED_IN_A_PLANE  |  4  |
| ATTITUDE  |  5  |
| ACRO  |  6  |
| TRAJECTORY  |  7  |
| ACEL  |  8  |


| REFERENCE_FRAME |   Value   |
|----------|:---------:|
|  UNDEFINED_FRAME  |  0  |
|  LOCAL_ENU_FRAME  |  1  |
|  BODY_FLU_FRAME  |  2  |
|  GLOBAL_LAT_LONG_ASML  | 3   |

Now, let's move to point [1.0, 0.0, 1.5]. Control mode at the controller is not set. You can set it by calling to controller service `set_control_mode`, setting the control to **POSITION MODE**:

```bash
ros2 service call /drone0/controller/set_control_mode as2_msgs/srv/SetControlMode "control_mode:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: ''
  yaw_mode: 1
  control_mode: 2
  reference_frame: 1"
```

After setting the position control mode, you can send a new **POSITION REFERENCE** to the controller:
```bash
ros2 topic pub /drone0/motion_reference/pose geometry_msgs/msg/PoseStamped "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'earth'
pose:
  position:
    x: 1.0
    y: 0.0
    z: 1.5
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

Try to do the same but using speed references. Set the control mode to **SPEED MODE**:
```bash
ros2 service call /drone0/controller/set_control_mode as2_msgs/srv/SetControlMode "control_mode:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: ''
  yaw_mode: 2
  control_mode: 3
  reference_frame: 2"
```

And sending new **SPEED REFERENCE** to the controller:
```bash
ros2 topic pub /drone0/motion_reference/twist geometry_msgs/msg/TwistStamped "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'drone0/base_link'
twist:
  linear:
    x: -0.5
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0" 
```

```bash
ros2 topic pub /drone0/motion_reference/twist geometry_msgs/msg/TwistStamped "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'drone0/base_link'
twist:
  linear:
    x: 0.0
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: 0.0" 
```

### 4. Move the drone using Python API

#### Take Off the drone

It can be done manually on the cli, but here you have a brief script to speed up:
```bash
./takeoff_drone.bash
```

#### Move the drone
Launch your mission using the AS2 Python API:

```bash
python3 mission.py
```

#### Land the drone

It can be done manually on the cli, but here you have a brief script to speed up:
```bash
./land_drone.bash
```

