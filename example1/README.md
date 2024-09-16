# Motion Reference Control example
Please refer to default [project gazebo](https://aerostack2.github.io/_02_examples/gazebo/project_gazebo/index.html) for more information.

## Installation

To install this project, clone the repository:

```bash
git clone https://github.com/aerostack2/demo_ROSConES24.git
```

To start using this project, please go to the root folder of the project.

## Execution

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
ros2 launch as2_motion_controller controller_launch.py namespace:=drone0 config_file:=config.yaml plugin_name:=pid_speed_controller plugin_config_file:=pid_speed_controller.yaml
```

### 3. Launch a mission

First, take off. It can be done manually on the cli, but here you have a brief script to speed up:
```bash
./takeoff_drone.bash
```

Now, let's move to point [1, 0, 1]. Control mode at the controller is not set. You can set it by calling to controller service `set_control_mode`: 
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

After setting the "POSITION" control mode, you can send a new reference to the controller:
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
    z: 1.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

Try to do the same but using speed references:
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

And sending new speed references:
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

### 4. Launch a mission through Python API

Launch your mission using the AS2 Python API:

```bash
python3 mission.py
```
