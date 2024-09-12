# Project Gazebo

Please refer to https://aerostack2.github.io/_02_examples/gazebo/project_gazebo/index.html for more information.

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

Launch your mission using the AS2 Python API:

```bash
python3 mission.py
```
