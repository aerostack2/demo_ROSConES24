# Plan Execution Control example
Please refer to default [project gazebo](https://aerostack2.github.io/_02_examples/gazebo/project_gazebo/index.html) for more information.

## Installation

To install this project, clone the repository:

```bash
git clone https://github.com/aerostack2/demo_ROSConES24.git
```

To start using this project, please go to the root folder of the project.

## Execution

### 1. Launch Aerostack using tmuxinator

```bash
tmuxinator start -p aerostack2.yml drone_namespace=drone0 simulation_config=world.yaml
```

Or use `launch_as2.bash` script. You can close tmux session using `stop.bash` script or manually killing tmux session (`tmux kill-session`).

### 2. Run mission
```bash
python3 mission.py
```