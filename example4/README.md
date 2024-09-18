# Example 4: Drone-convoy example

### 1A. Launching Aerostack2 components for simulation

For launching the demo, usign **Gazebo**, we automated the process using tmuxinator. You can run the following command:

```bash
tmuxinator start -p aerostack2_gazebo.yaml drone_namespace=drone0,drone1,drone2
```

### 1B. Launching Aerostack2 components for crazyflie

For launching the demo using **Crazyflie**, we automated the process using tmuxinator. You can run the following command:

```bash
tmuxinator start -p aerostack2_crazyflie.yaml drone_namespace=drone0,drone1,drone2
```

### 2. Launching the ground station utilities

For monitoring the drone status using RViz and send commands to it using Aerostack2 Keyboard Teleoperation, you can run the following command:

```bash
tmuxinator start -p ground_station.yaml drone_namespace=drone0,drone1,drone2 use_sim_time=true
```

> *Note: If not using Gazebo, set use_sim_time to False.*

### 3. Launching the follow-drone mission

For running the follow-drone mission you can run the following command for launching leader mission:
```bash
python3 mission_leader.py
```

> *Note: If not using Gazebo, add '-r' flag to the mission scripts to set use_sim_time to False.*

### 4. Stop everything

For stopping the demo you can run the following commands:

```bash
./stop.bash
```
