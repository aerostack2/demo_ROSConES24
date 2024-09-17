## Launching Aerostack2 components for crazyflie

For launching the demo using **Crazyflie**, we automated the process using tmuxinator. You can run the following command:

- Launch aerostack2 drones:
```
tmuxinator start -p aerostack2_crazyflie.yaml drone_namespace=drone0,drone1,drone2
```

## Launching Aerostack2 components for simulation

For launching the demo, usign **Gazebo**, we automated the process using tmuxinator. You can run the following command:

- Launch aerostack2 drones:
```
tmuxinator start -p aerostack2_gazebo.yaml drone_namespace=drone0,drone1,drone2
```

## Launching the ground station utilities

For monitoring the drone status using RViz and send commands to it using Aerostack2 Keyboard Teleoperation, you can run the following command:

- Launch the ground station:
```
tmuxinator start -p ground_station.yaml use_sim_time:=true
```

*Note: If not using Gazebo, set use_sim_time to False.*

## Launching the follow-drone mission

For running the follow-drone mission you can run the following commands:

- Launch leader mission:
```
python3 mission_leader.py
```

- Launch follower mission 1:
```
python3 mission_follower.py -f drone1 -l drone0
```

- Launch follower mission 2:
```
python3 mission_follower.py -f drone2 -l drone1
```

*Note: If not using Gazebo, add '-r' flag to the mission scripts to set use_sim_time to False.*

## Stop everything

For stopping the demo you can run the following commands:

```
./stop.bash
```

TODO: 
- -r de los mission
- config con los drones address
- readme, tmuxinator