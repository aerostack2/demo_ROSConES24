#!/bin/bash

ros2 service call /drone0/set_offboard_mode std_srvs/srv/SetBool "data: true"

ros2 service call /drone0/set_arming_state std_srvs/srv/SetBool "data: true"

ros2 service call /drone0/platform/state_machine_event as2_msgs/srv/SetPlatformStateMachineEvent "event:
  event: 2"

ros2 service call /drone0/platform_takeoff std_srvs/srv/SetBool "data: true"
