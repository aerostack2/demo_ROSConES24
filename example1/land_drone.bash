#!/bin/bash

# Default value: drone0
drone=${drone:-drone0}

# int8 EMERGENCY     = -1
# int8 ARM           = 0
# int8 DISARM        = 1
# int8 TAKE_OFF      = 2
# int8 TOOK_OFF      = 3
# int8 LAND          = 4
# int8 LANDED        = 5

sleep 0.5
ros2 service call /$drone/platform/state_machine_event as2_msgs/srv/SetPlatformStateMachineEvent "event:
  event: 4"

sleep 0.5
ros2 service call /$drone/platform_land std_srvs/srv/SetBool "data: true"
