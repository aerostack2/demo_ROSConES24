#!/bin/python3

"""
mission.py
"""

from time import sleep
import rclpy
from as2_python_api.drone_interface import DroneInterface

if __name__ == '__main__':
    rclpy.init()

    uav = DroneInterface("drone0", verbose=False, use_sim_time=True)

    sleep_time = 1.0

    height = 1.0
    dim = 1.0
    path = [
        [-dim, dim, height],
        [-dim, -dim, height],
        [dim, -dim, height],
        [dim, dim, height]
    ]

    ##### ARM OFFBOARD #####
    print("Offboard")
    uav.offboard()
    print("Arm")
    uav.arm()

    ##### TAKE OFF #####
    print("Take Off")
    uav.takeoff(height=1.0, speed=1.0)
    sleep(sleep_time)

    ##### GO TO #####
    for goal in path:
        print(f"Go to with path facing {goal}")
        uav.go_to.go_to_point_path_facing(goal, speed=0.5)
        print("Go to done")
    sleep(sleep_time)

    ##### LAND #####
    print("Landing")
    uav.land(speed=0.5)

    uav.disarm()
    uav.shutdown()
    rclpy.shutdown()
