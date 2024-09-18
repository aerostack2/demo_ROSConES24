#!/bin/python3

from time import sleep
import rclpy
from as2_python_api.drone_interface import DroneInterface

USE_SIM_TIME = False
HEIGHT = 1.0
SQUARE_DIM = 1.0

if __name__ == '__main__':
    rclpy.init()

    uav = DroneInterface(
        "drone0",
        verbose=False,
        use_sim_time=USE_SIM_TIME)

    ##### ARM OFFBOARD #####
    print("Offboard")
    uav.offboard()
    print("Arm")
    uav.arm()

    ##### TAKE OFF #####
    print("Take Off")
    uav.takeoff(height=HEIGHT, speed=1.0)
    sleep(1.0)

    ##### GO TO #####
    path = [
        [-SQUARE_DIM, SQUARE_DIM, HEIGHT],
        [-SQUARE_DIM, -SQUARE_DIM, HEIGHT],
        [SQUARE_DIM, -SQUARE_DIM, HEIGHT],
        [SQUARE_DIM, SQUARE_DIM, HEIGHT]
    ]
    for goal in path:
        print(f"Go to with path facing {goal}")
        uav.go_to.go_to_point_path_facing(goal, speed=0.5)
        print("Go to done")
    sleep(1.0)

    ##### LAND #####
    print("Landing")
    uav.land(speed=0.5)

    uav.shutdown()
    rclpy.shutdown()
