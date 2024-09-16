#!/bin/python3

"""
mission.py
"""

from time import sleep
import rclpy
from as2_python_api.drone_interface import DroneInterface


class DummyDrone(DroneInterface):
    def __init__(self, drone_name: str, verbose=False, use_sim_time=True):
        super().__init__(drone_name, verbose, use_sim_time)

    def preflight_checks(self):
        print("Offboard")
        self.offboard()
        print("Arm")
        self.arm()

    def do_mission(self):
        """ Run the mission """

        speed = 0.5
        takeoff_height = 1.0
        height = 1.0

        sleep_time = 1.0

        dim = 1.0
        path = [
            [-dim, dim, height],
            [-dim, -dim, height],
            [dim, -dim, height],
            [dim, dim, height]
        ]

        print("Start mission")

        self.preflight_checks()

        ##### TAKE OFF #####
        print("Take Off")
        self.takeoff(takeoff_height, speed=1.0)
        print("Take Off done")
        sleep(sleep_time)

        ##### GO TO #####
        for goal in path:
            print(f"Go to with path facing {goal}")
            self.go_to.go_to_point_path_facing(goal, speed=speed)
            print("Go to done")
        sleep(sleep_time)

        ##### LAND #####
        print("Landing")
        self.land(speed=0.5)
        print("Land done")

        self.disarm()


if __name__ == '__main__':
    rclpy.init()

    uav = DummyDrone("drone0", verbose=False, use_sim_time=True)

    uav.do_mission()

    uav.shutdown()
    rclpy.shutdown()

    print("Clean exit")
    exit(0)
