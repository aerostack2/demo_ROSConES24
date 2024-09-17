#!/bin/python3

"""Leader drone mission example."""

# Copyright 2024 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


__authors__ = 'Rafael Pérez Seguí'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

import argparse
from time import sleep

from as2_msgs.msg import PlatformStatus
from as2_python_api.drone_interface import DroneInterface, DroneInterfaceBase
import rclpy

LEADER_MAX_SPEED = 0.2  # Maximum speed for the leader drone (m/s)
SLEEP_TIME = 0.5  # Sleep time between actions (seconds)
SD_MIN_X = -0.3  # Minimum square dimension for the mission (m)
SD_MAX_X = 0.7  # Maximum square dimension for the mission (m)
SD_MIN_Y = -0.7  # Minimum square dimension for the mission (m)
SD_MAX_Y = 0.4  # Maximum square dimension for the mission (m)

LOWEST_HEIGHT = 1.0  # Lowest height for the mission (m)
HIGHEST_HEIGHT = 1.5  # Highest height for the mission (m)


def wait_to_takeoff(drone_interface: DroneInterfaceBase):
    """Wait for the drone to take off."""
    while drone_interface.info['state'] != PlatformStatus.FLYING:
        sleep(1.0)


def wait_for_land(drone_interface: DroneInterfaceBase):
    """Wait for the drone to take off."""
    while drone_interface.info['state'] != PlatformStatus.LANDING:
        sleep(1.0)


def leader_mission(leader_interface: DroneInterface, follower_interface_list: list):
    """Run the mission."""
    # ARM & OFFBOARD
    print(f'Leader {leader_interface.drone_id} arm')
    leader_interface.offboard()
    print(f'Leader {leader_interface.drone_id} offboard')
    leader_interface.arm()

    # TAKE OFF
    print(f'Leader {leader_interface.drone_id} take off')
    leader_interface.takeoff(LOWEST_HEIGHT, speed=0.5)

    if not leader_interface.takeoff.result:
        print(f'Leader {leader_interface.drone_id} take off failed')
        leader_interface.disarm()
        return
    print(f'Leader {leader_interface.drone_id} take off done')

    # WAIT FOLLOWERS TO TAKE OFF
    for follower_interface in follower_interface_list:
        print(f'Leader {leader_interface.drone_id} waiting '
              'for {follower_interface.drone_id} to take off')
        wait_to_takeoff(follower_interface)
        print(f'Leader {leader_interface.drone_id} - '
              '{follower_interface.drone_id} take off done')
    sleep(SLEEP_TIME)

    # SIMPLE MISSION
    print(f'Leader {leader_interface.drone_id} simple mission')
    path = [
        [SD_MAX_X, SD_MAX_Y, LOWEST_HEIGHT],  # Forward
        [SD_MAX_X, SD_MIN_Y, LOWEST_HEIGHT],  # Right
        [SD_MAX_X, SD_MIN_Y, HIGHEST_HEIGHT],  # Up
        [SD_MIN_X, SD_MIN_Y, HIGHEST_HEIGHT],  # Backward
        [SD_MIN_X, SD_MIN_Y, LOWEST_HEIGHT],  # Down
        [SD_MIN_X, SD_MAX_Y, LOWEST_HEIGHT],  # Left
        [SD_MAX_X, SD_MAX_Y, LOWEST_HEIGHT],  # Forward
        [SD_MAX_X, SD_MIN_Y, LOWEST_HEIGHT],  # Right
    ]

    for goal in path:
        print(f'Leader {leader_interface.drone_id} go to with path facing {goal}')
        leader_interface.go_to.go_to_point_path_facing(goal, speed=LEADER_MAX_SPEED)
        print('Leader {leader_interface.drone_id} go to done')
    sleep(7.0)

    # LAND
    sleep(SLEEP_TIME)
    print(f'Leader {leader_interface.drone_id} finish')
    print(f'Leader {leader_interface.drone_id} landing')
    leader_interface.land(speed=0.3)
    print(f'Leader {leader_interface.drone_id} land done')

    # DISARM
    leader_interface.disarm()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-l', '--leader_namespace',
                        type=str,
                        default='drone0',
                        help='Drone leader namespace')
    parser.add_argument('-f', '--namespaces',
                        type=str,
                        nargs='+',
                        default=['drone1', 'drone2'],
                        help='Drone follower namespaces (space-separated list)')
    parser.add_argument('-v', '--verbose',
                        action='store_true',
                        default=False,
                        help='Enable verbose output')
    parser.add_argument('-r','--use_sim_time',
                        action='store_false',
                        default=True,
                        help='Use simulation time')
                        

    args = parser.parse_args()
    leader_namespace = args.leader_namespace
    followers_namespaces = args.namespaces
    verbosity = args.verbose
    use_sim_time = args.use_sim_time

    print(f"Use simulation time: {use_sim_time}")

    rclpy.init()
    leader_interface = DroneInterface(
        drone_id=leader_namespace,
        verbose=verbosity,
        use_sim_time=use_sim_time)

    follower_interface_list = []
    for drone_namespace in followers_namespaces:
        follower_interface = DroneInterfaceBase(
            drone_id=drone_namespace,
            verbose=verbosity,
            use_sim_time=use_sim_time)
        follower_interface_list.append(follower_interface)

    leader_mission(
        leader_interface=leader_interface,
        follower_interface_list=follower_interface_list)

    leader_interface.shutdown()
    for follower_interface in follower_interface_list:
        follower_interface.shutdown()
    rclpy.shutdown()
    print('Clean exit')
    exit(0)
