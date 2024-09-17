#!/bin/python3

"""Follower drone mission example."""

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
from mission.follow_drone_module import FollowDroneModule
import rclpy


# Follow drone parameters
TAKEOFF_HEIGHT = 1.0  # Takeoff height in meters
SPEED = 0.2  # Max speed in m/s
SECURITY_DISTANCE_TO_LEADER = 0.4  # Meters
GOAL_THRESHOLD = 0.1  # Meters
LEADER_POSITION_SAMPLING_DISTANCE = 0.05  # Meters
RVIZ_DEBUG = True  # Enable RViz debug

SLEEP_TIME = 0.5  # Sleep time between actions (seconds)


def wait_to_takeoff(drone_interface: DroneInterfaceBase):
    """Wait for the drone to take off."""
    while drone_interface.info['state'] != PlatformStatus.FLYING:
        sleep(1.0)


def wait_for_land(drone_interface: DroneInterfaceBase):
    """Wait for the drone to take off."""
    while drone_interface.info['state'] != PlatformStatus.LANDING:
        sleep(1.0)


def follower_mission(follower_interface: DroneInterface, leader_interface: DroneInterfaceBase):
    """Run the mission."""
    follow_drone_module = FollowDroneModule(
        drone_interface=follower_interface,
        target_name_id=leader_interface.namespace,
        max_speed=SPEED,
        security_distance_to_leader=SECURITY_DISTANCE_TO_LEADER,
        goal_threshold=GOAL_THRESHOLD,
        leader_position_sampling_distance=LEADER_POSITION_SAMPLING_DISTANCE,
        debug=RVIZ_DEBUG)
    print(f'Follower {follower_interface.drone_id} ready')

    # Wait leader drone to take off
    print(f'Follower {follower_interface.drone_id} waiting '
          'for leader to take off')
    wait_to_takeoff(leader_interface)

    # ARM & OFFBOARD
    print(f'Follower {follower_interface.drone_id} arm')
    follower_interface.offboard()
    print(f'Follower {follower_interface.drone_id} offboard')
    follower_interface.arm()

    # TAKE OFF
    print(f'Follower {follower_interface.drone_id} take off')
    follower_interface.takeoff(TAKEOFF_HEIGHT, speed=0.5)

    if not follower_interface.takeoff.result:
        print(f'Follower {follower_interface.drone_id} take off failed')
        follower_interface.disarm()
        return
    print(f'Follower {follower_interface.drone_id} take off done')
    sleep(SLEEP_TIME)

    # FOLLOW UAV
    print(f'Follower {follower_interface.drone_id} following {leader_interface.namespace}')
    follow_drone_module.start_following()
    while rclpy.ok() and leader_interface.info['state'] == PlatformStatus.FLYING:
        follow_drone_module.continue_following()
        sleep(0.01)
    follow_drone_module.end_following()

    # LAND
    sleep(SLEEP_TIME)
    print(f'Follower {follower_interface.drone_id} finish')
    wait_for_land(leader_interface)
    print(f'Follower {follower_interface.drone_id} landing')
    follower_interface.land(speed=0.3)
    print(f'Follower {follower_interface.drone_id} land done')

    # DISARM
    follower_interface.disarm()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-l', '--leader_namespace',
                        type=str,
                        default='drone0',
                        help='Drone leader namespace')
    parser.add_argument('-f', '--follower_namespace',
                        type=str,
                        default='drone1',
                        help='Drone follower namespace')
    parser.add_argument('-v', '--verbose',
                        action='store_true',
                        default=False,
                        help='Enable verbose output')
    parser.add_argument('-r','--use_sim_time',
                        action='store_false',
                        default=True,
                        help='Use simulation time')

    args = parser.parse_args()
    follower_namespace = args.follower_namespace
    leader_namespace = args.leader_namespace
    verbosity = args.verbose
    use_sim_time = args.use_sim_time

    rclpy.init()
    follower_interface = DroneInterface(
        drone_id=follower_namespace,
        verbose=verbosity,
        spin_rate=100,
        use_sim_time=use_sim_time)

    leader_interface = DroneInterfaceBase(
        drone_id=leader_namespace,
        verbose=verbosity,
        use_sim_time=use_sim_time)

    follower_mission(
        follower_interface=follower_interface,
        leader_interface=leader_interface)

    follower_interface.shutdown()
    leader_interface.shutdown()
    rclpy.shutdown()
    print('Clean exit')
    exit(0)
