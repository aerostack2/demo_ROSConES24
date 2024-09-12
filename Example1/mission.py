#!/usr/bin/env python3

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
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
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

"""Simple mission for a single drone using motion references."""

__authors__ = 'Rafael Perez-Segui, Pedro Arias-Perez'
__copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
__license__ = 'BSD-3-Clause'

from time import sleep

from as2_python_api.drone_interface_teleop import DroneInterfaceTeleop
import rclpy

rclpy.init()

uav = DroneInterfaceTeleop(
    drone_id='drone0',
    use_sim_time=True,
    verbose=True)

position = [1.0, 0.0, 1.0]
max_speed = 0.5
yaw_speed = 0.0

print('Go to point: ', position)
time = 0
while time < 5:
    uav.motion_ref_handler.position.send_position_command_with_yaw_speed(
        position, max_speed, pose_frame_id='earth', twist_frame_id='earth', yaw_speed=0.0)
    sleep(1)
    time += 1
uav.motion_ref_handler.hover()

print('Hovering')
uav.motion_ref_handler.hover()
sleep(4)

speed = [-0.5, 0.0, 0.0]
print('Move at speed: ', max_speed)
time = 0
while time < 5:
    uav.motion_ref_handler.speed.send_speed_command_with_yaw_speed(
        speed, twist_frame_id='earth', yaw_speed=0.0)
    sleep(1)
    time += 1

print('Hovering')
time = 0
while time < 5:
    uav.motion_ref_handler.speed.send_speed_command_with_yaw_speed(
        [0.0, 0.0, 0.0], twist_frame_id='earth', yaw_speed=0.0)
    sleep(1)
    time += 1

uav.shutdown()
rclpy.shutdown()
print('Clean exit')
exit(0)
