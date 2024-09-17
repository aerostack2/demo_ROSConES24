#!/bin/python3

"""Follow Drone Module."""

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


from collections import deque
from math import atan2, sqrt
from typing import TYPE_CHECKING

import as2_core.as2_names as as2
from as2_motion_reference_handlers.hover_motion import HoverMotion
from as2_motion_reference_handlers.position_motion import PositionMotion
from as2_motion_reference_handlers.speed_motion import SpeedMotion
from geometry_msgs.msg import PoseStamped, Quaternion, TwistStamped
import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_system_default
from visualization_msgs.msg import Marker

if TYPE_CHECKING:
    from as2_python_api.drone_interface import DroneInterfaceBase


MARKERS_LIFETIME = 20  # Debug variable


def yaw_from_quaternion(quaternion: Quaternion) -> float:
    """
    Get yaw angle from quaternion.

    :param quaternion: Quaternion
    :type quaternion: Quaternion
    :return: Yaw angle
    :rtype: float
    """
    # Extract quaternion components
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    # Calculate yaw angle
    yaw = atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z)
    return yaw


class FollowDroneModule():
    """Follow Drone Module."""

    def __init__(self,
                 drone_interface: 'DroneInterfaceBase',
                 target_name_id: str,
                 max_speed: float,
                 security_distance_to_leader: float = 0.5,
                 goal_threshold: float = 0.2,
                 leader_position_sampling_distance: float = 0.05,
                 leader_queue_size: int = 1000,
                 debug: bool = False) -> None:
        """
        Follow Drone Module constructor.

        :param drone_interface: Drone interface
        :type drone_interface: DroneInterfaceBase
        :param target_name_id: Leader name id
        :type target_name_id: str
        :param max_speed: Max speed
        :type max_speed: float
        :param security_distance_to_leader: Security distance, defaults to 0.5
        :type security_distance_to_leader: float, optional
        :param goal_threshold: Goal threshold, defaults to 0.2
        :type goal_threshold: float, optional
        :param leader_position_sampling_distance: Leader position sampling distance,
        defaults to 0.05
        :type leader_position_sampling_distance: float, optional
        :param leader_queue_size: Leader queue size, defaults to 1000
        :type leader_queue_size: int, optional
        :param debug: Enable RViz debug, defaults to False
        :type debug: bool, optional
        """
        # Assign drone interface
        self.drone_interface = drone_interface
        self.__namespace = drone_interface.drone_id

        # Creates a subscription to the target drone self_localization/pose topic
        topic = '/' + target_name_id + '/' + as2.topics.self_localization.pose
        self.__leader_pose_sub = self.drone_interface.create_subscription(
            PoseStamped,
            topic,
            self.__leader_pose_callback,
            rclpy.qos.qos_profile_sensor_data)

        # Set max speed
        self.__max_speed = TwistStamped()
        self.__max_speed.twist.linear.x = max_speed
        self.__max_speed.twist.linear.y = max_speed
        self.__max_speed.twist.linear.z = max_speed

        # Assign parameters
        self.__security_distance_to_leader = security_distance_to_leader
        self.__goal_threshold = goal_threshold
        self.__leader_position_sampling_distance = leader_position_sampling_distance

        # Internal variables
        self.__leader_poses_queue = deque(maxlen=leader_queue_size)
        self.__initialize = False
        self.__fist_leader_pose = None

        # Motion handlers
        self.__motion_handler_position = PositionMotion(self.drone_interface)
        self.__motion_handler_speed = SpeedMotion(self.drone_interface)
        self.__motion_handler_hover = HoverMotion(self.drone_interface)

        # Debug variables using RViz
        self.__debug = debug
        if self.__debug:
            self.__debug_poses_pub = self.drone_interface.create_publisher(
                Marker, f'/viz/{self.__namespace}/queue_poses', qos_profile_system_default
            )

            self.__marker = Marker()
            self.__marker.ns = 'follow_drone_' + self.__namespace
            self.__marker.id = 1
            self.__marker.type = Marker.SPHERE
            self.__marker.action = Marker.ADD
            self.__marker.lifetime = Duration(seconds=MARKERS_LIFETIME).to_msg()
            self.__marker.scale.x = 0.05
            self.__marker.scale.y = 0.05
            self.__marker.scale.z = 0.05
            self.__marker.color.r = 0.5
            self.__marker.color.g = 0.5
            self.__marker.color.b = 0.5
            self.__marker.color.a = 1.0

        self.drone_interface.get_logger().info('Follow drone module initialized')

    def start_following(self) -> None:
        """Start following the drone."""
        self.__initialize = True
        self.drone_interface.get_logger().info('Follow drone started')

    def end_following(self) -> None:
        """End following the drone."""
        if self.__initialize and self.__motion_handler_hover is not None:
            self.hover()
        self.__initialize = False
        self.__leader_poses_queue.clear()
        self.drone_interface.get_logger().info('Follow drone ended')

    def continue_following(self) -> None:
        """Continue following the drone."""
        if len(self.__leader_poses_queue) == 0:
            return

        # Get distance between current drone pose and queue oldest element
        # current_pose = self.drone_interface.__pose
        current_position = self.drone_interface.position
        current_pose = PoseStamped()
        current_pose.pose.position.x = current_position[0]
        current_pose.pose.position.y = current_position[1]
        current_pose.pose.position.z = current_position[2]

        leader_pose = self.__leader_poses_queue[-1]
        distance = self.__distance(current_pose.pose, leader_pose.pose)

        # If distance is less than security distance to leader, hover the drone
        if distance < self.__security_distance_to_leader:
            self.hover()
            return

        # Goal
        goal_reference = self.__leader_poses_queue[0]

        # Distance between current drone pose and goal
        distance = self.__distance(current_pose.pose, goal_reference.pose)

        if distance < self.__goal_threshold:
            # Remove oldest element if queue size is greater than 1
            if len(self.__leader_poses_queue) > 1:
                self.__leader_poses_queue.popleft()
                goal_reference = self.__leader_poses_queue[0]

        # Send motion reference command
        yaw = yaw_from_quaternion(goal_reference.pose.orientation)
        self.__max_speed.header.frame_id = goal_reference.header.frame_id

        if len(self.__leader_poses_queue) > 1:
            twist_motion_reference = self.__get_twist_motion_reference(
                current_position,
                [goal_reference.pose.position.x,
                 goal_reference.pose.position.y,
                 goal_reference.pose.position.z],
                self.__max_speed)
            if not self.__motion_handler_speed.send_speed_command_with_yaw_angle(
                    twist_motion_reference,
                    twist_frame_id=twist_motion_reference.header.frame_id,
                    yaw_angle=yaw):
                self.drone_interface.get_logger().error('Error sending speed command')
        else:
            if not self.__motion_handler_position.send_position_command_with_yaw_angle(
                    pose=goal_reference,
                    twist_limit=self.__max_speed,
                    pose_frame_id=goal_reference.header.frame_id,
                    twist_frame_id=goal_reference.header.frame_id,
                    yaw_angle=yaw):
                self.drone_interface.get_logger().error('Error sending position command')

    def hover(self) -> bool:
        """Hover the drone."""
        if self.__motion_handler_hover is not None:
            return self.__motion_handler_hover.send_hover()
        return False

    def __leader_pose_callback(self, msg) -> None:
        """
        Leader pose callback.

        :param msg: Leader pose message
        :type msg: PoseStamped
        """
        # If distance between last queue element and new message is greater than threshold
        # append new message to queue
        if not self.__initialize:
            return
        if len(self.__leader_poses_queue) == 0:
            if self.__fist_leader_pose is None:
                self.__fist_leader_pose = msg
            distance_to_first = self.__distance(self.__fist_leader_pose.pose, msg.pose)
            if distance_to_first > self.__leader_position_sampling_distance:
                self.__leader_poses_queue.append(msg)
                return
            return
        last_pose = self.__leader_poses_queue[-1]
        distance = self.__distance(last_pose.pose, msg.pose)
        if distance > self.__leader_position_sampling_distance:
            self.__leader_poses_queue.append(msg)
            # Publish debug poses
            if self.__debug:
                self.__marker.header = msg.header
                self.__marker.pose = msg.pose
                self.__marker.id = len(self.__leader_poses_queue)
                self.__debug_poses_pub.publish(self.__marker)

    def __distance(self, pose1, pose2) -> float:
        """Calculate distance between two poses."""
        return sqrt(
            (pose1.position.x - pose2.position.x)**2 +
            (pose1.position.y - pose2.position.y)**2)

    def __get_twist_motion_reference(
            self,
            current_position,
            goal_position, max_speed) -> TwistStamped:
        """Get twist motion reference."""
        velocity_cmd = TwistStamped()
        velocity_cmd.header = max_speed.header

        velocity_vector = [
            goal_position[0] - current_position[0],
            goal_position[1] - current_position[1],
            goal_position[2] - current_position[2]]

        norm = sqrt(
            velocity_vector[0]**2 + velocity_vector[1]**2 + velocity_vector[2]**2)
        if norm == 0:
            return velocity_cmd

        velocity_vector = [
            velocity_vector[0] / norm,
            velocity_vector[1] / norm,
            velocity_vector[2] / norm]
        velocity_cmd.twist.linear.x = velocity_vector[0] * max_speed.twist.linear.x
        velocity_cmd.twist.linear.y = velocity_vector[1] * max_speed.twist.linear.y
        velocity_cmd.twist.linear.z = velocity_vector[2] * max_speed.twist.linear.z

        return velocity_cmd
