#!/usr/bin/env python

#
# This file is part of ROSbots ROS Drivers.
#
# Copyright
#
#     Copyright (C) 2017 Jack Pien <jack@rosbots.com>
#
# License
#
#     This program is free software: you can redistribute it and/or modify
#     it under the terms of the GNU Lesser General Public License as published
#     by the Free Software Foundation, either version 3 of the License, or
#     (at your option) any later version.
#
#     This program is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU Lesser General Public License for more details at
#     <http://www.gnu.org/licenses/lgpl-3.0-standalone.html>
#
# Documentation
#
#     http://www.rosbots.com
#

import math
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Pose2D

from robot import Robot
from rc_teleop import RCTeleop
from dynamics.differential_drive import DifferentialDrive

class Supervisor:
    def __init__(self):
        rospy.on_shutdown(self.shutdown_cb)

        self.controllers = {"rc": RCTeleop()}
        self.current_state = "rc"
        self.current_controller = self.controllers[self.current_state]

        self.robot = Robot()
        rospy.loginfo(rospy.get_caller_id() +
                      " wheelbase: " + str(self.robot.wheelbase) +
                      " wheel radius: " + str(self.robot.wheel_radius))
        
        self.dd = DifferentialDrive(self.robot.wheelbase,
                                    self.robot.wheel_radius)

        # Initialize previous wheel encoder ticks
        self.prev_wheel_ticks = None

    def execute(self):
        # Get commands in unicycle model
        ctrl_output = self.current_controller.execute()

        # Convert unicycle model commands to differential drive model
        diff_output = self.dd.uni_to_diff(ctrl_output["v"], ctrl_output["w"])

        if ctrl_output["v"] != 0.0 or ctrl_output["w"] != 0.0:
            rospy.loginfo(rospy.get_caller_id() + " v: " +
                          str(ctrl_output["v"]) +
                          " w: " + str(ctrl_output["w"]))
            rospy.loginfo(rospy.get_caller_id() + " vl: " +
                          str(diff_output["vl"]) +
                          " vr: " + str(diff_output["vr"]))
        
        # Set the wheel speeds
        self.robot.set_wheel_speed(diff_output["vr"], diff_output["vl"])

        self.update_odometry()
        
    def shutdown_cb(self):
        for ctrl in self.controllers.values():
            ctrl.shutdown()

        self.robot.shutdown()

    def update_odometry(self):
        # Get wheel encoder ticks
        ticks = self.robot.get_wheel_ticks()

        # Have not seen a wheel encoder message yet so no need to do anything
        if ticks["r"] == None or ticks["l"] == None:
            return

        # Robot may not start with encoder count at zero
        if self.prev_wheel_ticks == None:
            self.prev_wheel_ticks = {"r": ticks["r"], "l": ticks["l"]}
            rospy.loginfo(rospy.get_caller_id() + " initial r ticks: " +
                          str(ticks["r"]))
            rospy.loginfo(rospy.get_caller_id() + " initial l ticks: " +
                          str(ticks["l"]))
            
        
        # If ticks are the same since last time, then no need to update either
        if ticks["r"] == self.prev_wheel_ticks["r"] and \
           ticks["l"] == self.prev_wheel_ticks["l"]:
            return

        # Get current pose from robot
        prev_pose = self.robot.pose2D

        # Compute odometry - kinematics in meters
        R = self.robot.wheel_radius;
        L = self.robot.wheelbase;
        ticks_per_rev = self.robot.encoder_ticks_per_rev
        meters_per_tick = (2.0 * math.pi * R) / ticks_per_rev

        # How far did each wheel move
        meters_right = \
            meters_per_tick * (ticks["r"] - self.prev_wheel_ticks["r"])
        meters_left = \
            meters_per_tick * (ticks["l"] - self.prev_wheel_ticks["l"])
        meters_center = (meters_right + meters_left) * 0.5

        # Compute new pose
        x_dt = meters_center * math.cos(prev_pose.theta);
        y_dt = meters_center * math.sin(prev_pose.theta);
        theta_dt = (meters_right - meters_left) / L;
            
        new_pose = Pose2D(0.0, 0.0, 0.0)
        new_pose.x = prev_pose.x + x_dt
        new_pose.y = prev_pose.y + y_dt
        new_pose.theta = prev_pose.theta + theta_dt

        if True:
            rospy.loginfo(rospy.get_caller_id() + " prev r ticks: " +
                          str(self.prev_wheel_ticks["r"]))
            rospy.loginfo(rospy.get_caller_id() + " prev l ticks: " +
                          str(self.prev_wheel_ticks["l"]))
            rospy.loginfo(rospy.get_caller_id() + " r ticks: " +
                          str(ticks["r"]))
            rospy.loginfo(rospy.get_caller_id() + " l ticks: " +
                          str(ticks["l"]))

            rospy.loginfo(rospy.get_caller_id() + " x: " +
                          str(new_pose.x))
            rospy.loginfo(rospy.get_caller_id() + " y: " +
                          str(new_pose.y))
            rospy.loginfo(rospy.get_caller_id() + " theta: " +
                          str(new_pose.theta))

        # Update robot with new pose
        self.robot.pose2D = new_pose

        # Update the tick count
        self.prev_wheel_ticks["r"] = ticks["r"]
        self.prev_wheel_ticks["l"] = ticks["l"]

        # Broadcast pose as ROS tf
        """
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "rosbots_robot"
        t.transform.translation.x = new_pose.x
        t.transform.translation.y = new_pose.y
        t.transform.translation.z = 0.0
        q = tf.transformations.quaternion_from_euler(0, 0, new_pose.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)
        """

        
