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
from geometry_msgs.msg import Pose2D, TransformStamped

from robot import Robot
from rc_teleop import RCTeleop
from go_to_goal import GoToGoal
from stop import Stop
from dynamics.differential_drive import DifferentialDrive

class Supervisor:
    def __init__(self):
        rospy.on_shutdown(self.shutdown_cb)

        self.robot_name = rospy.get_name()
        self.robot = Robot()
        
        self.controllers = {"rc": RCTeleop(),
                            "gtg": GoToGoal(self.robot),
                            "stop": Stop()}
        self.switch_to_state("gtg")
        
        self.dd = DifferentialDrive(self.robot.wheelbase,
                                    self.robot.wheel_radius)

        # Initialize TF Broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        # Initialize previous wheel encoder ticks
        self.prev_wheel_ticks = None

    def switch_to_state(self, state):
        self.current_state = state
        self.current_controller = self.controllers[self.current_state]

    def execute(self):
        # Check events
        if self.current_state == "gtg" and self.current_controller.at_goal():
            self.switch_to_state("stop")
                        
        # Get commands in unicycle model
        ctrl_output = self.current_controller.execute()

        # Convert unicycle model commands to differential drive model
        diff_output = self.dd.uni_to_diff(ctrl_output["v"], ctrl_output["w"])

        if ctrl_output["v"] != 0.0 or ctrl_output["w"] != 0.0:
            rospy.loginfo(rospy.get_caller_id() +
                          " v: " + str(ctrl_output["v"]) +
                          " w: " + str(ctrl_output["w"]))
            rospy.loginfo(rospy.get_caller_id() +
                          " vl: " + str(diff_output["vl"]) +
                          " vr: " + str(diff_output["vr"]))
        
        # Set the wheel speeds
        self.robot.set_wheel_speed(diff_output["vr"], diff_output["vl"])

        self.update_odometry()

        self.publish_pose()
        
    def shutdown_cb(self):
        rospy.loginfo(rospy.get_caller_id() +
                      " current controller: " + self.current_state)
        self.switch_to_state("stop")
        
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
            rospy.loginfo(rospy.get_caller_id() + " initial l / r  ticks: " +
                          str(ticks["l"]) + " / " + str(ticks["r"]))
            
        
        # If ticks are the same since last time, then no need to update either
        if ticks["r"] == self.prev_wheel_ticks["r"] and \
           ticks["l"] == self.prev_wheel_ticks["l"]:
            return

        # Get current pose from robot
        prev_pose = self.robot.get_pose2D()

        # Compute odometry - kinematics in meters
        R = self.robot.wheel_radius;
        L = self.robot.wheelbase;
        ticks_per_rev = self.robot.encoder_ticks_per_rev
        meters_per_tick = (2.0 * math.pi * R) / ticks_per_rev

        # How far did each wheel move
        wheel_dir = self.robot.get_wheel_dir()
        meters_right = \
            meters_per_tick * (ticks["r"] - self.prev_wheel_ticks["r"]) * \
            wheel_dir["r"]
        meters_left = \
            meters_per_tick * (ticks["l"] - self.prev_wheel_ticks["l"]) * \
            wheel_dir["l"]
        meters_center = (meters_right + meters_left) * 0.5

        # Compute new pose
        x_dt = meters_center * math.cos(prev_pose.theta);
        y_dt = meters_center * math.sin(prev_pose.theta);
        theta_dt = (meters_right - meters_left) / L;
            
        new_pose = Pose2D(0.0, 0.0, 0.0)
        new_pose.x = prev_pose.x + x_dt
        new_pose.y = prev_pose.y + y_dt
        theta_tmp = prev_pose.theta + theta_dt
        new_pose.theta = math.atan2( math.cos(theta_tmp), math.sin(theta_tmp) )

        if True:
            rospy.loginfo(rospy.get_caller_id() + " prev l / r ticks: " +
                          str(self.prev_wheel_ticks["l"]) + " / " +
                          str(self.prev_wheel_ticks["r"]))
            rospy.loginfo(rospy.get_caller_id() + " l / r ticks: " +
                          str(ticks["l"]) + " / " + str(ticks["r"]))
            rospy.loginfo(rospy.get_caller_id() +
                          " meters left / meters right: " +
                          str(meters_left) + " / " + str(meters_right))

            rospy.loginfo(rospy.get_caller_id() + " x, y: " +
                          str(new_pose.x) + ", " + str(new_pose.y))
            rospy.loginfo(rospy.get_caller_id() + " theta (deg): " +
                          str(math.degrees(new_pose.theta)))

        # Update robot with new pose
        self.robot.set_pose2D(new_pose)

        # Update the tick count
        self.prev_wheel_ticks["r"] = ticks["r"]
        self.prev_wheel_ticks["l"] = ticks["l"]

    def publish_pose(self):
        # Broadcast pose as ROS tf
        ppp = self.robot.get_pose2D()
        t = TransformStamped()
        t.header.frame_id = "world"
        t.child_frame_id = self.robot_name
        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = ppp.x
        t.transform.translation.y = ppp.y
        t.transform.translation.z = 0.0
        q = tf.transformations.quaternion_from_euler(0, 0, ppp.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)

        
