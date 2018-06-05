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

from controller import Controller

from dynamics.pid import PID

class GoToGoal(Controller):
    def __init__(self, robot):
        rospy.loginfo(rospy.get_caller_id() + " GoToGoal initialized")

        # Goal location
        self.goal = {"x": -0.5, "y": -0.0}

        # PID controllers
        self.PID = {"v": PID(0.5, 0.05, 0.2),
                    "w": PID(0.5, 0.05, 0.2)}

        self.robot = robot

    def at_goal(self):
        pose2D = self.robot.get_pose2D()

        # Distance to goal
        u_x = self.goal["x"] - pose2D.x
        u_y = self.goal["y"] - pose2D.y
        d2_g = (u_x * u_x) + (u_y * u_y)

        # Are close to goal?
        close_to_goal = 0.05 
        d2_at_goal = (close_to_goal**2)
        
        if d2_g < d2_at_goal:
            return True
        return False
        

    def execute(self):
        output = {"v": 0, "w": 0}
        
        # Get robot pose
        pose2D = self.robot.get_pose2D()

        # Distance to goal
        u_x = self.goal["x"] - pose2D.x
        u_y = self.goal["y"] - pose2D.y
        d2_g = (u_x * u_x) + (u_y * u_y)

        # Are close to goal?
        d2_at_goal = (0.05**2)
        d2_far_away = (0.2**2) # 20cm
        if d2_g < d2_at_goal:
            # At the goal
            self.PID["v"].reset()
            self.PID["w"].reset()
            return output
        elif d2_g >= d2_far_away:
            output["v"] = 0.22
        else:
            # Not far, but not at goal
            output["v"] = 0.12

        # Compute angle to goal
        theta_g = math.atan2(u_y, u_x)

        # Difference between angle to goal and current heading
        theta_diff = theta_g - pose2D.theta
        d_y = math.sin(theta_diff)
        d_x = math.cos(theta_diff)
        theta_diff = math.atan2(d_y, d_x)

        rospy.loginfo(rospy.get_caller_id() +
                      " theta_g, diff, pose_theta: " +
                      str(math.degrees(theta_g)) + ", " +
                      str(math.degrees(theta_diff)) + ", " +
                      str(math.degrees(pose2D.theta)))

        # If angle is large, then we turn. Else we go straight
        theta_large = math.radians(15) # X degrees
        if abs(theta_diff) >= theta_large:
            output["v"] = 0.0
            output["w"] = self.PID["w"].output(theta_diff)
        else:
            # Angle is little, just go straight
            pass
        
        return output

    def shutdown(self):
        rospy.loginfo(rospy.get_caller_id() + " GoToGoal shutdown")
