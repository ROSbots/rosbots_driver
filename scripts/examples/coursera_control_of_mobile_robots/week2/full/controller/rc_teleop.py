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

import rospy
from geometry_msgs.msg import Twist

from controller import Controller

class RCTeleop(Controller):
    def __init__(self):
        rospy.loginfo(rospy.get_caller_id() + " RCTeleop initialized")
        rospy.Subscriber(rospy.get_name() + "/cmd_vel", Twist, self.twist_cb)

        self.v = 0
        self.w = 0

    def execute(self):
        #rospy.loginfo(rospy.get_caller_id() + " RCTeleop execute")
        output = {"v": self.v, "w": self.w}
        return output

    def shutdown(self):
        rospy.loginfo(rospy.get_caller_id() + " RCTeleop shutdown")

    def twist_cb(self, data):
        rospy.loginfo(rospy.get_caller_id() + \
                      ": Linear.x: %f -- Angular.z: %f", \
                      data.linear.x, data.angular.z)
        self.v = data.linear.x
        self.w = data.angular.z
