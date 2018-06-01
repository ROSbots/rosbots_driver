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

class Stop(Controller):
    def __init__(self):
        rospy.loginfo(rospy.get_caller_id() + " Stop initialized")
        
    def execute(self):
        output = {"v": 0, "w": 0}
        return output

    def shutdown(self):
        rospy.loginfo(rospy.get_caller_id() + " Stop shutdown")
