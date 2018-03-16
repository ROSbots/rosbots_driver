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
# This module pulses the power to a 5v power bank using a 2222 NPN transistor
# to prevent the power bank from shutting down due to inactivity. This allows
# us to use the power bank to power the wheels in place of the 4xAA battery
# pack. But since the wheels are not powered all the time, without this pulse
# module, the power bank will just shut itself off.
#

import rospy

class Controller:
    def __init__(self):
        pass

    def execute(self):
        pass

    def shutdown(self):
        pass
