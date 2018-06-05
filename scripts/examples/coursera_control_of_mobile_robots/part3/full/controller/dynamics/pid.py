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

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        # Accumulated error
        self.E_k = 0.0

        # Previous error
        self.e_k_1 = 0.0

    def reset(self):
        self.E_k = 0.0
        self.e_k_1 = 0.0

    def output(self, e_val):
        # Accumulate error
        self.E_k += e_val
        rval = (self.Kp * e_val) + (self.Ki * self.E_k) + \
               (self.Kd * (e_val - self.e_k_1))

        # Store away error 
        self.e_k_1 = e_val
        return rval
