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
from std_msgs.msg import Float32

class Robot:
    def __init__(self):
        # Diff drive robot attributes can be stored in parameter server
        # but otherwise a ROSbots dimensions are measured as the defaults
        # wheelbase of 140mm and wheel diameter of 70mm
        self.wheelbase = rospy.get_param("wheelbase", default=0.14)
        self.wheel_radius = rospy.get_param("wheel_radius", default=0.035)

        # Wheel min and max no-load velocities in radians per sec
        self.wheel_speed_min = rospy.get_param("wheel_speed/min", default=3.1)
        self.wheel_speed_mid = rospy.get_param("wheel_speed/mid", default=4.4)
        self.wheel_speed_max = rospy.get_param("wheel_speed/max", default=5.48)
        self.wheel_speed_min_power = \
            rospy.get_param("wheel_speed/min_power", default=0.5)
        self.wheel_speed_mid_power = \
            rospy.get_param("wheel_speed/mid_power", default=0.75)
        self.wheel_speed_max_power = \
            rospy.get_param("wheel_speed/max_power", default=1.0)

        # Publish out wheel power
        self.cur_wheel_power_right = Float32()
        self.cur_wheel_power_left = Float32()
        self.cur_wheel_power_right.data = 0.0
        self.cur_wheel_power_left.data = 0.0
        
        self.pub_power_right = \
            rospy.Publisher('/wheel_power_right', Float32, queue_size=10)
        self.pub_power_left = \
            rospy.Publisher('/wheel_power_left', Float32, queue_size=10)
        self.pub_power_right.publish(self.cur_wheel_power_right)
        self.pub_power_left.publish(self.cur_wheel_power_left)

        
    def shutdown(self):
        rospy.loginfo(rospy.get_caller_id() + " Robot shutdown")
        self.cur_wheel_power_right.data = 0.0
        self.cur_wheel_power_left.data = 0.0
        self.pub_power_right.publish(self.cur_wheel_power_right)
        self.pub_power_left.publish(self.cur_wheel_power_left)
        

    def velocity_to_power(self, v):
        av = abs(v)

        # If velocity is below minimum velocity turnable by PWM, then
        # just set to zero since the wheels won't spin anyway
        if av < self.wheel_speed_min:
            return 0.0

        a = b = a_pow = b_pow = None
        nnn = None
        if av >= self.wheel_speed_min and av < self.wheel_speed_mid:
            a = self.wheel_speed_min
            a_pow = self.wheel_speed_min_power
            b = self.wheel_speed_mid
            b_pow = self.wheel_speed_mid_power
        elif av >= self.wheel_speed_mid and av <= self.wheel_speed_max:
            a = self.wheel_speed_mid
            a_pow = self.wheel_speed_mid_power
            b = self.wheel_speed_max
            b_pow = self.wheel_speed_max_power
        
        # Linearly interpolate a and b
        nnn = ((av - a)/(b - a))
        wheel_power = ((nnn * (b_pow - a_pow)) + a_pow)

        if False:
            rospy.loginfo(rospy.get_caller_id() + ": " + str(a) + "," + str(b) +
                          "," + str(a_pow) + "," + str(b_pow))
            rospy.loginfo(rospy.get_caller_id() + " av: " + str(av))
            rospy.loginfo(rospy.get_caller_id() + " nnn: " + str(nnn))
            rospy.loginfo(rospy.get_caller_id() +
                          " wheel_power: " + str(wheel_power))

        assert(wheel_power <= 1.0)
        assert(wheel_power >= 0.0)

        # Negate if necessary
        if v < 0:
            wheel_power *= -1.0

        return wheel_power

    def set_wheel_speed(self, vr, vl):
        # Clamp the wheel speeds to actuator limits
        vr = max(min(vr, self.wheel_speed_max), self.wheel_speed_max * -1.0)
        vl = max(min(vl, self.wheel_speed_max), self.wheel_speed_max * -1.0)

        # Convert to power norms
        self.cur_wheel_power_right.data = self.velocity_to_power(vr)
        self.cur_wheel_power_left.data = self.velocity_to_power(vl)

        # Publish out
        if self.cur_wheel_power_right.data != 0.0 or \
           self.cur_wheel_power_left.data != 0.0:
            rospy.loginfo(rospy.get_caller_id() +
                          " right power: " + str(self.cur_wheel_power_right) +
                          " left power: " + str(self.cur_wheel_power_left))
        self.pub_power_right.publish(self.cur_wheel_power_right)
        self.pub_power_left.publish(self.cur_wheel_power_left)
        
        

    
        

        
