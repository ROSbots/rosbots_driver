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
from std_msgs.msg import UInt32, Float32


class WheelDrive:
    def __init__(self):
        rospy.Subscriber("/wheel_ticks_right", UInt32,
                         self.ticks_right_callback) 
        self.pub_velocity_right = \
            rospy.Publisher('/wheel_power_right', Float32, queue_size=10) 
        rospy.on_shutdown(self.shutdown_cb)
        self.wheel_power_right = Float32()
        self.wheel_power_right.data = 0.0

        self.ticks_right = None

        self.stop_wheel_ticks = 40 * 3
        
    def ticks_right_callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + " - ticks right: " + str(data.data))
        if self.ticks_right == None:
            self.ticks_right = UInt32()
            self.ticks_right.data = data.data
            rospy.loginfo(rospy.get_caller_id() + " INITIAL - ticks right: " + str(data.data))
            
        else:
            if self.wheel_power_right.data != 0.0 and \
               data.data >= self.ticks_right.data + self.stop_wheel_ticks:
                self.wheel_power_right.data = 0.0
                rospy.loginfo(rospy.get_caller_id() +
                              " STOPPING - power right: " +
                              str(self.wheel_power_right.data))
                self.pub_velocity_right.publish(self.wheel_power_right)
                rospy.loginfo(rospy.get_caller_id() +
                              " STOPPING - ticks right: " + str(data.data))
            elif data.data >= self.ticks_right.data + self.stop_wheel_ticks:
                self.ticks_right.data = data.data

    def run(self):
        self.wheel_power_right.data = -1.0
        self.pub_velocity_right.publish(self.wheel_power_right)

    def shutdown_cb(self):
        rospy.loginfo(rospy.get_caller_id() + " SHUTDOWN - ticks right: " + str(self.ticks_right.data))
        
        
    
def main():
    rospy.init_node('wheel_drive_01', anonymous=False)
    
    wheel_drive = WheelDrive()
    
    rate = rospy.Rate(10)

    once = 0
    while not rospy.is_shutdown():

        if once < 3:
            wheel_drive.run()
            once += 1
        
        #rospy.loginfo(rospy.get_caller_id() + ": Wake")
        
        rate.sleep()


if __name__ == '__main__':
    main()
