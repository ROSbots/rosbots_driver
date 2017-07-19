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

import math
#import RPIO as GPIO 
import RPi.GPIO as GPIO

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3Stamped

class PulsePower:
    
    # Broadcom pin outs
    # https://www.element14.com/community/servlet/JiveServlet/previewBody/73950-102-10-339300/pi3_gpio.png
    pulse_pin = 16

    def __init__(self):
        rospy.init_node('pulse_power', anonymous=True)        
        node_name = rospy.get_name()
        rospy.on_shutdown(self.shutdown_cb)

        
        GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
        
        GPIO.setup(self.pulse_pin, GPIO.OUT)

    def shutdown_cb(self):

        rospy.loginfo(rospy.get_caller_id() + ": Shutdown callback")
    
        GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
        self.pulse(GPIO.LOW)
        
        GPIO.cleanup()

    def pulse(self, sig):
        GPIO.output(self.pulse_pin, sig)
        
def main():
    ppower = PulsePower()

    pulse_hz = rospy.get_param('~pulse_hertz', 0.05)
    pulse_width_sec = rospy.get_param('~pulse_width_sec', 1.2)
    
    rate = rospy.Rate(pulse_hz) 

    while not rospy.is_shutdown():
        ppower.pulse(GPIO.HIGH)
        rospy.sleep(pulse_width_sec)
        ppower.pulse(GPIO.LOW)
        
        rate.sleep()


if __name__ == '__main__':
    main()
