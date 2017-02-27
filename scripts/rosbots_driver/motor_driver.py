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
#import RPIO as GPIO 
import RPi.GPIO as GPIO
from RPIO import PWM

import RPIO

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3Stamped

class MotorDriverL9110S:
    
    # Broadcom pin outs
    # https://www.element14.com/community/servlet/JiveServlet/previewBody/73950-102-10-339300/pi3_gpio.png
    left_ia = 23
    left_ib = 24
    right_ia = 20
    right_ib = 21

    encoder_right = 22
    encoder_left = 17
    
    pwm_subcycle_time_us = 20000 # 20ms cycle for PWM
    pwm_max_width = 20000
    pwm_granularity = 10

    def __init__(self):
        rospy.init_node('motor_driver', anonymous=True)        
        node_name = rospy.get_name()

        rospy.Subscriber(node_name + "/cmd_vel", Twist, self.twist_callback)    

        rospy.on_shutdown(self.shutdown_cb)
        
        # Get wheel circumference
        self._wheel_diam = rospy.get_param('~wheel_diameter_mm', 70.0)
        self._wheel_circ = self._wheel_diam * math.pi

        self._wheel_encoder_ticks_per_rotation = \
            rospy.get_param('~wheel_encoder_ticks_per_rotation', 20.0)
        self._wheel_arc_mm_per_tick = \
            self._wheel_circ / float(self._wheel_encoder_ticks_per_rotation)

        
        GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme
        GPIO.cleanup()
        
        GPIO.setup(self.left_ib, GPIO.OUT)
        GPIO.setup(self.right_ib, GPIO.OUT)
        
        GPIO.setup(self.encoder_right, GPIO.IN) # Right
        GPIO.setup(self.encoder_left, GPIO.IN) # Left
        
        self._servo = PWM.Servo(subcycle_time_us=self.pwm_subcycle_time_us)
        self._servo.set_servo(self.left_ia, 0) 
        self._servo.set_servo(self.right_ia, 0) 
        
        GPIO.output(self.left_ib, GPIO.LOW)
        GPIO.output(self.right_ib, GPIO.LOW)
                
        # Two GPIO interrupt callbacks for encoder
        self._encoder_count_left = 0
        self._encoder_count_right = 0
        self._encoder_last_publish_ticks_l = 0
        self._encoder_last_publish_ticks_r = 0
        self._encoder_last_publish_ts = rospy.Time.now()
        self._pub_velocity_left = \
            rospy.Publisher(node_name + '/mm_per_sec_left', \
                            Vector3Stamped, queue_size=10) 
        self._pub_velocity_right = \
            rospy.Publisher(node_name + '/mm_per_sec_right', \
                            Vector3Stamped, queue_size=10) 
        RPIO.setup(self.encoder_right, RPIO.IN)
        RPIO.add_interrupt_callback(self.encoder_right,
                                    self.encoder_callback_right, edge='rising',
                                    debounce_timeout_ms=10,
                                    pull_up_down=RPIO.PUD_DOWN,
                                    threaded_callback=True)
        RPIO.setup(self.encoder_left, RPIO.IN)
        RPIO.add_interrupt_callback(self.encoder_left,
                                    self.encoder_callback_left, edge='rising',
                                    debounce_timeout_ms=10,
                                    pull_up_down=RPIO.PUD_DOWN,
                                    threaded_callback=True)
        self.publish_velocity()
        
        # Starts waiting for interrupts 
        RPIO.wait_for_interrupts(threaded=True)

    def shutdown_cb(self):

        rospy.loginfo(rospy.get_caller_id() + ": Shutdown callback")
    
        GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme

        self._servo.stop_servo(self.left_ia)
        self._servo.stop_servo(self.right_ia)

        GPIO.output(self.left_ib, GPIO.LOW)
        GPIO.output(self.right_ib, GPIO.LOW)
        GPIO.cleanup()
        RPIO.cleanup()
            
            

    def twist_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + \
                      ": Linear.x: %f -- Angular.z: %f", \
                      data.linear.x, data.angular.z)
        x_dir = max(-1, min(1, data.linear.x))
        z_ang = max(-1, min(1, data.angular.z))

        lw = x_dir
        rw = x_dir

        if z_ang != 0:
            # Left wheel faster than right
            lw -= z_ang
            rw += z_ang

        lw = max(-1, min(1, lw))
        rw = max(-1, min(1, rw))

        rospy.loginfo(rospy.get_caller_id() + ": lw: %f -- rw: %f", lw, rw)

        if lw == 0:
            self._servo.set_servo(self.left_ia, 0) 
            GPIO.output(self.left_ib, GPIO.LOW)
        else:
            if lw > 0:
                pw = self.pwm_max_width * lw
                GPIO.output(self.left_ib, GPIO.LOW)
            else:
                pw = self.pwm_max_width - (self.pwm_max_width * (lw*-1))
                GPIO.output(self.left_ib, GPIO.HIGH)
            
            pw = int(pw/self.pwm_granularity) * self.pwm_granularity
            self._servo.set_servo(self.left_ia, pw) 
        

        if rw == 0:
            self._servo.set_servo(self.right_ia, 0) 
            GPIO.output(self.right_ib, GPIO.LOW)
        else:
            if rw > 0:
                pw = self.pwm_max_width * rw
                GPIO.output(self.right_ib, GPIO.LOW)
            else:
                pw = self.pwm_max_width - (self.pwm_max_width * (rw*-1))
                GPIO.output(self.right_ib, GPIO.HIGH)
            
            pw = int(pw/self.pwm_granularity) * self.pwm_granularity
            self._servo.set_servo(self.right_ia, pw) 

    def encoder_callback_left(self, gpio_id, val):
        self._encoder_count_left += 1

    def encoder_callback_right(self, gpio_id, val):
        self._encoder_count_right += 1

    def publish_velocity(self):
        # To get mm per second, we take number of ticks since last publish
        # multiply that by mm per tick, then divide by the time elapse
        # since timestamp of last publish
        ticks_l = self._encoder_count_left
        ticks_r = self._encoder_count_right
        cur_ts = rospy.Time.now()
        prev_ts = self._encoder_last_publish_ts
        dur_ts = cur_ts - prev_ts
        delta_ticks_l = ticks_l - self._encoder_last_publish_ticks_l
        delta_ticks_r = ticks_r - self._encoder_last_publish_ticks_r

        rospy.loginfo("delta R: " + str(delta_ticks_r))
        
        mm_traveled_left = delta_ticks_l * self._wheel_arc_mm_per_tick
        mm_traveled_right = delta_ticks_r * self._wheel_arc_mm_per_tick
        
        mm_per_sec_left = \
            (mm_traveled_left / (dur_ts.to_nsec()*0.000001)) * 1000.0
        mm_per_sec_right = \
            (mm_traveled_right / (dur_ts.to_nsec()*0.000001)) * 1000.0
        
        v3 = Vector3Stamped()
        v3.header.stamp = cur_ts
        v3.vector.x = mm_per_sec_left
        
        self._pub_velocity_left.publish(v3)

        v3.vector.x = mm_per_sec_right
        self._pub_velocity_right.publish(v3)

        # Update last publish
        self._encoder_last_publish_ts = cur_ts        
        self._encoder_last_publish_ticks_l = ticks_l
        self._encoder_last_publish_ticks_r = ticks_r
        
def main():
    mdriver = MotorDriverL9110S()

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        mdriver.publish_velocity()
        
        rate.sleep()


if __name__ == '__main__':
    main()
