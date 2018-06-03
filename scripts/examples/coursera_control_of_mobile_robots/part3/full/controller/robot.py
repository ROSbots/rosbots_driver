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
from thread import allocate_lock
import rospy
from std_msgs.msg import Float32, UInt32
from geometry_msgs.msg import Pose2D

class Robot:
    def __init__(self):
        # Diff drive robot attributes can be stored in parameter server
        # but otherwise a ROSbots dimensions are measured as the defaults
        # wheelbase of 140mm and wheel diameter of 70mm
        self.wheelbase = rospy.get_param("wheelbase", default=0.14)
        self.wheel_radius = rospy.get_param("wheel_radius", default=0.035)

        # Encoder disk ticks per revolution
        self.encoder_ticks_per_rev = \
            rospy.get_param("encoder_ticks_per_rev", default=40)
        self.meters_per_tick = \
            ((math.pi * 2.0 * self.wheel_radius) /
             (float)(self.encoder_ticks_per_rev))

        # Wheel min and max no-load velocities in meters per sec
        self.wheel_speed_min = rospy.get_param("wheel_speed/min", default=0.0)
        self.wheel_speed_mid = rospy.get_param("wheel_speed/mid", default=0.1)
        self.wheel_speed_max = rospy.get_param("wheel_speed/max", default=0.2)
        self.wheel_speed_min_power = \
            rospy.get_param("wheel_speed/min_power", default=0.0)
        self.wheel_speed_mid_power = \
            rospy.get_param("wheel_speed/mid_power", default=0.5)
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

        # Subscribe to wheel encoder ticks
        self.wheel_ticks_right_lock = allocate_lock()
        self.wheel_ticks_left_lock = allocate_lock()
        self.sub_wheel_ticks_right = \
            rospy.Subscriber("/wheel_ticks_right", UInt32, self.wheel_ticks_cb,
                             (True))
        self.sub_wheel_ticks_left = \
            rospy.Subscriber("/wheel_ticks_left", UInt32, self.wheel_ticks_cb,
                             (False))
        self._cur_wheel_ticks_right = None
        self._cur_wheel_ticks_left = None
        self._cur_wheel_ticks_ts = None
        self._prev_wheel_ticks = {"r" : None, "l" : None, "ts": None}
        self._wheel_velocity = {"r": 0.0, "l": 0.0}

        # Current robot pose
        self.pose2D = Pose2D(0.0, 0.0, 0.0)

        # PID variables - E_k is accumulated error, e_k_1 is previous error,
        # K are the gains.
        self.PID = {"E_k": {"r": 0.0, "l": 0.0},
                    "e_k_1": {"r": 0.0, "l": 0.0},
                    "Kp": 1.0, "Ki": 0.1, "Kd": 0.2}
        
    def shutdown(self):
        rospy.loginfo(rospy.get_caller_id() + " Robot shutdown")

        # Send stop command twice just in case
        for i in range(2):
            self.cur_wheel_power_right.data = 0.0
            self.cur_wheel_power_left.data = 0.0
            self.pub_power_right.publish(self.cur_wheel_power_right)
            self.pub_power_left.publish(self.cur_wheel_power_left)
            rospy.sleep(0.2)

    def get_pose2D(self):
        return self.pose2D

    def set_pose2D(self, pose2D):
        self.pose2D.x = pose2D.x
        self.pose2D.y = pose2D.y
        self.pose2D.theta = pose2D.theta
        
    def get_wheel_ticks(self):
        ticks = {}
        self.wheel_ticks_right_lock.acquire()
        self.wheel_ticks_left_lock.acquire()
        ticks["r"] = self._cur_wheel_ticks_right
        ticks["l"] = self._cur_wheel_ticks_left
        ticks["ts"] = self._cur_wheel_ticks_ts
        self.wheel_ticks_right_lock.release()
        self.wheel_ticks_left_lock.release()
        return ticks

    def get_wheel_dir(self):
        wheel_dir = {"r": 1.0, "l": 1.0}
        if self.cur_wheel_power_left.data < 0.0:
            wheel_dir["l"] = -1.0

        if self.cur_wheel_power_right.data < 0.0:
            wheel_dir["r"] = -1.0

        return wheel_dir

    def wheel_ticks_cb(self, ticks, is_right_wheel):
        if is_right_wheel:
            self.wheel_ticks_right_lock.acquire()
            self._cur_wheel_ticks_ts = rospy.Time.now()
            self._cur_wheel_ticks_right = ticks.data
            self.wheel_ticks_right_lock.release()
        else:
            self.wheel_ticks_left_lock.acquire()
            self._cur_wheel_ticks_ts = rospy.Time.now()
            self._cur_wheel_ticks_left = ticks.data
            self.wheel_ticks_left_lock.release()
        
            
            
    def velocity_to_power(self, v):
        # Clamp the velocity again to max fwd and back
        v = max(min(v, self.wheel_speed_max), self.wheel_speed_max * -1.0)
        av = abs(v)

        a = b = a_pow = b_pow = None
        nnn = None
        if av >= 0.0 and av < self.wheel_speed_mid:
            a = 0.0
            a_pow = 0.0
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

        cur_ticks = self.get_wheel_ticks()
        
        # Special stop case
        if vr == 0.0 and vl == 0.0:
            for ddd in ["l", "r"]:
                self.PID["E_k"][ddd] = 0.0
                self.PID["e_k_1"][ddd] = 0.0
                self._wheel_velocity[ddd] = 0.0
            self.cur_wheel_power_right.data = 0.0
            self.cur_wheel_power_left.data = 0.0
            self._prev_wheel_ticks["r"] = cur_ticks["r"]
            self._prev_wheel_ticks["l"] = cur_ticks["l"]
            self._prev_wheel_ticks["ts"] = cur_ticks["ts"]
        else:
            # What direction do we want to go in?
            v_dir = {"l": 1.0, "r": 1.0} 
            for ddd in ["l", "r"]:
                if (vl < 0.0 and ddd == "l") or \
                   (vr < 0.0 and ddd == "r"):
                    v_dir[ddd] = -1.0

            # If we are changing direction, let's first stop the robot
            # This is because our encoders can't tell what direction the wheels
            # are turning
            #if v_dir["l"] * self.cur_wheel_power_left.data < 0.0 or \
            #   v_dir["r"] * self.cur_wheel_power_right.data < 0.0:
            #    # RECURSION!!!
            #    self.set_wheel_speed(0.0, 0.0)

            # Get actual direction motors are turning in
            motor_dir = self.get_wheel_dir()
            
            # Compute velocity of wheels
            if self._prev_wheel_ticks["r"] != None and \
               self._prev_wheel_ticks["l"] != None and \
                self._prev_wheel_ticks["ts"] != None:
                tick_dur = cur_ticks["ts"] - self._prev_wheel_ticks["ts"]
                rospy.loginfo(rospy.get_caller_id() +
                              " tick_dur: " + \
                              str(tick_dur))
                inv_sec = 0.0
                if tick_dur.nsecs != 0:
                    inv_sec = 1000000000.0 / (float)(tick_dur.nsecs)
                rospy.loginfo(rospy.get_caller_id() +
                              " inv_sec: " + \
                              str(inv_sec))
                for ddd in ["l", "r"]:
                    self._wheel_velocity[ddd] = \
                        (float)(cur_ticks[ddd] -
                                self._prev_wheel_ticks[ddd]) * \
                                inv_sec * self.meters_per_tick * motor_dir[ddd]
            self._prev_wheel_ticks["r"] = cur_ticks["r"]
            self._prev_wheel_ticks["l"] = cur_ticks["l"]
            self._prev_wheel_ticks["ts"] = cur_ticks["ts"]

            # PID the 2 wheel velocities
            e_wheel_pow_final = {"r": 0.0, "l": 0.0}
            e_pow = {"r": 0.0, "l": 0.0}
            e_pow["r"] = self.velocity_to_power(vr - self._wheel_velocity["r"])
            e_pow["l"] = self.velocity_to_power(vl - self._wheel_velocity["l"])
            for ddd in ["l", "r"]:
                # Accumulate error
                self.PID["E_k"][ddd] += e_pow[ddd]
                e_wheel_pow_final[ddd] = self.PID["Kp"] * e_pow[ddd] + \
                        self.PID["Ki"] * self.PID["E_k"][ddd] + \
                        self.PID["Kd"] * (e_pow[ddd] - self.PID["e_k_1"][ddd])

                # Store away previous error
                self.PID["e_k_1"][ddd] = e_pow[ddd]

            for ddd in ["l", "r"]:
                vvv = vl
                if ddd == "r":
                    vvv = vr
                rospy.loginfo(rospy.get_caller_id() +
                              " " + ddd + ", wheel_vel, e_pow, e_wheel_pow_final: " +
                              str(vvv) + ", " +
                              str(self._wheel_velocity[ddd]) +  ", " +
                              str(e_pow[ddd]) +  ", " +
                              str(e_wheel_pow_final[ddd]))
        
            # Add to current power settings and then clamp to -1 and 1
            # TTD: Only clamp to forward rotation for now!
            self.cur_wheel_power_right.data = \
                max(min(self.cur_wheel_power_right.data +
                        e_wheel_pow_final["r"],1.0),0.0) #-1.0)
            self.cur_wheel_power_left.data =  \
                max(min(self.cur_wheel_power_left.data +
                        e_wheel_pow_final["l"],1.0),0.0) #-1.0)
        
        # Publish out
        if self.cur_wheel_power_right.data != 0.0 or \
           self.cur_wheel_power_left.data != 0.0:
            rospy.loginfo(rospy.get_caller_id() +
                          " left velocity: " + \
                          str(self._wheel_velocity["l"]) +
                          " right velocity: " + \
                          str(self._wheel_velocity["r"]))
            rospy.loginfo(rospy.get_caller_id() +
                          " left power: " + \
                          str(self.cur_wheel_power_left) +
                          " right power: " + \
                          str(self.cur_wheel_power_right))
        self.pub_power_right.publish(self.cur_wheel_power_right)
        self.pub_power_left.publish(self.cur_wheel_power_left)

        
