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

from robot import Robot
from rc_teleop import RCTeleop
from dynamics.differential_drive import DifferentialDrive

class Supervisor:
    def __init__(self):
        rospy.on_shutdown(self.shutdown_cb)

        self.controllers = {"rc": RCTeleop()}
        self.current_state = "rc"
        self.current_controller = self.controllers[self.current_state]

        self.robot = Robot()
        rospy.loginfo(rospy.get_caller_id() +
                      " wheelbase: " + str(self.robot.wheelbase) +
                      " wheel radius: " + str(self.robot.wheel_radius))
        
        self.dd = DifferentialDrive(self.robot.wheelbase,
                                    self.robot.wheel_radius)

    def execute(self):
        # Get commands in unicycle model
        ctrl_output = self.current_controller.execute()
        rospy.loginfo(rospy.get_caller_id() + " v: " + str(ctrl_output["v"]) +
                      " w: " + str(ctrl_output["w"]))

        # Convert unicycle model commands to differential drive model
        diff_output = self.dd.uni_to_diff(ctrl_output["v"], ctrl_output["w"])
        rospy.loginfo(rospy.get_caller_id() + " vl: " + str(diff_output["vl"]) +
                      " vr: " + str(diff_output["vr"]))
        
        # Set the wheel speeds
        self.robot.set_wheel_speed(diff_output["vr"], diff_output["vl"])
        
    def shutdown_cb(self):
        for ctrl in self.controllers.values():
            ctrl.shutdown()

        self.robot.shutdown()

        
