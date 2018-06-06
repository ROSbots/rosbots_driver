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
import numpy as np
import cv2 as cv

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
 
# CV Bridge Python does not support compressed image
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError

class FollowFace():
    def __init__(self):
        rospy.init_node('follow_face', anonymous=False)

        rospy.loginfo(rospy.get_caller_id() + " FollowFace initialized")

        self.image_sub = rospy.Subscriber("/camera/image/compressed",
                                          CompressedImage, self.image_cb)

        home_path = '/home/jpien/local/gitspace'
        haar_folder = home_path + '/opencv/data/haarcascades'
        haar_file = haar_folder + '/haarcascade_frontalface_default.xml'
        
        self.face_cascade = cv.CascadeClassifier(haar_file)

        self.twist_pub = rospy.Publisher('/rosbots_robot/cmd_vel', Twist,
                                         queue_size=1)

        self.face_found_lock = allocate_lock()
        self.face_found = None
        self.stop_msg = Twist()

    def image_cb(self, in_msg):
        
        np_arr = np.fromstring(in_msg.data, np.uint8)
        image_np = cv.imdecode(np_arr, cv.IMREAD_COLOR)
        gray = cv.cvtColor(image_np, cv.COLOR_BGR2GRAY)
        height, width = gray.shape[:2]
        
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        self.face_found_lock.acquire()
        self.face_found = None
        for (x,y,w,h) in faces:
            #cv.rectangle(image_np,(x,y),(x+w,y+h),(255,0,0),2)
            xc = x + (w*0.5)
            xc_norm = xc / (float)(width)
            xc_norm -= 0.5
            self.face_found = {"x": xc_norm}
            rospy.loginfo("xc: " + str(xc_norm))
            break
            #roi_gray = gray[y:y+h, x:x+w]
            #roi_color = img[y:y+h, x:x+w]
            #eyes = eye_cascade.detectMultiScale(roi_gray)
            #for (ex,ey,ew,eh) in eyes:
            #    cv.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
            #rospy.loginfo(rospy.get_caller_id() + " Face: " +
            #              str(x) + ", " + str(y))
        self.face_found_lock.release()
        #cv.imshow('cv_img', image_np)
        #cv.waitKey(2)
    
    def run(self):
        twist_msg = Twist()

        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.prev_dir = "stop"
        cur_dir = "stop"
        
        self.face_found_lock.acquire()
        if self.face_found != None:
            if abs(self.face_found["x"]) < 0.15:
                # In front
                cur_dir = "fwd"
                twist_msg.linear.x = 0.15
                twist_msg.angular.z = 0.0
            elif self.face_found["x"] < 0.0:
                # To the left
                cur_dir = "left"
                twist_msg.linear.x = 0.15
                twist_msg.angular.z = 2.0
            else:
                # To the right
                cur_dir = "right"
                twist_msg.linear.x = 0.15
                twist_msg.angular.z = -2.0
        self.face_found_lock.release()

        if self.prev_dir != cur_dir:
            self.twist_pub.publish(self.stop_msg)
        self.twist_pub.publish(twist_msg)
        self.prev_dir = cur_dir

    def shutdown(self):
        rospy.loginfo(rospy.get_caller_id() + " FollowFace shutdown")
        
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.twist_pub.publish(twist_msg)
        
        cv.destroyAllWindows()


if __name__ == "__main__":
    ff = FollowFace()
    
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ff.run()
        rate.sleep()
        
    ff.shutdown()
