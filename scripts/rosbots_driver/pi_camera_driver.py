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
import time
import io
import numpy as np
import cv2
import rospy
from picamera.array import PiRGBArray
from picamera import PiCamera
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

def shutdown_cb():
    pass

class ImgContainer(object):

    def log(self, msg):
        if False:
            rospy.loginfo(msg)
    
    def get_image_msg(self):
        # Clear stream
        self._compress_stream.seek(0)
        self._compress_stream.truncate()
        
        grab_time = rospy.Time.now()
        self._picamera.capture(self._compress_stream, format="jpeg",
                               use_video_port=True)
        dur = rospy.Time.now() - grab_time
        img_ts = grab_time + (dur * 0.5)

        stream = self._compress_stream
        stream.seek(0)
        stream_data = stream.getvalue()
        
        # Generate compressed image
        image_msg = CompressedImage()
        image_msg.format = "jpeg"
        image_msg.data = stream_data
        
        image_msg.header.stamp = img_ts
        image_msg.header.frame_id = "0"
        return image_msg
        
    def __init__(self):        
        self._picamera = PiCamera()
        self._picamera.resolution = (640,480)
        self._picamera.vflip = False 
        self._picamera.hflip = False 
        self._picamera.framerate = 60
        self._compress_stream = io.BytesIO()
        
    def get_cv_image_compress(self):
        stream = self._compress_stream

def main():
    rospy.init_node('camera', anonymous=False)
    
    rospy.loginfo(rospy.get_caller_id() + " OpenCV version %s",
                  cv2.__version__)    
    rospy.on_shutdown(shutdown_cb)
    
    image_pub = rospy.Publisher("~image/compressed",
                                CompressedImage, queue_size=5)
    img_ct = ImgContainer()
    
    prev_connections = 0
    rate = rospy.Rate(15) # 5 Hz
    while not rospy.is_shutdown():
        img_compress_msg = img_ct.get_image_msg()
        image_pub.publish(img_compress_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()    
    except rospy.ROSInterruptException:
        pass
