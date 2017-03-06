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
import requests
import numpy as np
import cv2
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def shutdown_cb():
    pass

class ImgContainer(object):
    SRC_PICAMERA=1
    SRC_932L=2
    SRC_932L_MJPEG=3

    _picamera = None
    _piraw_capture = None

    @classmethod
    def init_picamera(cls):
        if cls._picamera == None:
            from picamera.array import PiRGBArray
            from picamera import PiCamera
            cls._picamera = PiCamera()
            cls._picamera.resolution = (640,480)
            cls._picamera.vflip = True
            cls._picamera.hflip = True
            cls._picamera.framerate = 16
            cls._piraw_capture = PiRGBArray(cls._picamera)

    def log(self, msg):
        if False:
            rospy.loginfo(msg)

    def _get_image_url(self, cam_url, username, password):
        self.log("Making camera request")
        r = requests.get(cam_url, auth=(username, password), timeout=1)
        self.log("Converting to byte array")
        arr = np.asarray(bytearray(r.content), dtype=np.uint8)
        self.log("Converting to CV Mat")
        self._cv_img_orig = cv2.imdecode(arr, -1) #cv2.CV_LOAD_IMAGE_GRAYSCALE)
        self._cv_img = cv2.GaussianBlur(self._cv_img_orig, (5,5), 0)
        self.log("Done creating image container")
        
    def get_image_932l(self):
        username = self._932l_uname #"admin"
        password = self._932l_pwd #""
        cam_url = "http://" + self._932l_ip + "/image/jpeg.cgi"
            
        self._get_image_url(cam_url, username, password)

    def _get_image_932l_mjpeg_ffwd(self):
        # Grab a whole bunch of frames till we're up to date
        for idx in range(0, 200):
            grab_time = rospy.Time.now()
            self._vid_capture.grab()
            dur = rospy.Time.now() - grab_time
            #rospy.loginfo("Time to grab: " + str(dur.to_sec()))
            if dur.to_sec() > 0.001:
                #rospy.loginfo("Retrieving frame")
                break;
            
    def get_image_932l_mjpeg(self):
        self._get_image_932l_mjpeg_ffwd()
        
        ret, self._cv_img_orig = self._vid_capture.retrieve()
        self._cv_img = cv2.GaussianBlur(self._cv_img_orig, (5,5), 0)
        
        
    def get_image_picamera(self):
        self._piraw_capture.truncate(0)
        self._picamera.capture(self._piraw_capture, format="bgr", \
                               use_video_port=True)
        self._cv_img_orig = self._piraw_capture.array
        self._cv_img = cv2.GaussianBlur(self._cv_img_orig, (5,5), 0)

    def get_image_filesystem(self, fn):
        self._cv_img_orig = cv2.imread(fn) #, flags=0)
        self._cv_img = cv2.GaussianBlur(self._cv_img_orig, (5,5), 0)

    def get_image(self):
        try:
            if self._source == ImgContainer.SRC_PICAMERA:
                self.get_image_picamera()
            elif self._source == ImgContainer.SRC_932L:
                self.get_image_932l()
            elif self._source == ImgContainer.SRC_932L_MJPEG:
                self.get_image_932l_mjpeg()
            self._valid = True
        except Exception as ex:
            self._valid = False

            # Try to connect again - really only applicable to video stream?
            self.connect()
            raise ex

    def __init__(self, source=SRC_932L, fn=None,
                 dlink932l_ip=None, dlink932l_uname=None, dlink932l_pwd=None,
                 video_hz=15):
        try:
            self._source = source
            if source == ImgContainer.SRC_PICAMERA:
                self.init_picamera()
            elif source == ImgContainer.SRC_932L:
                self._932l_ip = dlink932l_ip
                self._932l_uname = dlink932l_uname
                self._932l_pwd = dlink932l_pwd
            elif source == ImgContainer.SRC_932L_MJPEG:
                self._932l_ip = dlink932l_ip
                self._932l_uname = dlink932l_uname
                self._932l_pwd = dlink932l_pwd
                
                self._vid_capture = cv2.VideoCapture()
                self._vid_hz = video_hz
                    
            self._valid = True
        except Exception as ex:
            self._valid = False
            raise ex

    def connect(self):
        if self._source == ImgContainer.SRC_932L_MJPEG and \
           self._vid_capture.isOpened() == False:
            # Open connection
            start_time = rospy.Time.now()
            self._vid_capture.open("http://" + self._932l_uname + ":" + \
                                   self._932l_pwd + "@" + self._932l_ip + \
                                   "/MJPEG.CGI?.mjpg")
            dur_open = rospy.Time.now() - start_time

            # Fast forward past buffered frames
            for iii in range(0, int(dur_open.to_sec() * self._vid_hz)):
                self._vid_capture.grab()
            
    def release(self):
        if self._source == ImgContainer.SRC_932L_MJPEG and \
           self._vid_capture.isOpened() == True:
            # Close
            self._vid_capture.release()
            
    def valid(self):
        return self._valid 
        
    def get_cv_image(self):
        return self._cv_img

    def get_original_cv_image(self):
        return self._cv_img_orig
    
    def get_original_jpg_image(self):
        rval, jpg_img = cv2.imencode(".jpg", self.get_original_cv_image())
        if rval:
            bimg = bytearray(jpg_img)
            return bimg
        else:
            return None

    def save_image(self, fn):
        cv2.imwrite(fn, self.get_original_cv_image())
       

def main():
    rospy.init_node('camera_driver', anonymous=True)
    
    rospy.loginfo(rospy.get_caller_id() + " OpenCV version %s", cv2.__version__)

    param_dl_ip = "dlink932l_ip"
    param_dl_uname = "dlink932l_username"
    param_dl_pwd = "dlink932l_pwd"
    param_dl_use_mjpeg = "dlink932_use_mjpeg"
    dlink932l_ip = rospy.get_param("~" + param_dl_ip, default=None)
    dlink932l_uname = rospy.get_param("~" + param_dl_uname, default="admin")
    dlink932l_pwd = rospy.get_param("~" + param_dl_pwd, default="")
    dlink932l_use_mjpeg = rospy.get_param("~" + param_dl_use_mjpeg, default=True)
    mjpeg_hz = rospy.get_param("~mjpeg_hz", default=15)
    
    rospy.on_shutdown(shutdown_cb)
    
    image_pub = rospy.Publisher("camera_image",Image, queue_size=25)
    bridge = CvBridge()

    if dlink932l_ip == None:
        src_img = ImgContainer.SRC_PICAMERA 
    else:
        if dlink932l_use_mjpeg == True:
            src_img = ImgContainer.SRC_932L_MJPEG
        else:
            src_img = ImgContainer.SRC_932L
        
    img_ct = ImgContainer(src_img, fn=None, dlink932l_ip=dlink932l_ip,
                          dlink932l_uname=dlink932l_uname,
                          dlink932l_pwd=dlink932l_pwd,
                          video_hz=mjpeg_hz)

    prev_connections = 0
    rate = rospy.Rate(15) # 5 Hz
    rate_no_conn = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        if image_pub.get_num_connections() > 0:
            if prev_connections == 0:
                img_ct.connect()
                prev_connections = 1
                
            img_ct.get_image()
            img_msg = bridge.cv2_to_imgmsg(img_ct.get_original_cv_image(), "bgr8")
            image_pub.publish(img_msg)
            rate.sleep()
        else:
            if prev_connections != 0:
                img_ct.release()
                prev_connections = 0

            #rospy.loginfo("No one")
            rate_no_conn.sleep()

if __name__ == '__main__':
    try:
        main()    
    except rospy.ROSInterruptException:
        pass
