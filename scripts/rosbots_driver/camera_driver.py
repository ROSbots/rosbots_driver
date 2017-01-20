#!/usr/bin/env python
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
            self._valid = True
        except Exception as ex:
            self._valid = False
            raise ex

    def __init__(self, source=SRC_932L, fn=None,
                 dlink932l_ip=None, dlink932l_uname=None, dlink932l_pwd=None):
        try:
            self._source = source
            if source == ImgContainer.SRC_PICAMERA:
                self.init_picamera()
            elif source == ImgContainer.SRC_932L:
                self._932l_ip = dlink932l_ip
                self._932l_uname = dlink932l_uname
                self._932l_pwd = dlink932l_pwd
            self._valid = True
        except Exception as ex:
            self._valid = False
            raise ex

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
    dlink932l_ip = rospy.get_param("~" + param_dl_ip, default=None)
    dlink932l_uname = rospy.get_param("~" + param_dl_uname, default="admin")
    dlink932l_pwd = rospy.get_param("~" + param_dl_pwd, default="")
    
    rospy.on_shutdown(shutdown_cb)
    
    image_pub = rospy.Publisher("camera_image",Image, queue_size=5)
    bridge = CvBridge()

    if dlink932l_ip == None:
        src_img = ImgContainer.SRC_PICAMERA 
    else:
        src_img = ImgContainer.SRC_932L
        
    img_ct = ImgContainer(src_img, fn=None, dlink932l_ip=dlink932l_ip,
                          dlink932l_uname=dlink932l_uname,
                          dlink932l_pwd=dlink932l_pwd)
    
    rate = rospy.Rate(5) # 5 Hz
    while not rospy.is_shutdown():
        img_ct.get_image()
        img_msg = bridge.cv2_to_imgmsg(img_ct.get_original_cv_image(), "bgr8")
        image_pub.publish(img_msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()    
    except rospy.ROSInterruptException:
        pass
