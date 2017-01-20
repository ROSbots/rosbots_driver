#!/usr/bin/env python
#import RPIO as GPIO 
import RPi.GPIO as GPIO
from RPIO import PWM

import RPIO

import rospy
from geometry_msgs.msg import Twist

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

        rospy.Subscriber("twist", Twist, self.twist_callback)    

        rospy.on_shutdown(self.shutdown_cb)
        
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
        RPIO.setup(self.encoder_right, RPIO.IN)
        RPIO.add_interrupt_callback(self.encoder_right,
                                    self.encoder_callback, edge='rising',
                                    debounce_timeout_ms=10,
                                    pull_up_down=RPIO.PUD_DOWN,
                                    threaded_callback=True)
        RPIO.setup(self.encoder_left, RPIO.IN)
        RPIO.add_interrupt_callback(self.encoder_left,
                                    self.encoder_callback, edge='rising',
                                    debounce_timeout_ms=10,
                                    pull_up_down=RPIO.PUD_DOWN,
                                    threaded_callback=True)
        
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

    def encoder_callback(self, gpio_id, val):
        rospy.loginfo(rospy.get_caller_id() + ": gpio %s: %s", gpio_id, val)
            

def main():
    mdriver = MotorDriverL9110S()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    main()
