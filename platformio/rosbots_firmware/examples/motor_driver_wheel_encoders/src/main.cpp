/**
 * Motor Driver Example
 */

#include <ros.h>
#include "Arduino.h"

#include <std_msgs/UInt32.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

#define M_LEFT_PWM 6
#define M_LEFT_FR 7
#define M_RIGHT_PWM 5
#define M_RIGHT_FR 4

void turnWheel( const std_msgs::Float32 &wheel_power,
                unsigned int pwm_pin,
                unsigned int fr_pin ) {
    float factor = max(min(wheel_power.data, 1.0f), -1.0f);
    if( factor >= 0 ) {
        digitalWrite(fr_pin, LOW);
        analogWrite(pwm_pin, (unsigned int)(255 * factor));
    } else {
        digitalWrite(fr_pin, HIGH);
        analogWrite(pwm_pin, (unsigned int)(255 * (1.0f + factor)));
    }   
}
void rightWheelCb( const std_msgs::Float32 &wheel_power ) {
    turnWheel( wheel_power, M_RIGHT_PWM, M_RIGHT_FR );
}
void leftWheelCb( const std_msgs::Float32 &wheel_power ) {
    turnWheel( wheel_power, M_LEFT_PWM, M_LEFT_FR );
}
ros::Subscriber<std_msgs::Float32> sub_right("wheel_power_right",
                                            &rightWheelCb );
ros::Subscriber<std_msgs::Float32> sub_left("wheel_power_left",
                                           &leftWheelCb );

volatile unsigned int rwheel=0;
volatile unsigned long rdebounce=0;
volatile unsigned int lwheel=0;
volatile unsigned long ldebounce=0;

#define DEBOUNCE_MS 1500 // in micro-sec

void lspeed() {
    // N microseconds since last interrupt
    unsigned long m = micros();
    if( m - ldebounce > DEBOUNCE_MS ){ 
        lwheel = lwheel + 1;
    }
    // We assume real signal is less than debounce ms
    // so always update the last bounce time
    ldebounce = m;
}

void rspeed() {
    // N microseconds since last interrupt
    unsigned long m = micros();
    if( m - rdebounce > DEBOUNCE_MS ){
        rwheel = rwheel + 1;
    }
    // We assume real signal is less than debounce ms
    // so always update the last bounce time
    rdebounce = m;
}

std_msgs::UInt32 rticks_msg;
std_msgs::UInt32 lticks_msg;
ros::Publisher rticks_pub("wheel_ticks_right", &rticks_msg);
ros::Publisher lticks_pub("wheel_ticks_left", &lticks_msg);

void setup()
{
    // Set up wheel encoder interrupts
    pinMode(2, INPUT);
    attachInterrupt(digitalPinToInterrupt(2), rspeed, CHANGE);
    pinMode(3, INPUT);
    attachInterrupt(digitalPinToInterrupt(3), lspeed, CHANGE);
    
    // initialize LED digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    
    pinMode(M_LEFT_PWM, OUTPUT);
    pinMode(M_LEFT_FR, OUTPUT);
    pinMode(M_RIGHT_PWM, OUTPUT);
    pinMode(M_RIGHT_FR, OUTPUT);
    
    // Init motors to stop
    digitalWrite(M_LEFT_FR, LOW);
    digitalWrite(M_RIGHT_FR, LOW);
    analogWrite(M_LEFT_PWM, 0);
    analogWrite(M_RIGHT_PWM, 0);
    
    nh.initNode();
    nh.subscribe(sub_right);
    nh.subscribe(sub_left);
    
    nh.advertise(rticks_pub);
    nh.advertise(lticks_pub);

    // Necessary for encoder interrupts to initialize
    delay(200);
}

unsigned long last_tick_publish_ms = 0;
void loop()
{
    unsigned int rw = rwheel;
    unsigned int lw = lwheel;

    // Only publish once per second if tick count has not
    // changed
    if( millis() >= last_tick_publish_ms + 1000 ||
        rticks_msg.data != rw ||
        lticks_msg.data != lw ) {
        // Write out wheel encoder ticks
        rticks_msg.data = rw;
        rticks_pub.publish(&rticks_msg);
        lticks_msg.data = lw;
        lticks_pub.publish(&lticks_msg);
        last_tick_publish_ms = millis();
    } 
    
  //nh.loginfo("Log Me");
  nh.spinOnce();

  // wait for a second
  delay(200);
}
