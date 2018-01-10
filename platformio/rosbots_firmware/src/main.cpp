/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <ros.h>
#include "Arduino.h"

#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
geometry_msgs::Twist twist_msg;

ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

#define M_LEFT_PWM 5
#define M_LEFT_FR 4
#define M_RIGHT_PWM 3
#define M_RIGHT_FR 2

#define EN_LEFT 8
#define EN_RIGHT 7

char fwd_rcv[5] = "fwd";
char stop_rcv[5] = "stop";

void twistCb( const geometry_msgs::Twist &twist_msg ) {
  if( twist_msg.linear.x > 0 ) {
    float factor = min(twist_msg.linear.x, 1.0f);
    digitalWrite(M_LEFT_FR, LOW);
    digitalWrite(M_RIGHT_FR, LOW);
    analogWrite(M_LEFT_PWM, (unsigned int)(255 * factor));
    analogWrite(M_RIGHT_PWM, (unsigned int)(255 * factor));
    str_msg.data = "fwd";
    chatter.publish(&str_msg);
  } else {
    digitalWrite(M_LEFT_FR, LOW);
    digitalWrite(M_RIGHT_FR, LOW);
    analogWrite(M_LEFT_PWM, 0);
    analogWrite(M_RIGHT_PWM, 0);
    str_msg.data = "stop";
    chatter.publish(&str_msg);
  }
  // str_msg.data = twist_rcv;
  // chatter.publish( &str_msg );
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &twistCb );

unsigned int curEnLeft;
unsigned int curEnRight;
unsigned int countEnLeft;
unsigned int countEnRight;

void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(M_LEFT_PWM, OUTPUT);
  pinMode(M_LEFT_FR, OUTPUT);
  pinMode(M_RIGHT_PWM, OUTPUT);
  pinMode(M_RIGHT_FR, OUTPUT);

  // Init motors to stop
  digitalWrite(M_LEFT_FR, LOW);
  digitalWrite(M_RIGHT_FR, LOW);
  analogWrite(M_LEFT_PWM, 0);
  analogWrite(M_RIGHT_PWM, 0);

  // Init encoders
  pinMode(EN_LEFT, INPUT);
  pinMode(EN_RIGHT, INPUT);
  curEnLeft = digitalRead(EN_LEFT);
  curEnRight = digitalRead(EN_RIGHT);
  countEnLeft = 0;
  countEnRight = 0;

  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  // wait for a second
  // delay(1000);
  
  // turn the LED off by making the voltage LOW
  // digitalWrite(LED_BUILTIN, LOW);

  //str_msg.data = hello;
  //chatter.publish( &str_msg );
  //nh.loginfo("UNO blinked");
  nh.spinOnce();

  // wait for a second
  //delay(1000);
}
