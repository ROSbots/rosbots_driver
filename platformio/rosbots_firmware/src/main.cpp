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

char twist_rcv[15] = "twist received";

void twistCb( const geometry_msgs::Twist &twist_msg ) {
  str_msg.data = twist_rcv;
  chatter.publish( &str_msg );
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &twistCb );


void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);
  
  // wait for a second
  delay(1000);
  
  // turn the LED off by making the voltage LOW
  digitalWrite(LED_BUILTIN, LOW);

  
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.loginfo("UNO blinked");
  nh.spinOnce();

  // wait for a second
  delay(1000);
}
