/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <sensor_msgs/Joy.h>

#define ZERO_THROTTLE 127
#define MAX_THROTTLE 140

int motorPin1 = 3;
int motorPin2 = 5;
int motorPin3 = 5;
int motorPin4 = 9;

ros::NodeHandle nh;



void messageCb( const sensor_msgs::Joy& joystick_msg){
  float raw_throttle = joystick_msg.axes[0] * (MAX_THROTTLE - 127);
  int throttle = (int) raw_throttle + 127;
  analogWrite(motorPin1, throttle);
  analogWrite(motorPin2, throttle);
  analogWrite(motorPin3, throttle);
  analogWrite(motorPin4, throttle);
}

ros::Subscriber<sensor_msgs::Joy> sub("controller_values", &messageCb );

void setup()
{
    
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);

  analogWrite(motorPin1, ZERO_THROTTLE);
  analogWrite(motorPin2, ZERO_THROTTLE);
  analogWrite(motorPin3, ZERO_THROTTLE);
  analogWrite(motorPin4, ZERO_THROTTLE);

  nh.initNode();
  nh.subscribe(sub);

  delay(5000);
}

void loop()
{
  nh.spinOnce();
}