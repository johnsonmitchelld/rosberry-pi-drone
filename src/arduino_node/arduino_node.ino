/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
// ROS include files
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

// IMU include files
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define ZERO_THROTTLE 127
#define MAX_THROTTLE 255
#define THROTTLE_LIMIT 180

#define MOTOR1 3 // front left
#define MOTOR2 9 // front right
#define MOTOR3 10 // rear right
#define MOTOR4 11 // rear left

float throttle_setting = 0;

float voltage = 0;
unsigned long lastVoltageMsg = 0;
unsigned long current_time = 0;

void messageCb( const sensor_msgs::Joy& joystick_msg);

Adafruit_BNO055 bno = Adafruit_BNO055(55);

ros::NodeHandle nh;
geometry_msgs::Vector3 sensor_msg;
std_msgs::Float32 voltage_msg;
ros::Publisher pub("imureading", &sensor_msg);
ros::Publisher voltagepub("battery_voltage", &voltage_msg);

ros::Subscriber<sensor_msgs::Joy> sub("controller_values", &messageCb );

void setup()
{
  pinMode(MOTOR1, OUTPUT);
  pinMode(MOTOR2, OUTPUT);
  pinMode(MOTOR3, OUTPUT);
  pinMode(MOTOR4, OUTPUT);

  analogWrite(MOTOR1, ZERO_THROTTLE);
  analogWrite(MOTOR2, ZERO_THROTTLE);
  analogWrite(MOTOR3, ZERO_THROTTLE);
  analogWrite(MOTOR4, ZERO_THROTTLE);

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  bno.setExtCrystalUse(true);

  nh.initNode();
  nh.advertise(pub);
  nh.advertise(voltagepub);
  nh.subscribe(sub);

  sensor_msg.x = 0.0;
  sensor_msg.y = 0.0;
  sensor_msg.z = 0.0;

  readVoltage();
  
  delay(5000);
}

void loop()
{
  current_time = millis();
  readVoltage();
  readSensor();
  nh.spinOnce();
}

void messageCb( const sensor_msgs::Joy& joystick_msg){
  throttle_setting = joystick_msg.axes[0];
  float raw_throttle = joystick_msg.axes[0] * (THROTTLE_LIMIT - 127);
  int throttle = (int) raw_throttle + 127;
  sensor_msg.x = raw_throttle;
  if (voltage > 12.0){
    updatemotors(throttle);
  }
  else {
    updatemotors(0);
  }
}

void readSensor(){
  sensors_event_t event; 
  bno.getEvent(&event);
  sensor_msg.x = event.orientation.x;
  sensor_msg.y = event.orientation.y;
  sensor_msg.z = event.orientation.z; 
  pub.publish( &sensor_msg );
}

void updatemotors(int throttle){
  analogWrite(MOTOR1, throttle);
  analogWrite(MOTOR2, throttle);
  analogWrite(MOTOR3, throttle);
  analogWrite(MOTOR4, throttle);
}

void readVoltage(){
  if(current_time - lastVoltageMsg > 1000){
    int sensorValue = analogRead(A0);
    voltage = sensorValue * (5.0 / 1024.0) * 4;    
    lastVoltageMsg = current_time;
    voltage_msg.data = voltage;
    voltagepub.publish( &voltage_msg );
  }
}

