/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
// ROS include files
#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

// IMU include files
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define ZERO_THROTTLE   127
#define MAX_THROTTLE    255
#define THROTTLE_LIMIT  180

#define MOTOR1 3 // front left - clockwise
#define MOTOR2 9 // front right - counterclockwise
#define MOTOR3 10 // rear left - counterclockwise
#define MOTOR4 11 // rear right - clockwise


#define ROLL      0
#define PITCH     1
#define YAW       2

float throttle_setting = 0;

// Used to calculate pulse duration on each channel
unsigned long current_time;


// Calculated angular motion on each axis: Yaw, Pitch, Roll
float angular_motions[3] = {0, 0, 0};

/**
 * Real measures on 3 axis calculated from gyro AND accelerometer in that order : Yaw, Pitch, Roll
 *  - Left wing up implies a positive roll
 *  - Nose up implies a positive pitch
 *  - Nose right implies a positive yaw
 */
float measures[3] = {0, 0, 0};

// MPU's temperature
int temperature;

// Init flag set to TRUE after first loop
boolean initialized;
// ----------------------- Variables for servo signal generation -------------
unsigned int  period; // Sampling period
unsigned long loop_timer;
unsigned long now, difference;

float throttle_setting_esc1 = 0.0;
float throttle_setting_esc2 = 0.0;
float throttle_setting_esc3 = 0.0;
float throttle_setting_esc4 = 0.0;

// ------------- Global variables used for PID controller --------------------
float pid_set_points[3] = {0, 0, 0}; // Roll, Pitch, Yaw
// Errors
float errors[3];                     // Measured errors (compared to instructions) : [Roll, Pitch, Yaw]
float delta_err[3]      = {0, 0, 0}; // Error deltas in that order   : Roll, Pitch, Yaw
float error_sum[3]      = {0, 0, 0}; // Error sums (used for integral component) : [Roll, Pitch, Yaw]
float previous_error[3] = {0, 0, 0}; // Last errors (used for derivative component) : [Roll, Pitch, Yaw]
// PID coefficients
float Kp[3] = {.01, .01, .01};    // P coefficients in that order : Roll, Pitch, Yaw
float Ki[3] = {0.0, 0.0, 0.0}; // I coefficients in that order : Roll, Pitch, Yaw
float Kd[3] = {0.0, 0.0, 0.0};        // D coefficients in that order : Roll, Pitch, Yaw

float yaw_pid      = 0;
float pitch_pid    = 0;
float roll_pid     = 0;
// ---------------------------------------------------------------------------

float battery_voltage;
unsigned long last_voltage_msg = 0;
unsigned long last_imu_reading = 0;
unsigned long imu_reading_interval = 1000/180; // (milliseconds) 180Hz is the approximate run time of the control loop

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
  Serial.begin(9600);
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
  sensor_msg.x = measures[ROLL];
  sensor_msg.y = measures[PITCH];
  sensor_msg.z = measures[YAW];

  nh.initNode();
  nh.advertise(pub);
  nh.advertise(voltagepub);
  nh.subscribe(sub);

  delay(5000);
  readVoltage();
  readSensor();
}

void loop()
{
  readSensor();
  
  calculateErrors();

  pidController();

  applyMotorSpeed();

  readVoltage();

  publishData();

  nh.spinOnce();
}

void readSensor(){
  sensors_event_t event; 
  bno.getEvent(&event);
  imu_reading_interval = millis() - last_imu_reading;
  last_imu_reading = millis();
  measures[ROLL] = event.orientation.z;
  measures[PITCH] = -event.orientation.y;
  measures[YAW] = event.orientation.x;
}

/**
 * Calculate errors used by PID controller
 */
void calculateErrors() {
    // Calculate current errors
    errors[YAW]   = pid_set_points[YAW];
    errors[PITCH] = measures[PITCH] - pid_set_points[PITCH];
    errors[ROLL]  = measures[ROLL] - pid_set_points[ROLL];

    // Calculate sum of errors : Integral coefficients
    error_sum[PITCH] += errors[PITCH];
    error_sum[ROLL]  += errors[ROLL];

    // Keep values in acceptable range
    error_sum[PITCH] = minMax(error_sum[PITCH], -30.0, 30.0);
    error_sum[ROLL]  = minMax(error_sum[ROLL], -30.0, 30.0);

    // Calculate error delta : Derivative coefficients. Multiply by 1000 because time is in milliseconds
    delta_err[PITCH] = 1000 * (errors[PITCH] - previous_error[PITCH]) / (float) imu_reading_interval;
    delta_err[ROLL]  = 1000 * (errors[ROLL]  - previous_error[ROLL]) / (float) imu_reading_interval;

    // Save current error as previous_error for next time
    previous_error[YAW]   = errors[YAW];
    previous_error[PITCH] = errors[PITCH];
    previous_error[ROLL]  = errors[ROLL];
}


/**
 * Calculate motor speed for each motor of an X quadcopter depending on received instructions and measures from sensor
 * by applying PID control.
 *
 * (A) (B)     x
 *   \ /     z ↑
 *    X       \|
 *   / \       +----→ y
 * (C) (D)
 *
 * Motors A & D run clockwise.
 * Motors B & C run counter-clockwise.
 * 
 */
void pidController() {

    // Do not calculate anything if throttle is 0
    if (throttle_setting >= 0.0) {
        // PID = e.Kp + ∫e.Ki + Δe.Kd
        yaw_pid   = (errors[YAW]   * Kp[YAW]);
        pitch_pid = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (delta_err[PITCH] * Kd[PITCH]);
        roll_pid  = (errors[ROLL]  * Kp[ROLL])  + (error_sum[ROLL]  * Ki[ROLL])  + (delta_err[ROLL]  * Kd[ROLL]);

        // Keep values within acceptable range. TODO export hard-coded values in variables/const
        yaw_pid   = minMax(yaw_pid, -0.5, 0.5);
        pitch_pid = minMax(pitch_pid, -0.5, 0.5);
        roll_pid  = minMax(roll_pid, -0.5, 0.5);

        // Calculate pulse duration for each ESC
        throttle_setting_esc1 = throttle_setting - roll_pid - pitch_pid + yaw_pid;
        throttle_setting_esc2 = throttle_setting + roll_pid - pitch_pid - yaw_pid;
        throttle_setting_esc3 = throttle_setting - roll_pid + pitch_pid - yaw_pid;
        throttle_setting_esc4 = throttle_setting + roll_pid + pitch_pid + yaw_pid;
    }

    // Prevent out-of-range-values
    throttle_setting_esc1 = minMax(throttle_setting_esc1, 0.0, 1.0);
    throttle_setting_esc2 = minMax(throttle_setting_esc2, 0.0, 1.0);
    throttle_setting_esc3 = minMax(throttle_setting_esc3, 0.0, 1.0);
    throttle_setting_esc4 = minMax(throttle_setting_esc4, 0.0, 1.0);
}


void applyMotorSpeed() {
  analogWrite(MOTOR1, calculatePWMSetting(throttle_setting_esc1));
  analogWrite(MOTOR2, calculatePWMSetting(throttle_setting_esc2));
  analogWrite(MOTOR3, calculatePWMSetting(throttle_setting_esc3));
  analogWrite(MOTOR4, calculatePWMSetting(throttle_setting_esc4));
}


int calculatePWMSetting(float throttle) {
    float raw_throttle = throttle * (THROTTLE_LIMIT - 127);
    int pwm_setting = (int) raw_throttle + 127;
    return pwm_setting;
}

void readVoltage(){
  if(millis() - last_voltage_msg > 1000){
    int sensorValue = analogRead(A0);
    battery_voltage = sensorValue * (5.0 / 1024.0) * 4;    
    voltage_msg.data = battery_voltage;
    voltagepub.publish( &voltage_msg );
    last_voltage_msg = current_time;
  }
}

void publishData(){
  sensor_msg.x = roll_pid;
  sensor_msg.y = pitch_pid;
  sensor_msg.z = yaw_pid; 
  pub.publish( &sensor_msg );
}

void messageCb( const sensor_msgs::Joy& joystick_msg){
  throttle_setting = joystick_msg.axes[0];
  pid_set_points[ROLL] = joystick_msg.axes[1];
  pid_set_points[PITCH] = joystick_msg.axes[2];
  pid_set_points[YAW] = joystick_msg.axes[3];
}

/**
 * Make sure that given value is not over min_value/max_value range.
 *
 * @param float value     : The value to convert
 * @param float min_value : The min value
 * @param float max_value : The max value
 *
 * @return float
 */
float minMax(float value, float min_value, float max_value) {
    if (value > max_value) {
        value = max_value;
    } else if (value < min_value) {
        value = min_value;
    }

    return value;
}
