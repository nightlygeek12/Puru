
/*******************************************************************************
* Copyright 2021 Drebar
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
********************************************************************************/

/*Authors: Asyraf Shahrom, Baarath Kunjunni*/

/**************************************************************
*Define ROS parameters
***************************************************************/

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include "variable_def.h"



/**************************************************************
*Define IMU libraries
***************************************************************/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

/**************************************************************
*ROS NodeHandle
***************************************************************/

ros::NodeHandle nh;
ros::Time rosNow(void);
uint32_t current_offset;

/**************************************************************
*Param definition
***************************************************************/
int seq=0;
/**************************************************************
*ROS Publisher
***************************************************************/
sensor_msgs::Imu imu_raw_msg;
ros::Publisher imu_raw_pub("imu_raw", &imu_raw_msg);

// sensor_msgs::Imu imu_data_msg;
// ros::Publisher imu_data_pub("imu_data", &imu_data_msg);

sensor_msgs::MagneticField  mag_msg;
ros::Publisher mag_data_pub("mag_data", &mag_msg);

std_msgs::Float32 speed_msg;
ros::Publisher speed_pub("velocity", &speed_msg);

nav_msgs::Odometry wheel_odom;
ros::Publisher odom_pub("wheel/odometry", &wheel_odom);

geometry_msgs::TwistWithCovarianceStamped wheel_speed;
ros::Publisher vel_pub("wheel/speed", &wheel_speed);
/*************************************************************
 * ROS callback function
**************************************************************/
void joystick(const geometry_msgs::Twist& msg){
  x_axis = msg.linear.x;
  z_axis = msg.angular.z;

  vel=x_axis*200;
  turn=z_axis*200;
}
/**************************************************************
*ROS Subscriber
***************************************************************/
ros::Subscriber<geometry_msgs::Twist> joy_sub("/cmd_vel", &joystick);

/***************************************************************
*SETUP FREQUENCY
****************************************************************/
#define IMU_PUBLISH_FREQUENCY           200//Hz
#define ENC_PUBLISH_FREQUENCY	          100 //Hz
#define CONTROL_MOTOR_SPEED_FREQUENCY   30//HZ
#define CONTROL_MOTOR_TIMEOUT           50 //ms
#define NAV_PUBLISH_FREQUENCY           30//Hz
/***************************************************************
* SoftwareTimer
****************************************************************/

static uint32_t tTime[10];

ros::Time rosNow()
{
  return nh.now();
}
