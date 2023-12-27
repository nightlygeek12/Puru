
#define ROSSERIAL_ARDUINO_TCP

#include "portenta_setup.h"

#include "portenta_pindef.h"

/******************************************************\
 * TCP SETUP header files
 ******************************************************/
#include <SPI.h>
#include <Ethernet.h>


byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 105);

// Set the rosserial socket server IP address
IPAddress server(192,168,1,102);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;


void setup() {
  Serial.begin(115200);
  Ethernet.begin(mac, ip);
  delay (1000);
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  //nh.getHardware()->setBaud(115200);
  nh.subscribe(joy_sub);
  nh.advertise(speed_pub);
  nh.advertise(imu_raw_pub);
  nh.advertise(mag_data_pub);
  //nh.advertise(odom_pub);
  nh.advertise(vel_pub);
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  pinMode(pinC, INPUT_PULLUP);
  pinMode(pinD, INPUT_PULLUP);
  
  pinMode(pwm_right, OUTPUT);
  pinMode(dir_right, OUTPUT);
  pinMode(pwm_left, OUTPUT);
  pinMode(dir_left, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(pinA), A_CHANGE, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(pinB), B_CHANGE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinC), C_CHANGE, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(pinD), D_CHANGE, CHANGE);
  bno.begin();
  
  delay (1000);
  while (!nh.connected())
  {
    nh.spinOnce();
  }
  nh.loginfo("PORTENTA CONNECTED");
  delay(1000);

}

void loop() {
  
//  get_encoder_right();
//  get_encoder_left();
//  get_speed();
//  Serial.print(get_encoder_right());
//  Serial.print("\t\t");
//  Serial.println(get_encoder_left());
  //Serial.print("\t\t");
  //Serial.println(get_speed());
  uint32_t t=millis();  
  
  if ((t-tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
  {
    
    if ((t-tTime[3]) > CONTROL_MOTOR_TIMEOUT) 
    {
      pwmR=0;
      pwmL=0;
      tTime[3] = t;
    } 
    
    else {
     
      robot_pose();
      motorfunction();
    }
    tTime[0] = t;
    
  }
  
  if ((t-tTime[1])>=(1000/ENC_PUBLISH_FREQUENCY))
  {
  //get_encoder_right();
  //get_encoder_left();
  get_speed();
  publishspeed_msg();
  //publishwheel_nav();
  publishwheel_vel();
  
  tTime[1]=t;
  }

  
  if ((t-tTime[2]) >= (1000 / IMU_PUBLISH_FREQUENCY))
  {
  publishmag_data();
  publishimu_raw();
  tTime[2]=t;
  }

  

  nh.spinOnce();
  delay(10);
}

/**********************************************************\
*FUNCTION DEFINITION
\***********************************************************/

void robot_pose(){
  if(vel>0){
 pwmR=vel-turn;
 pwmL=vel+turn;
  }
  else if(vel<0){
    pwmR=vel+turn;
    pwmL=vel-turn;
    
  }

 if (pwmR>200){
  pwmR=200;
 }
 else{
  pwmR=pwmR;
 }

 if(pwmL>200){
  pwmL=200;
 }
 else{
  pwmL=pwmL;
 }
}

void motorfunction(){
  analogWrite(pwm_right,abs(pwmR));
  analogWrite(pwm_left,abs(pwmL));
  if (pwmR>0){
    digitalWrite(dir_right,HIGH); 
  }
  else if(pwmR<0){
    digitalWrite(dir_right,LOW); 
  }

  if (pwmL>0){
    digitalWrite(dir_left,LOW);
  }

  else if(pwmL<0){
     digitalWrite(dir_left,HIGH);
  }
}

float get_speed()
{
  //velocity=(speed_wheel_left+speed_wheel_right)/2;
  velocity=(get_encoder_left()+get_encoder_right())/2;
  //velocity=get_encoder_right();
  return velocity;
}

float get_encoder_left()
{
  
  long currT=micros();
  deltaT_Left=(currT-prevT_Left)/1.0e6;
  prevT_Left=currT;

  PPS_Left=(pulsesA-previous_pulseA)/deltaT_Left;
  previous_pulseA=pulsesA;
  rpm_left=PPS_Left*(60*0.00025);
  speed_wheel_left = rpm_left*(2*PI*wheel_rad/60);
  
  return speed_wheel_left;
  //return pulse_per_periodA;
}

float get_encoder_right()
{
  long currT=micros();
  deltaT_Right=(currT-prevT_Right)/1.0e6;
  prevT_Right=currT;

  PPS_Right=(pulsesB-previous_pulseB)/deltaT_Right;
  previous_pulseB=pulsesB;
  rpm_right=PPS_Right*(60*0.00025);
  speed_wheel_right = rpm_right*(2*PI*wheel_rad/60);
  
  return speed_wheel_right;
  //return pulse_per_periodB;
}

/***********************************************************\
*PUBLISHER FUNCTION
\***********************************************************/

void publishspeed_msg()
{
  speed_msg.data=velocity;
  speed_pub.publish(&speed_msg);
}

/************************************************\
 * Encoder interrupt service routine functions
\************************************************/



void A_CHANGE() {
  if ( digitalRead(pinB) == 0 ) {
    if ( digitalRead(pinA) == 0 ) {
      pulsesA--;
    } else {
      pulsesA++;
    }
  } else {
    if ( digitalRead(pinA) == 0 ) {
      pulsesA++;
    } else {
      // A rose, B is low
      pulsesA--;
    }
  }

}


void B_CHANGE(){
  if ( digitalRead(pinA) == 0 ) {
    if ( digitalRead(pinB) == 0 ) {
      // B fell, A is low
      pulsesA ++; // moving forward
    } else {
      // B rose, A is low
      pulsesA--; // moving reverse
    }
 } else {
    if ( digitalRead(pinB) == 0 ) {
      // B fell, A is high
      pulsesA--; // moving reverse
    } else {
      // B rose, A is high
      pulsesA++; // moving forward
    }
  }
  
}


void C_CHANGE() {
  if ( digitalRead(pinD) == 0 ) {
    if ( digitalRead(pinC) == 0 ) {
      pulsesB++;
    } else {
      pulsesB--;
    }
  } else {
    if ( digitalRead(pinC) == 0 ) {
      pulsesB--;
    } else {
      // A rose, B is low
      pulsesB++;
    }
  }
  
}


void D_CHANGE(){
  if ( digitalRead(pinC) == 0 ) {
    if ( digitalRead(pinD) == 0 ) {
      // B fell, A is low
      pulsesB--; // moving forward
    } else {
      // B rose, A is low
      pulsesB++; // moving reverse
    }
 } else {
    if ( digitalRead(pinD) == 0 ) {
      // B fell, A is high
      pulsesB++; // moving reverse
    } else {
      // B rose, A is high
      pulsesB--; // moving forward
    }
  }
  
}



/***********************************************************\
*IMU FUNCTION
\***********************************************************/
void publishimu_raw()
{
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); 
  imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Quaternion quat=bno.getQuat();

  imu_raw_msg.header.stamp=rosNow();
  imu_raw_msg.header.frame_id="base_link";
  imu_raw_msg.header.seq=seq;
  imu_raw_msg.orientation.w=quat.w();
  imu_raw_msg.orientation.x=quat.x();
  imu_raw_msg.orientation.y=quat.y();
  imu_raw_msg.orientation.z=quat.z();
  
  imu_raw_msg.orientation_covariance[0] = 0.0025;
  imu_raw_msg.orientation_covariance[1] = 0;
  imu_raw_msg.orientation_covariance[2] = 0;
  imu_raw_msg.orientation_covariance[3] = 0;
  imu_raw_msg.orientation_covariance[4] = 0.0025;
  imu_raw_msg.orientation_covariance[5] = 0;
  imu_raw_msg.orientation_covariance[6] = 0;
  imu_raw_msg.orientation_covariance[7] = 0;
  imu_raw_msg.orientation_covariance[8] = 0.0025;

  
  imu_raw_msg.angular_velocity.x=gyroscope.x();
  imu_raw_msg.angular_velocity.y=gyroscope.y();
  imu_raw_msg.angular_velocity.z=(gyroscope.z()*M_PI)/180; //60
  
  imu_raw_msg.angular_velocity_covariance[0] = 0.02;
  imu_raw_msg.angular_velocity_covariance[1] = 0;
  imu_raw_msg.angular_velocity_covariance[2] = 0;
  imu_raw_msg.angular_velocity_covariance[3] = 0;
  imu_raw_msg.angular_velocity_covariance[4] = 0.02;
  imu_raw_msg.angular_velocity_covariance[5] = 0;
  imu_raw_msg.angular_velocity_covariance[6] = 0;
  imu_raw_msg.angular_velocity_covariance[7] = 0;
  imu_raw_msg.angular_velocity_covariance[8] = 0.02;

  
  imu_raw_msg.linear_acceleration.x=linearaccel.x();
  imu_raw_msg.linear_acceleration.y=linearaccel.y();
  imu_raw_msg.linear_acceleration.z=linearaccel.z();
  
  imu_raw_msg.linear_acceleration_covariance[0] = 0.04;
  imu_raw_msg.linear_acceleration_covariance[1] = 0;
  imu_raw_msg.linear_acceleration_covariance[2] = 0;
  imu_raw_msg.linear_acceleration_covariance[3] = 0;
  imu_raw_msg.linear_acceleration_covariance[4] = 0.04;
  imu_raw_msg.linear_acceleration_covariance[5] = 0;
  imu_raw_msg.linear_acceleration_covariance[6] = 0;
  imu_raw_msg.linear_acceleration_covariance[7] = 0;
  imu_raw_msg.linear_acceleration_covariance[8] = 0.04;
  
  imu_raw_pub.publish(&imu_raw_msg);
  
}
void publishmag_data()
{
  imu::Vector<3> magnetometer = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  mag_msg.header.stamp=rosNow();
  mag_msg.header.frame_id="imu_mag";
  mag_msg.header.seq=seq;
  mag_msg.magnetic_field.x=magnetometer.x();
  mag_msg.magnetic_field.y=magnetometer.y();
  mag_msg.magnetic_field.z=magnetometer.z();
  mag_data_pub.publish(&mag_msg);
}

//void publishwheel_nav()
//{
//  imu::Quaternion quat=bno.getQuat();
//
//  wheel_odom.header.stamp=rosNow();
//  wheel_odom.header.frame_id="odom";
//  wheel_odom.child_frame_id="base_link";
//  wheel_odom.pose.pose.position.x=0;
//  wheel_odom.pose.pose.position.y=0;
//  wheel_odom.pose.pose.position.z=0;
//  wheel_odom.pose.pose.orientation.x=quat.x();
//  wheel_odom.pose.pose.orientation.y=quat.y();
//  wheel_odom.pose.pose.orientation.z=quat.z();
//  wheel_odom.pose.pose.orientation.w=quat.w();
//  wheel_odom.twist.twist.linear.x=velocity;
//  wheel_odom.twist.twist.linear.y=0;
//  wheel_odom.twist.twist.linear.z=0;
//  wheel_odom.twist.twist.angular.x=0;
//  wheel_odom.twist.twist.angular.y=0;
//  wheel_odom.twist.twist.angular.z=0;
//
//
//   for(int i = 0; i<36; i++) {
//    if(i == 0 || i == 7 || i == 14) {
//      wheel_odom.pose.covariance[i] = .01;
//     }
//     else if (i == 21 || i == 28 || i== 35) {
//       wheel_odom.pose.covariance[i] += 0.1;
//     }
//     else {
//       wheel_odom.pose.covariance[i] = 0;
//     }
//  }
// 
//  odom_pub.publish(&wheel_odom);
//
// 
//}


void publishwheel_vel()
{
  wheel_speed.header.stamp=rosNow();
  wheel_speed.header.frame_id="base_link";
  wheel_speed.twist.twist.linear.x=velocity;
  wheel_speed.twist.twist.linear.y=0;
  wheel_speed.twist.twist.linear.z=0;
  wheel_speed.twist.twist.angular.x=0;
  wheel_speed.twist.twist.angular.y=0;
  wheel_speed.twist.twist.angular.z=0;



  vel_pub.publish(&wheel_speed);
  
}
