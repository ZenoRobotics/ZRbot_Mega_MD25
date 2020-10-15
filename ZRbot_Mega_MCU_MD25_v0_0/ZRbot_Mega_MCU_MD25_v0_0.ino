//-------------------------------------------------------------------------------------
// File: zrbot_mega_mcu_md25.ino 
//
//--------------------------------------------------------------------------
// Written by Peter J. Zeno, Ph.D. <zenorobotics@gmail.com>, October 2020
//
// Copyright (C) ZenoRobotics, LLC - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential.
//--------------------------------------------------------------------------
//
// Version: 
//    First version built from tankbot_odometry_rosserial_v0_2.ino Modifications section 
//    list changes made with respect to the golden version. 
// 
// Modifications:
//    - Added external switch that enables/disables the rosserial logic.
//    - Added stopmotors() bookend calls to adjLeftRight() function. 
//    - Added 100nf between Vcc and Gnd of MD25. The MD25 propagates a lot of noise to the Arduino
//      and rpi via the serial interface to the Arduino. This causes the Arduino to lose connection
//      (rosserial) with the RPi and freezes up communication to the MD25 as well. The robot may act
//      erratically, like turning in circles and accelerating. 
//    - Enabled 2 second stop on MD25 in case of lost connection. Thus, to move forward, commands to move 
//      forward need to be received sooner than 2 seconds apart.
//
// Description: 
//    Arduino Mega 2560 board directly connected to MD25 motor controller (EMG30 motors), the MPU 6050 
//    IMU for yaw data, and an IR sensor for teleop control via a IR remote. This sketch uses
//    rosserial to create a node that publishes odometry data to ROS running on the Raspberry Pi 4 model B
//    SBC. A subscriber will be added soon to get motor control command from gmapping SLAM path planning/control
//    in the near future. The RPi is connected to the YDLidar G2 sensor and a video cam for QR code or ARTag
//    recognition. A remote laptop connects to the RPi via a VNC wifi connection for RVIZ display and various 
//    mode data communication support.
//
// Note:
//    Code only works when using MEGA2560 due to multiple serial ports and pin assignments
//    RPi 4B is running with:
//    The Ubiquity Robotics Raspberry Pi images are based on Ubuntu 16.04. They are pre-installed 
//    with ROS Kinetic. Source: https://downloads.ubiquityrobotics.com/pi.html
//
// Issues:
//    *Running rosserial script on RPi to acquire published odometry data from the Arduino will give an
//     error pertainng to buffer overflow and lose sync with the serial connection between RPi 4B and Arduino.
//     To fix this, the buffer size in two Arduino/libraries/ros_lib files need to be changed from 512 to 1024
//     (ros.h & ros/node_handle.h) as described here: 
//     https://answers.ros.org/question/73627/how-to-increase-rosserial-buffer-size/
//
//-------------------------------------------------------------------------------------
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include "common.h"
#include "md25.h"
#include <I2C.h>

#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>


//#define DEBUG 1

ros::NodeHandle  nh;
nav_msgs::Odometry msg;  

ros::Publisher odom_pub("odom", &msg);
tf::TransformBroadcaster odom_broadcaster;

ros::Time current_time, last_time;

// Funtion Declarations
void setUpIMU(void);
double getGyroHeading(double);
void calcSendOdomData();
//bool getIRValue(void);

void cmd_vel_cb(const geometry_msgs::Twist & msg) {
  // Read the message. Act accordingly.
  // We only care about the linear x, and the rotational z.
  float x = msg.linear.x;
  float z_rotation = msg.angular.z;

  // Decide on the morot state we need, according to command.
  if (x > 0 && z_rotation == 0) {       // Move Forward
    goForward(MotorRunTime,leftMotorSpeedAdj, rightMotorSpeedAdj);  // non-delay command (other than 10ms)
    pointTurn_G = false;
  }
  else if (x < 0 && z_rotation == 0) {  // Move Backward
    goBackward(MotorRunTime,leftMotorSpeedAdj, rightMotorSpeedAdj);
    pointTurn_G = false;
  }
  else if (x == 0 && z_rotation > 0) {  // Turn Left
    stopMotors();
    zeroEncoders();
    turnTowardsObject(adjust_left, 50, TurnSpeed); 
    pointTurn_G = true;
  }
  else if (x == 0 && z_rotation < 0) {  // Turn Right
    stopMotors();
    zeroEncoders();
    turnTowardsObject(adjust_right, 50, TurnSpeed); 
    pointTurn_G = true;
  }
  else {
    stopMotors();
    pointTurn_G = false;
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_cb);

void setup() {
 // initialize serial communication
    int i = 0;

    pinMode(IR_RCVR_PIN,INPUT);
    pinMode(ROSSERIAL_ENABLE_PIN,INPUT);
    rsEnable = digitalRead(ROSSERIAL_ENABLE_PIN);

    if (rsEnable == HIGH) 
       Serial.begin(57600);  // Serial comm. link with RPi
    else 
       receiver.enableIRIn(); // IR Remote receiver enable interrupts
  
    
    

    //-------------------------------------------------------------------------
    //                        STATUS LEDs Setup Section
    //-------------------------------------------------------------------------
    
    pinMode(SYST_GO_BLUE_PIN,OUTPUT);

    // Output Pin Init Values
    digitalWrite(SYST_GO_BLUE_PIN,LOW);
    

    //-------------------------------------------------------------------------
    //                           MD25 Setup Section
    //-------------------------------------------------------------------------
    // configure system status io pins
 
    // Set serial communication baud rate between arduino & MD25 motor controller
    md25.begin(38400);
    
    // Set mode to 2, Both motors controlled by writing to speed 1: *** For now.
    md25.write(CMD);                                            
    md25.write(SETMODE);
    md25.write(byte(0x00)); //2);              //(byte (0x00));
    delay(5);
    
    // Enable Regulator Feedback for encoder feedback to regulate power of motors
    md25.write(CMD);                                            
    md25.write(ENABLE_REGULATORS);
    delay(5);
    
    //
    md25.write(CMD);                         // Set MD25 accelleration value
    md25.write(WRITEACCEL);
    md25.write(MD25_Accel_Rate);             // From 1 to 10 steps per 25 ms
    delay(5);

    // Allow motors to keep turning after 2 seconds of non-communication with MD25 Motor Controller board
    md25.write(CMD);                                            
    md25.write(DISABLE_TIMEOUT);
    delay(5);

    byte  volts_byte  = readVolts();
    int   volts_whole = volts_byte/10;
    int   volts_fract = volts_byte%10;
    float volts = volts_whole + volts_fract/10;
    
    #ifdef DEBUG 
      //Serial.begin(57600);  // Serial comm. link with RPi
      
      Serial.print("Volts byte = ");
      Serial.print(volts_byte);
      Serial.print(", Volts = ");
      Serial.print(volts);
      Serial.print(", Volts fraction = ");
      Serial.println(volts_fract);
      Serial.print("\n");
      //readVI();
    #endif

   
    //------------- setup IMU ---------------
    setUpIMU();
    
    // IMU average drift calculation

    // calculate average drift rate
    for (int i=0; i<200; i++)  { // 2 second sample
       delay(20);
       while (i2cRead(0x47,i2cBuffer,2));
       gyroZ     = ((i2cBuffer[0] << 8) | i2cBuffer[1]) - gyroZOffset;
       gyroRate += (double)gyroZ/131.0; // Convert to deg/s
    }
    gyroTimer = micros();
    avgGyroDriftRate = (gyroRate/200); 
    
    #ifdef DEBUG
      Serial.print("Gyro Is Calibrated. Avg drift rate = ");
      Serial.print(avgGyroDriftRate);
      Serial.println(" deg/s \n");
      //debugTimer = micros();
    #endif
    
    // Get current azimuth angle with respect to start angle
    gyroHeading   = getGyroHeading(gyroHeading);       // get new heading, factor in drift over time, and limit to +/- 180 degrees
    loopTimer     = gyroTimer;              // gyroTimer is set in gettGyroAngle() function
    targetHeading = gyroHeading; 
   
    #ifdef DEBUG
      Serial.print("gyroHeading = ");
      Serial.print(gyroHeading);
      Serial.println(" deg\n");
    #endif

   
    zeroEncoders();
    
    delay(10);

  if (rsEnable == HIGH) {
     nh.initNode();
     //nh.getHardware()->setBaud(57600);
     nh.advertise(odom_pub);  // advertise publisher to roscore
     nh.subscribe(sub);

     pt_a = readEncoders();
  
     current_time = nh.now();
     last_time = nh.now();
     last_tnsec = last_time.nsec; 

     pointTurn_G = false;
     calcSendOdomData();
  }

 

  // Publisher & Subscriber Init Indicator
  digitalWrite(SYST_GO_BLUE_PIN,HIGH);
}


void loop() {
    
  calcSendOdomData();
  if (rsEnable == HIGH) 
    calcSendOdomData();
  else {
    getIRValue(); //test_drive_array[i]);  // rosserial func: calcSendOdomData(); embedded in case statement
    delay(80);
  }

}
  

//---------------------------------------------------------------------

void setUpIMU(void){

  /* Setup IMU */
    Wire.begin();

    i2cBuffer[0] = 19;   // Set the sample rate to 400Hz - 8kHz/(19+1) = 400Hz
    i2cBuffer[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cBuffer[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cBuffer[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g

    while (i2cWriteL(0x19,i2cBuffer,4,false)); // Write to all four registers at once
    while (i2cWriteS(0x6B,0x09,true)); // PLL with X axis gyroscope reference, disable temperature sensor and disable sleep mode
    while (i2cRead(0x75,i2cBuffer,1));
    if (i2cBuffer[0] != 0x68) { // Read "WHO_AM_I" register
      #ifdef DEBUG
      Serial.print("Error reading sensor");
      #endif
      while (1); // Halt
    }

    delay(10); // Wait for the sensor to get ready

  
}



//************************************************************************
// Convert current heading of robot to value between 180 to -180 degrees 
//************************************************************************
double convertHeadingToRange(double heading)  {
  while((heading >= 180) || (heading <= -180)) {  // works for this test case but needs more logic
     #ifdef DEBUG
     Serial.print("\n***Heading Converted.*** Was: ");
     Serial.print(heading);
     #endif
     if(heading <= -180)
        heading = heading + 360;
     else
        heading = heading - 360;
        
     #ifdef DEBUG
     Serial.print(", now it is: ");
     Serial.println(heading);
     Serial.print("\n");
     #endif
  }
  
  return heading;
}


//-------------------------------------------
// Get current gyro Z axis (azimuth) angle 
//-------------------------------------------
double getGyroHeading(double currentGyroHeading) {
  
  double dt = 0.0;
  //double currentGyroHeading = 0.0; // use if you want to zero out heading
  
  // Get change in gyro Z axis (azimuth) angle from last reading
  while (i2cRead(0x47,i2cBuffer,2));
  gyroZ = ((i2cBuffer[0] << 8) | i2cBuffer[1]) - gyroZOffset;
    
  gyroRate = (double)gyroZ/131.0; // Convert to deg/s

  vth = gyroRate*PI/180;
  
  //eliminate drift
  //if (abs(gyroRate) < gyroDriftRate)
  //   gyroRate = 0.0;
  dt = ((double)(micros()-gyroTimer)/1000000.0);
    
  //eliminate drift
  currentGyroHeading += ((gyroRate - avgGyroDriftRate)*dt); 

  gyroTimer = micros();                           // set timer for last time function called : value used globablly
        
  currentGyroHeading = convertHeadingToRange(currentGyroHeading);
/*
  #ifdef DEBUG
     Serial.print("In getGyroHeading, dt = ");
     Serial.println(dt);
     Serial.print("In getGyroHeading, Heading = ");
     Serial.println(currentGyroHeading);
  #endif
 */
  return currentGyroHeading;    
}

void calcSendOdomData()  {

    gyroHeading   = getGyroHeading(gyroHeading);       // get new heading, factor in drift over time, and limit to +/- 180 degrees
    theta = gyroHeading*PI/180;

    if (!pointTurn_G) {
       pt_b = readEncoders();  // need to tf this value
       dl = (double) ((pt_b - pt_a) * 0.04 * PI)/360;  // 1 rev = 360 ticks = 2piR = pi*D, where D = 4cm
    }
    else  {
       // zero encoders
       // set all x, y related values to zero
       pt_a = readEncoders();  // should be zero
       pt_b = pt_a;
       dl = 0;
    }
  
    dx = dl * cos(theta);         // x coord
    dy = dl * sin(theta);         // y coord 

    
    x += dx;
    y += dy;

    current_time = nh.now();
    
    //compute odometry in a typical way given the velocities of the robot
    curr_tnsec = current_time.nsec; 
  
    dt = (curr_tnsec - last_tnsec)/1000000000;
    
    vx  = dx * dt;
    vy  = dy * dt;
    //vth is calculated in getGyroHeading();

    #ifdef DEBUG
      Serial.print("gyroHeading = "); Serial.print(gyroHeading); Serial.println(" deg\n");
      Serial.print("curr_tnsec = "); Serial.print(curr_tnsec); Serial.println(" nsecs\n");
      Serial.print("last_tnsec = "); Serial.print(last_tnsec); Serial.println(" nsecs\n");
      Serial.print("dt     = "); Serial.print(dt); Serial.println(" secs\n");
      Serial.print("x      = "); Serial.print(x); Serial.println(" ticks\n");  //360 ticks/rotation
      Serial.print("y      = "); Serial.print(y); Serial.println(" ticks\n");
      Serial.print("theta  = "); Serial.print(theta); Serial.println(" radians\n");
      Serial.print("vx     = "); Serial.print(vx); Serial.println(" ticks/s\n");  //360 ticks/rotation
      Serial.print("vy     = "); Serial.print(vy); Serial.println(" ticks/s\n");
      Serial.print("vth    = "); Serial.print(vth); Serial.println(" radians/s\n\n");
    #endif


    //since all odometry is 6DOF we'll need a quaternion created from yaw
    //geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
    geometry_msgs::Quaternion q;
    //q.x = 0;
    //q.y = 0;
    //q.z = sin(theta * 0.5);
    //q.w = cos(theta * 0.5);

    q = tf::createQuaternionFromYaw(theta);

    msg.header.stamp = nh.now();
    msg.header.frame_id = "odom";
    msg.child_frame_id  = "base_link";

    // nav_msgs::Odometry - header, child_frame_id, pose (geometry_msgs::PoseWithCovariance), twist
    //                                              pose = position (geometry_msgs::Point) float x, y, z, & covariance
    //                                                   & orientation (geometry_msgs::Quaternion) float x, y, z, w
    //                                              twist = linear (geometry_msgs::Vector3) 
    //                                                   & angular (geometry_msgs::Vector3)                                                 

    //set pose (geometry_msgs::PoseWithCovariance) position - Point
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;

    //set pose orientatiaon
    //msg.pose.orientation.x = x;
    //msg.pose.orientation.y = y;
    //msg.pose.orientation.z = 0.0;
    //msg.pose.orientation.w = 0.0;
    
    //replaces above orientation x,y,z,w above
    msg.pose.pose.orientation = q;  //tf::createQuaternionFromYaw(theta);

    msg.twist.twist.linear.x = vx; //x;
    msg.twist.twist.linear.y = vy; //y;
    msg.twist.twist.linear.z = 0.0;

    //set the velocity
    msg.twist.twist.angular.x = 0.0; //vx;
    msg.twist.twist.angular.y = 0.0; //vy;
    msg.twist.twist.angular.z = vth;
    //msg.twist.angular.w = theta;

    if (rsEnable == HIGH) {  // rosserial logic switch set to enabled?
      //publish the message
      odom_pub.publish(&msg);
    }
    
    last_tnsec = curr_tnsec;
    pt_a = pt_b;

    if (rsEnable == HIGH) {  // rosserial logic switch set to enabled?
      nh.spinOnce();
    }

    if (rsEnable == HIGH) // rosserial switch enabled?
      delay(rosTranDelay);
    else
      delay(100); 
}

void getIRValue() { 
  
  unsigned int value; 
  
  if (receiver.decode(&output)) {
    value = output.value;
    
    switch (value) {
         case zero:  // zero encoders
           // Code
           zeroEncoders();                // non-delay command (other than 10ms)
           printIRRemoteDecodeValue(0);
           break;
           
         case one:   // increment left motor
           // Code
           leftMotorSpeedAdj += motorSpeedIncDecAmt;
           printIRRemoteDecodeValue(1);
           break;
           
         case two:   // move forward (continuous)
           // Code
           goForward(MotorRunTime, leftMotorSpeedAdj, rightMotorSpeedAdj);  // non-delay command (other than 10ms)
           //stopMotors();
           printIRRemoteDecodeValue(2);
           break;
           
         case three: // increment right motor
           // Code
           rightMotorSpeedAdj += motorSpeedIncDecAmt;
           break;
           
         case four:  // point turn left - long
           // Code 
           stopMotors();               // non-delay command (other than 10ms)
           turnTowardsObject(adjust_left, MotorRunTime, TurnSpeed); 
           printIRRemoteDecodeValue(4);
           break;
           
         case five:   // stop motors
           // Code
           stopMotors();
           printIRRemoteDecodeValue(5);
           break;
           
         case six:    // point turn right - long
           // Code
           stopMotors();               // non-delay command (other than 10ms)
           turnTowardsObject(adjust_right, MotorRunTime, TurnSpeed); 
           printIRRemoteDecodeValue(6);
           break;
           
         case seven:  // decrement left motor speed
           // Code
           leftMotorSpeedAdj -= motorSpeedIncDecAmt;
           printIRRemoteDecodeValue(7);
           break;
           
         case eight:   // move backwards (continuous)
           // Code
           goBackward(MotorRunTime, leftMotorSpeedAdj, rightMotorSpeedAdj);
           //stopMotors();
           printIRRemoteDecodeValue(8);
           break;
           
         case nine:   // decrement right motor speed
           // Code
           rightMotorSpeedAdj -= motorSpeedIncDecAmt;
           printIRRemoteDecodeValue(9);
           break;  
           
         default:
           // Code
           stopMotors();
           Serial.print("Key Select = Unknown!!!\n");
           break;
       } // end of switch-case

      receiver.resume();
  }
  
}

void printIRRemoteDecodeValue(unsigned int num) {
  
    Serial.print("Key Select = ");
    Serial.print(num);
    Serial.println();
    
}
