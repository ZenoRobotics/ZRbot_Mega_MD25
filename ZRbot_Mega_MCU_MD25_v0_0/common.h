// File: common.h
//
//--------------------------------------------------------------------------
// Written by Peter J. Zeno, Ph.D. <zenorobotics@gmail.com>, October 2020
//
// Copyright (C) ZenoRobotics, LLC - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential.
//--------------------------------------------------------------------------
//
// Description

//#define DEBUG 1
#include <IRremote.h>

//++++++++++++++++++++++++++++++++++++++++++++
// Serial Port Definitions
//++++++++++++++++++++++++++++++++++++++++++++
/* Mega 2560:
 * Serial1 on pins 19 (RX) and 18 (TX), 
 * Serial2 on pins 17 (RX) and 16 (TX), 
 * Serial3 on pins 15 (RX) and 14 (TX)
 */
#define md25 Serial2     // only works when using MEGA2560

//----------------------------------------------
//----------    Pin Assignments   --------------
//----------------------------------------------
#define IR_RCVR_PIN                       11
#define ROSSERIAL_ENABLE_PIN              23  

//Uncomment if adding IR receiver
// Instantiate IR receiver
IRrecv receiver(IR_RCVR_PIN);
decode_results output;

//------------- STATUS LED PINS ---------------
#define SYST_GO_BLUE_PIN                  22


//----------------------------------------------
//------      End of Pin Assignments     -------
//----------------------------------------------

// MD25 Command/Status addresses/data
#define CMD                 (byte)0x00              //  MD25 command byte of 0

#define WRITESP1            0x31                    // Byte writes speed to motor 1
#define WRITESP2TURN        0x32                    // Byte writes turn value when in modes 2 & 3
#define WRITEACCEL          0x33                    // Byte writes a value of acceleration
#define RESETREG            0x35                    // Byte resets encoders
#define SETMODE             0x34                    // Byte sets mode
#define READIVS             0x2C                    // Byte reads motor currents and battery voltage        
#define READENCS            0x25                    // Byte reads both encoders
#define READENC1            0x23                    // Byte reads encoder1 
#define READENC2            0x24                    // Byte reads encoder2
#define GET_VER             0x29
#define GET_SPEED1          0x21                    // Byte reads motor 1's set speed
#define GET_SPEED2          0x22                    // Byte reads motor 2's set speed
#define DISABLE_REGULATORS  0x36
#define ENABLE_REGULATORS   0x37
#define DISABLE_TIMEOUT     0x38
#define ENABLE_TIMEOUT      0x39




//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//       IMU Data 
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;

/* Used for timing */
uint32_t gyroTimer;  // Timer used for determining gyro rate
uint32_t timer;

double gyroXangle, gyroYangle; // Angle calculate using the gyro
double targetHeading = 0.0;

// MPU-6050 Results
double   gyroRate           = 0.0;
double   gyroHeading        = 0.0;
double   avgGyroDriftRate   = 0.0; // deg/s

uint8_t i2cBuffer[8]; // Buffer for I2C data

//**************************************
//       Constants
//**************************************
const int16_t  gyroZOffset       =  -85;
const int    BarrierTurnTime     = 1100;   // ms
const int    MaxSpeed            =   25;
const int    TurnSpeed           =   20; 
const int    fwdSpeed            =   25;
const int    bwdSpeed            =   20;
const int    AdjustTurnTime      =  120;   // ms
const int    AdjustTurnSpeed     =    4;
const int    commDelay           =    0;   // number of milliseconds delay before continuing program after writing to motor controller
const int    MD25_Accel_Rate     =    2;
const int    MotorRunTime        =   50;

// rosserial tranmission rate/delay
const int    rosTranDelay        =   60;  //*ms delay added between each rosserial odometry transmission

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//       MD25 Variables
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
double   loopTimer               = 0.0;
int      leftMotorSpeedAdj       = fwdSpeed;
int      rightMotorSpeedAdj      = fwdSpeed;  
const int    motorSpeedIncDecAmt  =  5;  

//byte   volts_byte;
//int    volts_whole;
//int    volts_fract;
//float  volts;
int      inByte;

double   knownAngle              = 0.0;
long     distanceBeforeTurn      = 0.0;  // Set in turnLeftRight() function, just prior to turn.
long     retDistance             = 0;
long     currEncoderVal          = 0;
bool     Home                    = false;
bool     HeadingHome             = false;

bool pointTurn_G = false;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//        Samsung BN59 Remote Codes
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
const unsigned int zero  = 34935;
const unsigned int one   = 8415;
const unsigned int two   = 41055;
const unsigned int three = 24735;
const unsigned int four  = 4335;
const unsigned int five  = 36975;
const unsigned int six   = 20655;
const unsigned int seven = 12495;
const unsigned int eight = 45135;
const unsigned int nine  = 28815;


enum action_enum {
  stop_,
  go_straight,
  go_backward,
  turn_l_or_r,
  turn_left,
  turn_right,
  adjust_left,
  adjust_right
};

action_enum  nextAction = stop_;

// Rosserial Variables
double x = 0.0;
double y = 0.0;
double theta = 0.0;
double dx, dy, dl = 0.0;

double vx = 0.1;
double vy = -0.1;
double vth = 0.1;

long pt_a, pt_b = 0.0;

double curr_tnsec = 0.0; 
double last_tnsec = 0.0;
double dt         = 0.0;

int rsEnable = LOW; // ROSSERIAL: Low = not enabled, High = enabled
