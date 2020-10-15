//--------------------------------------------------------------------------
// Written by Peter J. Zeno, Ph.D. <zenorobotics@gmail.com>, October 2020
//
// Copyright (C) ZenoRobotics, LLC - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential.
//--------------------------------------------------------------------------
/**********************************************************
  File: md25.h 
  
   Description: This sketch interfaces and controls the 
   MD25 - 12v 2.8A dual H-bridge motor driver
   Be the first to review this product

   Designed to work with our EMG30 gear motors, the MD25 will drive two motors.
   Controllled by Serial or the I2C Interface. 
   Two modes of operation, direct individual control of the motors or the ability to send a speed and a turn commands. 

   Voltage - 12v only. The 5v for the logic supplied from an on-board regulator, 300mA available for your own circuits. 
   Current - Up to 2.8A for each motor. 
   Encoder - Processes quadrature encoder inputs from motor. 360 counts per wheel turn when used with the EMG30.
   Size - 110mm x 52mm x 25mm 

   Note: This code assumes that an Arduino 2560 is used so that there are multiple serial ports to use, other that the 
   console serial_0 port

   RD02 = MD25 + EMG30 + Hardware & tires

  
 **********************************************************/

#include <math.h>

#include <Wire.h>



/*
Note:
 Not all pins on the Mega and Mega 2560 support change interrupts,
 so only the following can be used for RX:
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

 Not all pins on the Leonardo and Micro support change interrupts,
 so only the following can be used for RX:
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).   
*/



//--------------------------------------------------
//    Function declarations
//--------------------------------------------------
//void turnTowardsObject(action_enum turnDirection, int turnTime, int turnSpeed);
//void goForward(int speedVal, int leftMod, int rightMod, int commDelay);
//void stopMotors();  
//void zeroEncoders();
//long readEncoder();
//byte readVolts();
//void readVI();


//***********************************************************************************
// Turns Left or Right For Sent Turn Time and Speed 
//***********************************************************************************
void turnTowardsObject(action_enum turnDirection, int turnTime, int turnSpeed)  {
// Left= -127 to -1; Right= 1 to 127;  Turn time in ms
     if ((turnDirection == adjust_left) || (turnDirection == turn_left)) {
        md25.write(CMD);                    
        md25.write(WRITESP1);
        md25.write(128-turnSpeed);
        delay(0);
        md25.write(CMD);                    
        md25.write(WRITESP2TURN);
        md25.write(128+turnSpeed);
     }
     
     else if ((turnDirection == adjust_right) || (turnDirection == turn_right)) {
        md25.write(CMD);                    
        md25.write(WRITESP1);
        md25.write(128+turnSpeed);        // set to 4 right now
        delay(0);
        md25.write(CMD);                    
        md25.write(WRITESP2TURN);
        md25.write(128-turnSpeed);        // set to 4 right now
     }

     delay(turnTime); //turnTime
     //stopMotors();
     
     // Get current azimuth angle with respect to start angle
//     gyroHeading = getGyroHeading(gyroHeading);       // get new heading, factor in drift over time, and limit to +/- 180 degrees
     
}


//***********************************************************************************
// Go Forward
//***********************************************************************************
void goForward(int commDelay, unsigned int lftMotorSpd, unsigned int rtMotorSpd) {
  
     // write values to motors
     // left motor?
     md25.write(CMD);                   
     md25.write(WRITESP1);
     md25.write(128+rtMotorSpd);
     delay(1);
     // right motor?
     md25.write(CMD);                    
     md25.write(WRITESP2TURN);
     md25.write(128+lftMotorSpd);
     delay(10); // commDelay

 
}

//***********************************************************************************
// Go Backward
//***********************************************************************************
void goBackward(int commDelay, unsigned int lftMotorSpd, unsigned int rtMotorSpd) {
  
     // write values to motors
     // left motor?
     md25.write(CMD);                   
     md25.write(WRITESP1);
     md25.write(128-rtMotorSpd);
     delay(0);
     // right motor?
     md25.write(CMD);                    
     md25.write(WRITESP2TURN);
     md25.write(128-rtMotorSpd);
     delay(commDelay);
 
}

//**************************************************************************
// Stop Motors
//**************************************************************************
void stopMotors() {
  goForward(0,0,0);         // Stop=0; Backwards= -127 to -1; Forwards= 1 to 127;
}

//***********************************************************************************
// Slight Turns Left or Right For Sent Turn Time and Speed 
//***********************************************************************************
void adjLeftRight(action_enum turnDirection, int turnTime)  {
//    if ((turnDirection == turn_left) || (turnDirection == turn_right)) {
      stopMotors();
      turnTowardsObject(turnDirection, BarrierTurnTime, TurnSpeed); // removed stop motors - BarrierTurnTime not used
      delay(turnTime);
      stopMotors();
      //goForward(fwdSpeed,0,0,10);     // never had stop motors command in function
//    }
//    else {
//      turnTowardsObject(turnDirection, BarrierTurnTime, TurnSpeed); // removed stop motors - BarrierTurnTime not used
//      delay(turnTime);
//    }
}

//**************************************************************************
// Zero out both Encoder Countts
//**************************************************************************
void zeroEncoders() {
  
   md25.write(CMD);                    
   md25.write(RESETREG);
   delay(1);
 
} 
 
//**************************************************************************
// Reading from Encoders messes up PID/Kalman cooperation.
//**************************************************************************
long readEncoders(){                        // Function to read and display the value of both encoders, returns value of first encoder

  long result1 = 0; 
  long result2 = 0;
  long avgDistance = 0;
  
  md25.write(CMD);
  md25.write(READENCS);
  while(md25.available() < 8){}          // Wait for 8 bytes, first 4 encoder 1 values second 4 encoder 2 values 
  result1 = md25.read();                 // First byte for encoder 1, HH.
  result1 <<= 8;
  result1 += md25.read();                // Second byte for encoder 1, HL
  result1 <<= 8;
  result1 += md25.read();                // Third byte for encoder 1, LH
  result1 <<= 8;
  result1  += md25.read();               // Fourth byte for encoder 1, LL
  result2 = md25.read();
  result2 <<= 8;
  result2 += md25.read();
  result2 <<= 8;
  result2 += md25.read();
  result2 <<= 8;
  result2 += md25.read();
  
  /*
  #ifdef DEBUG
  Serial.print("Encoder 1:");               // Displays data to the LCD03 screen
  Serial.print(result1,DEC);
  Serial.print(" ");                        // Print a blank space to clear any unwanted characters that are leftover on the LCD03 display
  
  delay(5);                                // Delay for LCD03 to process data
  
  Serial.print("Encoder 2:");
  Serial.print(result2,DEC);
  Serial.print(" ");
  #endif
 */ 
  return avgDistance = (result1 + result2)/2;                                   
}

long readEncoder1(){                        // Function to read and display the value of both encoders, returns value of first encoder

  long result1 = 0; 
  
  md25.write(CMD);
  md25.write(READENC1);
  while(md25.available() < 8){}          // Wait for 4 bytes: encoder 1
  result1 = md25.read();                 // First byte for encoder 1, HH.
  result1 <<= 8;
  result1 += md25.read();                // Second byte for encoder 1, HL
  result1 <<= 8;
  result1 += md25.read();                // Third byte for encoder 1, LH
  result1 <<= 8;
  result1  += md25.read();               // Fourth byte for encoder 1, LL
  
  /*
  #ifdef DEBUG
  Serial.print("Encoder 1:");               // Displays data to the LCD03 screen
  Serial.print(result1,DEC);
  Serial.print(" ");                        // Print a blank space to clear any unwanted characters that are leftover on the LCD03 display
  
  delay(5);                                // Delay for LCD03 to process data
  
  Serial.print("Encoder 2:");
  Serial.print(result2,DEC);
  Serial.print(" ");
  #endif
 */ 
  return result1;                                   
}


long readEncoder2(){                     // Function to read and display the value of encoder 2, returns value of first encoder

  long result2 = 0; 
  
  md25.write(CMD);
  md25.write(READENC1);
  while(md25.available() < 8){}          // Wait for 4 bytes: encoder 2 values 
  result2 = md25.read();                 // First byte for encoder 2, HH.
  result2 <<= 8;
  result2 += md25.read();                // Second byte for encoder 2, HL
  result2 <<= 8;
  result2 += md25.read();                // Third byte for encoder 2, LH
  result2 <<= 8;
  result2  += md25.read();               // Fourth byte for encoder 2, LL
  
  /*
  #ifdef DEBUG
  Serial.print("Encoder 1:");               // Displays data to the LCD03 screen
  Serial.print(result1,DEC);
  Serial.print(" ");                        // Print a blank space to clear any unwanted characters that are leftover on the LCD03 display
  
  delay(5);                                // Delay for LCD03 to process data
  
  Serial.print("Encoder 2:");
  Serial.print(result2,DEC);
  Serial.print(" ");
  #endif
 */ 
  return result2;                                   
}
  
  
byte readVolts(){                                                 // Function reads current for both motors and battery voltage
  byte batteryVolts, mot1_current, mot2_current = 0;
  md25.write(CMD);
  md25.write(READIVS);                                          // Send byte to readbattery voltage and motor currents
  while(md25.available() < 3){}                                 // Wait for the 3 bytes to become available then get them
  batteryVolts = md25.read();
  mot1_current = md25.read();
  mot2_current = md25.read();
  
  return batteryVolts;

}


void readVI(){                                                 // Function reads current for both motors and battery voltage
  byte batteryVolts, mot1_current, mot2_current = 0;
  md25.write(CMD);
  md25.write(READIVS);                                          // Send byte to readbattery voltage and motor currents
  while(md25.available() < 3){}                                 // Wait for the 3 bytes to become available then get them
  batteryVolts = md25.read();
  mot1_current = md25.read();
  mot2_current = md25.read();

  Serial.print("Mot1 I:");
  Serial.print(mot1_current,DEC);
  Serial.print(" Mot2 I:");
  Serial.print(mot2_current,DEC);
  Serial.print(" "); 
  
  delay(5);
  
  //Serial.print("Rev:");
  //Serial.print(softwareRev, DEC);
  Serial.print(" ");
  Serial.print("Volts:");
  Serial.print(batteryVolts/10,DEC);                               // Seperate whole number and descimal place of battery voltage and display
  Serial.print(".");  
  Serial.print(batteryVolts%10,DEC);
  Serial.println(" ");   
 
}

void getDisplayBatteryVoltage() {
    byte  volts_byte  = readVolts();
    int   volts_whole = volts_byte/10;
    int   volts_fract = volts_byte%10;
    float volts = volts_whole + volts_fract/10;

      Serial.print("Volts byte = ");
      Serial.print(volts_byte);
      Serial.print(", Volts = ");
      Serial.print(volts);
      Serial.print(", Volts fraction = ");
      Serial.println(volts_fract);
      Serial.print("\n");
     
}

void getSetSpeed(){                                                 // Function reads current for both motors and battery voltage
  byte currSetSpeed, mot1_speed, mot2_speed = 0;
  md25.write(CMD);
  md25.write(GET_SPEED1);                                          // Send byte to readbattery voltage and motor currents
  while(md25.available() < 1){}                                    // Wait for the 1 byte to become available then get them
  mot1_speed = md25.read();

  Serial.print("Mot1 Set Speed:");
  Serial.print(mot1_speed,DEC);
  Serial.print(" "); 
  
  delay(10);
  
  md25.write(CMD);
  md25.write(GET_SPEED2);                                          // Send byte to readbattery voltage and motor currents
  while(md25.available() < 1){}                                    // Wait for the 1 byte to become available then get them
  mot2_speed = md25.read();

  Serial.print("Mot2 Set Speed:");
  Serial.print(mot2_speed,DEC);
  Serial.println(" ");   
}
