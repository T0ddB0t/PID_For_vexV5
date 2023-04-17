/*----------------------------------------------------------------------------*/
/*    Module:       main.cpp                                                  */
/*    Author:       Franky                                                    */
/*    Created:      Thu Sep 26 2022                                           */
/*    Description:  ARGGGGGGGGGGGGGGGGGG                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name?]               [Type?]        [Port(s)??????]
// CENTER               encoder       A, B            
// TEST1                motor         14              
// TEST2                motor_group   2, 3            
// IMU                  inertial      18              
// Controller1          controller                    
// Drivetrain           drivetrain    1, 10, 11, 20   
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <math.h>
//#include <iostream>
using namespace vex;
//using namespace std;
// A global instance of competition
competition Competition;
// Code that the author wrote
class PID{  
public:
//Get the encoder PID to work then the IMU sensor
  float Kp;
  float Ki;
  float Kd;

  void pidFWBK(double sp, double speed) {
    //just have the encoder in the middle of the bot
    CENTER.setPosition(0, rev);
    double er;
    //double pv; 
    //double i;
    double integ;
    double der;
    double pre_er;
    double uT = 0;
    double pv = CENTER.position(rev) * 1/13.1875/25.4;
    //sp *= (1/13.1875);
    if(sp > pv){
      //sp *= ((1/13.1875)/25.4);
      while(sp > pv){
        er = sp - pv;
        integ = integ + er;
        if(er == 0 || fabs(er) <= sp){
			    integ = 0;
		    }
        der = er - pre_er;
        // final step
        // Brain.Screen.print(er);
        double Pout = Kp * er;
        double Iout = Ki * integ;
        double Dout = Kd * der;
        uT = Pout + Iout + Dout;
        // makes the move
        pre_er = er;
      
        Brain.Screen.print(uT);
        Brain.Screen.print(" ");
        Drivetrain.setDriveVelocity(speed, rpm);
        Drivetrain.driveFor(uT, inches);
        if(sp == CENTER.position(rev)){
          wait(60, msec);
          if(sp == CENTER.position(rev)){
            break;
          }
        }
      }
    }
    if(sp < pv){
      while(sp < pv){
        er = sp - pv;
        //sp *= (1/13.1875);
        integ = integ + er;
        if(er == 0 || fabs(er) <= sp){
			    integ = 0;
		    }
        der = er - pre_er;
        // final step
        // Brain.Screen.print(er);
        double Pout = Kp * er;
        double Iout = Ki * integ;
        double Dout = Kd * der;
        uT = Pout + Iout + Dout;
        // makes the move
        pre_er = er;

        Brain.Screen.print(uT);
        Brain.Screen.print(" ");
        Drivetrain.setDriveVelocity(speed, rpm);
        Drivetrain.driveFor(uT, inches);
        //wait(5, msec);
        //TEST2.turnFor(right, uT, rev);
        if(sp == CENTER.position(rev)){
          wait(60, msec);
          if(sp == CENTER.position(rev)){
            break;
          }
        }
      }
    }
  }
  //going to use the IMU sensor for turning 
  void pidSIDE(double sp, double speed){
    double er = sp - IMU.angle(degrees);
    double integ;
    double der;
    double pre_er;
    double uT = 0;
    
    while (sp > IMU.angle(degrees)){
      er = sp - IMU.angle(degrees);
      integ = integ + er;
      if(er == 0 || fabs(er) >= sp){
			  integ = 0;
		  }
      der = (er - pre_er);
      // final step
      pre_er = er;
      double Pout = Kp * er;
      double Iout = Ki * integ;
      double Dout = Kd * der;
      uT = Pout + Iout + Dout;
      // makes the bot turn

      Brain.Screen.print(uT);
      Brain.Screen.print(" ");
      Drivetrain.turnFor(uT, degrees);
      if(sp == IMU.angle(rev)){
        break;
      }
    }
  }
};
// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
PID pid;
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ... 
  //TEST1.setPosition(0, rev);
  CENTER.setPosition(0, rev);
  IMU.calibrate();
  pid.Kp = 2.3f;
  pid.Ki = 1.3f;
  pid.Kd = 2.0f;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void auton(void) {
  pre_auton();
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  // pid.pidFWBK(setpoint, speed);
  // pid.pidSIDE(setpoint, speed); 

  pid.pidFWBK(300, 70);
  //pid.pidFWBK(0, 50);
  //Drivetrain.driveFor(forward, 10, mm);
  while (1) {
    wait(20, msec); // Sleep the task for a short amount of time to prevent wasted resources.
  }
}
// driver stuff   //
void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    wait(20, msec); // Sleep the task for a short amount of time to prevent wasted resources.
  }
}

// Main will set up the competition functions and callbacks.
int main() {
  // Set up callbacks for autonomous and driver control periods.
  //Competition.autonomous(auton);
  //Competition.drivercontrol(usercontrol);
  // Run the pre-autonomous function.
  //usercontrol();
  pre_auton();
  auton();
  
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
