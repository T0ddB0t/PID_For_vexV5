/*----------------------------------------------------------------------------*/
/*  Module:       main.cpp                                                    */
/*    Author:       Franky                                                    */
/*    Created:      Thu Sep 26 2022                                           */
/*    Description:  ARGGGGGGGGGGGGGGGGGG                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// RIGHT                encoder       A, B            
// LEFT                 encoder       C, D            
// CENTER               encoder       E, F            
// TEST1                motor         14              
// LEFT1                motor_group   1, 10           
// RIGHT1               motor_group   11, 20          
// Drivetrain           drivetrain    2, 3            
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>
using namespace vex;
using namespace std;
// A global instance of competition
competition Competition;
// Code that the author wrote
class PID{  
public:
//Get the encoder PID to work then the IMU sensor
  double out;
  double out1;
  void pidFWBK(double sp, double speed, double Kp, double Ki, double Kd) {
    // do i try a if statement to see if the LEFT/RIGHT encoders are equal?
    // I would find the diffrence and add || subtract it to the er
    double er = sp - TEST1.position(rev);
    double integ;
    double der;
    double pre_er;
    double uT = 0;
    while(sp > TEST1.position(rev)){
      er = sp - TEST1.position(rev);
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
      /*
      R1.spinFor(forward, uT,degrees),
      L2.spinFor(forward, uT,degrees),
      L1.spinFor(forward, uT,degrees),
      L2.spinFor(forward, uT,degrees);
      */
      pre_er = er;
      
      Brain.Screen.print(uT);
      //Brain.Screen.print(" ");
      if(sp == TEST1.position(degrees)){ 
        TEST1.setVelocity(0, pct);
        TEST1.spinFor(uT, rev);
      }else{
        TEST1.setVelocity(speed, pct);
        TEST1.spinFor(uT, rev);
      }
      //wait(5, msec);
      //Drivetrain.turnFor(right, uT, degrees);

    }
  }
  void pidSIDE(int sp, /*int pv*/ double Kp, double Ki, double Kd ,double max, double min) {
    double er = sp + CENTER.position(degrees);
    double integ;
    double der;
    double pre_er;
    double uT = 0;
    while (fabs(er) < sp) {
      er = sp - CENTER.position(degrees);
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
      /*
      R1.spinFor(forward, uT,degrees),
      L2.spinFor(forward, uT,degrees),
      L1.spinFor(forward, uT,degrees),
      L2.spinFor(forward, uT,degrees);
      */
      Brain.Screen.print(uT);
      Brain.Screen.print(" ");
      TEST1.spinFor(uT, degrees);
      wait(15, msec);
      //Drivetrain.turnFor(right, uT, degrees); 
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

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  //LEFT.setPosition(0, degrees);
  //RIGHT.setPosition(0, degrees);
  //CENTER.setPosition(0, degrees);
  TEST1.setPosition(0, degrees);
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
  // pidFWBK(sp,speed,Kp,Ki,Kd)
  // pidSIDE(sp,speed,Kp,Ki,Kd)
  PID pid;
  //pid.pidFWBK(10, 0, .1, .001, 5.5);
  pid.pidFWBK(30, 40, .4, .49, .45);
  //TEST1.spinFor(90, degrees);
  //Drivetrain.turnFor(left, 90, degrees);
  // pid.pidFWBK(10, 0, .1, .001, 5.5);
  while (1) {
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}
////////////////////
//  box of info   //
//                //
//                //
// driver stuff   //
////////////////////
void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    wait(20, msec); // Sleep the task for a short amount of time to prevent wasted resources.
  }
}

// Main will set up the competition functions and callbacks.
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(auton);
  Competition.drivercontrol(usercontrol);
  // Run the pre-autonomous function.
  //usercontrol();
  pre_auton();
  auton();
  
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
