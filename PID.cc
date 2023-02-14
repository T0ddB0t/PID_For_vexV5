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
// Drivetrain           drivetrain    1, 10, 11, 20   
// TEST1                motor         2               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>
using namespace vex;
using namespace std;
// A global instance of competition
competition Competition;
// Code that the author wrote
class pid {
public:
  double out;
  double out1;
  double pre_er;
  void pidFWBK(double sp, double pv, double Kp, double Ki, double Kd /*, double max, double min*/) {
    // do i try a if statement to see if the LEFT/RIGHT encoders are equal?
    // I would find the diffrence and add || subtract it to the er
    if (LEFT.position(degrees) != RIGHT.position(degrees)) {
    }
    while (pv > sp) {
      double er = sp - pv;
      double integ = 0;
      integ += er;
      double der = (er - pre_er);
      // final step
      double Pout = Kp * er;
      double Iout = Ki * integ;
      double Dout = Kd * der;
      double uT = Pout + Iout - Dout;
      // Makes the bot go back or forward
      /*
     L1.spinFor(forward, uT, degrees);
     R1.spinFor(forward, -uT, degrees);
     R2.spinFor(forward, -uT, degrees);
     L2.spinFor(forward, uT, degrees);
     */
      Drivetrain.driveFor(uT, inches);
      pre_er = er;
    }

  }
  void pidSIDE(int sp, /*int pv*/ double Kp, double Ki, double Kd /*,double max, double min*/) {
    double er = sp - CENTER.position(degrees);
    double integ;
    double der;
    double uT = 0;
    while (fabs(er) > uT) {
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
      double uT = Pout + Iout + Dout;
      // makes the bot turn
      /*
      R1.spinFor(forward, uT,degrees),
      L2.spinFor(forward, uT,degrees),
      L1.spinFor(forward, uT,degrees),
      L2.spinFor(forward, uT,degrees);
      */

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
  LEFT.setPosition(0, degrees);
  RIGHT.setPosition(0, degrees);
  CENTER.setPosition(0, degrees);
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
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

  // pidFW,sp,pv,Kp,Ki,Kd,max,min)
  // pidSI,sp,pv,Kp,Ki,Kd,max,min)
  pid p;
  // problem is prob because it doesnt have a thing to check wether or not it
  // done executing like the drivetrain class functions
  // p.pidFWBK(10, 0, .1, .001, 5.5);
  p.pidSIDE(90, .1, .001, 5.5);
  //TEST1.spinFor(90, degrees);
  //Drivetrain.turnFor(left, 90, degrees);
  // p.pidFWBK(10, 0, .1, .001, 5.5);
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
  //pre_auton();
  auton();
  
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
