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
// [Name]               [Type]        [Port(s)]
// EncoderA             encoder       A, B            
// Drivetrain           drivetrain    6, 16, 5        
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "vex.h"
#include <math.h>
//#include <iostream>
using namespace vex;
//using namespace std;
// A global instance of competition

competition Competition;

// Code that the author wrote
// void TurnLeft(){
//   motor Left = motor(PORT6, ratio18_1, false);
//   motor Right = motor(PORT16, ratio18_1, false);
//   Drieb = motor_group(Left, Right);
//   extern motor_group Drieb;
// }
// void TurnRight(){
//   motor Left = motor(PORT6, ratio18_1, true);
//   motor Right = motor(PORT16, ratio18_1, true);
//   Drieb = motor_group(Left, Right);
//   extern motor_group Drieb;
// }
// void Forward(){
//   motor Left = motor(PORT6, ratio18_1, true);
//   motor Right = motor(PORT16, ratio18_1, false);
//   Drieb = motor_group(Left, Right);
// }
class PID{
private:
  double pv;
  double er;
public:
//Get the encoder PID to work then the Drivetrain sensor
  double Kp;
  double Ki;
  double Kd;
  void pidFWBK(double sp, double speed) {
    //just have the encoder in the middle of the bot
    CENTER.setPosition(0, rev);
    //Forward();
    //double pv; 
    //double i;
    double integ = 0;
    double der;
    double pre_er = 0;
    double uT = 0;
    pv = CENTER.position(rev); //* (13.1875 * 25.4); // 10);
    //sp *= (1/13.1875);
    if(sp != pv){
      //sp *= ((1/13.1875)/25.4);
      while(sp != pv){
        pv = CENTER.position(rev); //* (13.1875 * 25.4);// 10);
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
        Drieb.setVelocity(speed,rpm);
        // if(CENTER.position(rev) > sp){
          // if(RIGHT.position(rev) && LEFT.position(rev) > sp){
        Drieb.spinFor(uT,turns);
          // }
          // wait(20, msec);
        // }
        //if(sp == pv){
          //wait(4, sec);
          //if(sp == pv){
          //  break;
        //  }
        //}
        pv = CENTER.position(rev);
        wait(20, msec);
        //break;
      }
    }
    // if(sp < pv){
    //   while(sp < pv){
    //     pv = CENTER.position(rev); //* (13.1875 * 25.4); // 10);
    //     er = sp - pv;
    //     //sp *= (1/13.1875);
    //     integ = integ + er;
    //     if(er == 0 || fabs(er) <= sp){
		// 	    integ = 0;
		//     }
    //     der = er - pre_er;
    //     // final step
    //     // Brain.Screen.print(er);
    //     double Pout = Kp * er;
    //     double Iout = Ki * integ;
    //     double Dout = Kd * der;
    //     uT = Pout + Iout + Dout;
    //     // makes the move
    //     pre_er = er;

    //     Brain.Screen.print(uT);
    //     Brain.Screen.print(" ");
        
    //     Drieb.setVelocity(speed,rpm);
    //     //if(CENTER.position(rev) < sp){
    //     //  if(LEFT.position(rev) && RIGHT.position(rev) < sp){  
    //     Drieb.spinFor(forward ,uT, rev);
    //     //  }
    //     //  wait(20, msec);
    //     // }
    //     //wait(5, msec);
    //     //TEST2.turnFor(right, uT, rev)
    //     if(sp == CENTER.position(rev) && sp == uT){
    //       wait(60, msec);
    //       if(sp == CENTER.position(rev) && sp == uT){
    //         break;
    //       }
    //     }
    //     pv = CENTER.position(rev);
    //     wait(20, msec);
    //     //break;
    //   }
    // }
  }
  //going to use the Drivetrain sensor for turning 
  void pidTURN(double sp, double speed){
    Drivetrain.setHeading(0, degrees);
    pv = Drivetrain.heading(degrees);
    er = sp - pv;
    Drieb.setVelocity(speed, rpm);
    double integ;
    double der;
    double pre_er;
    double uT = 0;
    while (sp != Drivetrain.heading(degrees)){
      pv = Drivetrain.heading();
      er = sp - pv;
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
      int x = sp;
      float percent = (1 * ((x / 100) * 360 ) / 9);
      if(sp > 0){
        uT -= percent;
        Drivetrain.turnFor(uT, degrees);
      }else{
        uT += percent;
        Drivetrain.turnFor(uT, degrees);
      }
      // if(sp == Drivetrain.heading(degrees) || sp == uT){
      //   wait(4, sec);
      //   if(sp == Drivetrain.heading(degrees) && sp == uT){
      //     break;
      //   }
      // }
      wait(20, sec);
    }
    // while (sp < Drivetrain.angle(degrees)){
    //   er = sp - Drivetrain.angle(degrees);
    //  integ = integ + er;
    //   if(er == 0 || fabs(er) >= sp){
		// 	  integ = 0;
		//   }
    //   der = (er - pre_er);
    //   // final step
    //   pre_er = er;
    //   double Pout = Kp * er;
    //   double Iout = Ki * integ;
    //   double Dout = Kd * der;
    //   uT = Pout + Iout + Dout;
    //   // makes the bot turn
    //   Brain.Screen.print(uT);
    //   Brain.Screen.print(" ");
    //   int x = sp;
    //   float percent = 4 * (((x / 100) * 250 ) / 9);
    //   Drivetrain.turnFor(uT + percent, degrees);
    //   // if(sp == Drivetrain.angle(degrees) && sp == uT){
    //   //   wait(4, sec);
    //   //   if(sp == Drivetrain.angle(degrees) && sp == uT){
    //   //     break;
    //   //   }
    //   //}
    //   wait(20,sec);
    // }
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
  pid.Kp = .59f;
  pid.Ki = .41f;
  pid.Kd = .45f;
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
  // pid.pidFWBK(setpoint || moving fwrd or back, speed);
  // pid.pidTURN(setpoint || Turning, speed); 
  //pid.pidFWBK(1, 40);
  pid.pidTURN(-90, 10);
  // int x = 90;
  // float percent = ((x / 100) * 250 ) / 9;
  // Drivetrain.turnFor(left, x - 25, degrees);
  //pid.pidFWBK(-10, 70);
  //pid.pidFWBK(0, 50);
  //Drivetrain.driveFor(forward, 10, mm);
  while (1) {
    //Brain.Screen.print(Drivetrain.heading());
    //Brain.Screen.clearScreen();
    wait(6, sec); // Sleep the task for a short amount of time to prevent wasted resources.
    //Brain.Screen.clearScreen();
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
