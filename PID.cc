#include "PID.h"

void PID::pidFWBK(double sp, int speed){
    CENTER.setPosition(0, rev);
    double pv; 
    double integ = 0;
    double der;
    double pre_er = 0;
    double er;
    pv = CENTER.position(rev);
    if(sp != pv){
      while(sp != pv){
        pv = CENTER.position(rev); 
        er = sp - pv;
        integ = integ + er;
        if(er == 0 || fabs(er) <= sp){
			    integ = 0;
		    }
        der = er - pre_er;
        // final step
        // Brain.Screen.print(er);
        double Pout = *Kp * er;
        double Iout = *Ki * integ;
        double Dout = *Kd * der;
        *uT = Pout + Iout + Dout;
        // makes the move
        pre_er = er;
        
        Brain.Screen.print(uT);
        Brain.Screen.print(" ");
        Drivetrain.setDriveVelocity(speed,rpm);
        if(CENTER.position(rev) > sp){
          Drivetrain.driveFor(forward,*uT,inches);
          wait(20, msec);
        }
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
}

void PID::pidTURN(double sp, int speed){
    inert.setHeading(0, degrees);
    double pv = inert.heading(degrees);
    double er = sp - pv;
    double integ;
    double der;
    double pre_er;
    while (sp != pv){
      pv = inert.heading();
      er = sp - pv;
      integ = integ + er;
      if(er == 0 || fabs(er) >= sp){
			  integ = 0;
		  }
      der = (er - pre_er);
      // final step
      pre_er = er;
      double Pout = *Kp * er;
      double Iout = *Ki * integ;
      double Dout = *Kd * der;
      *uT = Pout + Iout + Dout;
      // makes the bot turn
      Brain.Screen.print(*uT);
      Brain.Screen.print(" ");
      int x = sp;
      float percent = (1 * ((x / 100) * 360 ) / 9);
      if(sp > 0){
        *uT -= percent;
        Drivetrain.turnFor(*uT, degrees);
      }else{
        *uT += percent;
        Drivetrain.turnFor(*uT, degrees);
      }
      // if(sp == Drivetrain.heading(degrees) || sp == uT){
      //   wait(4, sec);
      //   if(sp == Drivetrain.heading(degrees) && sp == uT){
      //     break;
      //   }
      // }
      wait(20, msec);
    }
}
