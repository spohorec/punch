/**
 * pidcontroller.h
 * Electric Kool-Aid  PID controller.
 * [adapted from 5yler/gigabug/pidcontroller.cpp]
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 *
 * @date    2016-01-10    syler   refactored header files
 * @date  	2017-07-09	  sarah   ported from gigabug, minor changes
 * @date 2017-07-14 	  sarah   Added header guards.
 **/

#ifndef __PIDCONTROLLER_H
#define __PIDCONTROLLER_H

//$ constants
// const static double RPM_TO_M_S = (2 * PI * wheelRadius) / 60.0;   //$ conversion from RPM to meters per second

// const static double STEERING_PWM_RANGE = 255.0;
// const static double STEERING_ANGLE_RANGE = 50 * (PI / 180); //$ [radians] this is the correct steering range
// const static double ABS_MAX_STEERING_ANGLE = 25 * (PI / 180); //$ [radians]

// ----------------------------------------------------------------------------------

class PIDController {
public:
  PIDController(long kp, long ki, long kd, long out_max, long out_min);
  int update(int ref, int in);
  void resetIntegrator();
  void resetGains(long kp, long ki, long kd);
private:
  long _kp, _ki, _kd;
  long _out_max, _out_min;
  
  int _last_in;
  long _integral;
};
  

#endif


