/**
* motorinterface.h
* Electric Kool-Aide Motor Controller-Controllers
* 
* 	@author Sarah Pohorecky <spohorec@mit.edu>
* 
* 	@date 2017-07-08 creation.
* 	@date 2017-07-13 update with reverse handling.
* 	@date 2017-07-13 refactor, hopefully more reasonably?
*	@date 2017-07-13 added ServoInterface outline
*   @date 2017-07-14 refactored with references. Bug fixing. Added header guards.
**/

#ifndef __MOTORINTERFACE_H
#define __MOTORINTERFACE_H

#include <Arduino.h>
#include "sensors.h"		
#include "pidcontroller.h"

#define V_BATTERY_MAX 33.6 //@def

// ----------------------------------------------------------------------------------


/**
 * @class MotorInterface
 * @brief Handles rpm and braking commands. Interface with Kelly Controller.
**/
class MotorInterface {
public:
  MotorInterface(int p_motor_pwm, int p_regen_pwm, int p_reverse_switch,PIDController& motor_pid, SpeedSensor& encoder);
	void handleCmds(int motor_cmd, int regen_cmd);
	void usePID(bool pid_on);
	void getLastInps(int *_return_array);
private:
	void handleRegen();
	void handleMotor();

	void setReverseOn(bool reverse_on);

	PIDController& _motor_pid;
	SpeedSensor& _encoder;

	int _p_motor_pwm, _p_regen_pwm;
	int _p_reverse_switch;
	bool _regen_on, _reverse_on, _pid_on;

	int _last_motor_cmd, _last_regen_cmd;
	int _last_motor_input, _last_regen_input;

	long _last_rpm;

};

// ----------------------------------------------------------------------------------

/**
 * @class ServoInterface
 * @brief interface with servo and angle sensor
**/

class ServoInterface {
public:
	ServoInterface(int p_servo_pwmA, int p_servo_pwmB, int p_servo_enable, AngleSensor& servo_pot, PIDController& servo_pid);
	void handleCmd(int cmd);
	void useServo(bool servo_on);
	void usePID(bool pid_on);

private:
	AngleSensor& _servo_pot;
	PIDController& _servo_pid;

	int _p_servo_pwmA, _p_servo_pwmB, _p_servo_enable;

	bool _servo_on, _pid_on;
	int _last_angle, _last_angle_command, _last_angle_input;

};

#endif
