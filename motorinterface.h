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
 * @class FieldInterface
 * @brief Interacts with field controller and temperature sensor
**/
class FieldInterface {
public:
	FieldInterface(double min_v,
		double slow_max_v, double fast_max_v,
		long rpm_lower_limit, long rpm_upper_limit, 
		double overheat_temperature, int p_temp_indicator, Thermistor& thermistor); //E! what should min actually be?
	void sendCmd(int cmd);
	void setMaxVoltage(int motor_rpm);
	int getLastFieldInput();

private:
	void checkOverheat();

	double _min_v, _max_v;
	double _slow_max_v, _fast_max_v;

	long _rpm_lower_limit, _rpm_upper_limit;

	Thermistor& _thermistor;

	double _temperature, _overheat_temperature;
	int _p_temp_indicator;

	int _last_field_input;

};

// ----------------------------------------------------------------------------------


/**
 * @class MotorInterface
 * @brief Handles rpm, field, and braking commands. Interface with Kelly Controller and Field Controller
**/
class MotorInterface {
public:
	MotorInterface(int p_motor_pwm, int p_regen_pwm, int p_reverse_switch, int regen_min_field, FieldInterface& field, PIDController& motor_pid, SpeedSensor& encoder);
	void handleCmds(int motor_cmd, int field_cmd, int regen_cmd);
	void usePID(bool pid_on);
	
	int getLastMotorInput();	
	int getLastFieldInput();	
	int getLastRegenInput();
	int getLastRPM();	
private:
	void handleField();
	void handleRegen();
	void handleMotor();

	void setReverseOn(bool reverse_on);

	PIDController& _motor_pid;
	SpeedSensor& _encoder;
	FieldInterface& _field;

	int _p_motor_pwm, _p_regen_pwm;
	int _p_reverse_switch;
	bool _reverse_on, _pid_on;

	int _regen_min_field;

	int _last_motor_cmd, _last_field_cmd, _last_regen_cmd;
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