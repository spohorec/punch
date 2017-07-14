/** 
 * acid.h
 * Electric Kool-Aid Acid Test Main Header
 * 
 * @author Sarah Pohorecky <spohorec@mit.edu> 
 *
 * @date 2017-07-08 creation
 * @date 2017-07-13 updates with new pinouts
 * @date 2017-07-14 some outlining
 * @date 2017-07-14 refactored with references. Bug fixing. Added header guards.
**/

#ifndef __ACID_H
#define __ACID_H

#include <Arduino.h>
#include "commander.h"
#include "motorinterface.h"
#include "sensors.h"
#include "pidcontroller.h"

#define TRIPPING true //@def I am funny hi

#define P_AUTONOMY_SWITCH 1 //@def Physical mode switch input pin

#define P_REGEN_PWM 10 //@def Regen output pin
#define P_BRAKE_1 7 //@def Physical brake switch 1 input pin
#define P_BRAKE_2 8 //@def Physical brake switch 2 input pin
#define REGEN_MIN_FIELD 7.0 //@def [V] Minimum field voltage needed for regen braking //E! TODO confirm

#define P_REVERSE_SWITCH 0 //@def Physical reverse switch input pin
#define P_SET_REVERSE 12 //@def Kelly reverse switch command output pin

//E Motor parameters	
#define P_MOTOR_PWM 9 ///@def Kelly motor command output pin
#define P_MOTOR_THROTTLE A0 //@def Motor throttle input pin
#define MOTOR_THROTTLE_MIN_V 0.85 //@def Motor throttle min voltage 
#define MOTOR_THROTTLE_MAX_V 4.35 //@def Motor throttle max voltage

//E Encoder parameters
#define P_ENCODER 2 //@def Encoder input pin
#define ENCODER_INTERRUPT 0 //@def Interrupt attached to motor encoder
#define PULSES_PER_REV 600.0 //@def Number of encoder pulses per full motor revolution //E! TODO Confirm this value for Kool-Aid

//E Field parameters
#define P_FIELD_PWM 11 //@def Field driver command output pin
#define P_FIELD_THROTTLE A1 //@def Field Throttle input pin
#define FIELD_THROTTLE_MIN_V 0.85 //@def Field throttle min voltage
#define FIELD_THROTTLE_MAX_V 4.35 //@def Field throttle max voltage

#define FIELD_MIN_V 5.0 //@def Minimum field voltage with 0% input
#define FIELD_SLOW_MAX_V 12.0 //@def Maximum field voltage with 100% input at low speed
#define FIELD_FAST_MAX_V 9.0 //@def Maximum field voltage with 100% input at high speed

#define FIELD_RPM_LOWER_LIMIT 20 //@def Limit of "low speed" //E! TODO Choose better value 
#define FIELD_RPM_UPPER_LIMIT 80 //@def Limit of "high speed" //E! TODO Choose better value

#define FIELD_OVERHEAT_TEMP 80.0 //@def [*C] Temperature at which to warn user //E! Choose better value
#define P_TEMP_INDICATOR 4 //@def Overheating indicator output pin

//E Thermistor parameters
#define P_THERMISTOR A3 //@def Thermistor input pin
#define THERM_REF_RES 10000.0 //@def [Ohms] Thermistor divider reference resistance
#define THERM_T0 25.0 //@def [*C] Thermistor baesline temperature
#define THERM_R0 4200.0 //@def [Ohms] Thermistor baseline resistance
#define THERM_B 1.0 //@def [gross unit] Thermistor B-parameter

//E Servo parameters
#define P_SET_SERVO 13 //@def MegaMoto enable command output pin
#define P_SERVO_PWM_A 6 ///@def Servo PWM A pin
#define P_SERVO_PWM_B 5 //@def Servo PWM B pin

#define P_SERVO_POT A2 //@def Servo pot input pin
#define SERVO_POT_MIN 0 //@def Minimum reading on pot //E! TODO
#define SERVO_POT_MID 512 //@def Centered reading on pot //E! TODO
#define	SERVO_POT_MAX 1023 //@def Maximum reading on pot //E! TODO

//E PID parameters
#define MOTOR_PID_OUT_MIN -255 //@def Minimum motor PID output cmd //E! TODO
#define MOTOR_PID_OUT_MAX 255 //@def Maximum motor PID output cmd //E! TODO
#define KP_MOTOR 1 //@def Motor PID proportional gain //E! TODO
#define KI_MOTOR 1 //@def Motor PID integral gain //E! TODO
#define KD_MOTOR 1 //@def Motor PID differential gain //E! TODO

#define SERVO_PID_OUT_MIN -255 //@def Minimum servo PID output cmd //E! TODO
#define SERVO_PID_OUT_MAX 255 //@def Maximum servo PID output cmd //E! TODO
#define KP_SERVO 1 //@def Motor PID proportional gain //E! TODO
#define KI_SERVO 1 //@def Motor PID integral gain //E! TODO
#define KD_SERVO 1 //@def Motor PID differential gain //E! TODO

//E Control parameters
#define MOTOR_LOOP 10
#define STEER_LOOP 10
#define PUB_LOOP 10

/**
 * @class Acid
 * @brief Main control class. Ties everything together.
**/
class Acid {
public:
	Acid(PhysCommander& pcommander, JetsonCommander& jcommander, MotorInterface& motor, ServoInterface& servo, int motor_interval, int steer_interval, int pub_interval);
	void prep();
	void drop();
private:
	void speed();
	void steer();
	void publish();
	void setAutonomy(int mode);

	int _motor_interval, _steer_interval, _pub_interval;

	PhysCommander& _pcommander;
	JetsonCommander& _jcommander;

	Commander* _commander;

	MotorInterface& _motor;
	ServoInterface& _servo;

	int _mode;

	unsigned long _t_last_motor, _t_last_steer, _t_last_pub;
};

#endif