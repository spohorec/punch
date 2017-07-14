/** 
	acid.h
	Electric Kool-Aid Acid Test Main Header
	

	@date 2017-07-08 creation
	@date 2017-07-13 updates with new pinouts


**/

#include <Arduino.h>

#include "commander.h"
#include "motorinterface.h"

#define TRIPPING true //@def I am funny hi

#define P_AUTONOMY_SWITCH 1 //@def Physical mode switch input pin

#define P_REGEN_PWM 10 //@def Regen output pin
#define P_BRAKE_1 7 //@def Physical brake switch 1 input pin
#define P_BRAKE_2 8 //@def Physical brake switch 2 input pin

#define P_REVERSE_SWITCH 0 //@def Physical reverse switch input pin
#define P_SET_REVERSE 12 //@def Kelly reverse switch command output pin
	
#define P_MOTOR_PWM 9 ///@def Kelly motor command output pin
#define P_MOTOR_THROTTLE A0 //@def Motor throttle input pin
#define MOTOR_THROTTLE_MIN 0.85 //@def Motor throttle min voltage 
#define MOTOR_THROTTLE_MAX 4.35 //@def Motor throttle max voltage

#define P_ENCODER 2 //@def Encoder input pin
#define ENCODER_INTERRUPT 0 //@def Interrupt attached to motor encoder

#define P_FIELD_PWM 11 //@def Field driver command output pin
#define P_FIELD_THROTTLE A1 //@def Field Throttle input pin
#define FIELD_THROTTLE_MIN 0.85 //@def Field throttle min voltage
#define FIELD_THROTTLE_MAX 4.35 //@def Field throttle max voltage

#define P_THERMISTOR A3 //@def Thermistor input pin
#define P_TEMP_INDICATOR 4 //@def Overheating indicator output pin

#define P_SET_SERVO 13 //@def MegaMoto enable command output pin
#define P_SERVO_PWM_A 6 ///@def Servo PWM A pin
#define P_SERVO_PWM_B 5 //@def Servo PWM B pin
#define P_SERVO_POT A2 //@def Servo pot input pin
	

/**
 * @class Acid
 * @brief Main control class. Ties everything together.
**/
class Acid {
public:
	Acid(int motor_interval, int steer_interval, int pub_interval);
	void prep();
	void drop();
private:
	void speed();
	void steer();
	void setAutonomy();

	int _motor_interval, _steer_interval, _pub_interval;

	PhysCommander _pcommander;
	Commander *_commander;

	MotorInterface _motor;
	ServoInterface _servo;

	int _mode;
};
