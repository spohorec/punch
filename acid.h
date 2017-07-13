/** 
	acid.h
	Electric Kool-Aid Turing Test Main Header
	

	@date 2017-07-08 creation
	@date 2017-07-13 updates with new pinouts


**/

#include <Arduino.h>

#include "commander.h"
#include "motorinterface.h"

#define TRIPPING true

#define P_AUTONOMY_SWITCH 1 //E Physical mode switch input pin

#define P_REGEN_PWM 10 //E Regen output pin
#define P_BRAKE_1 7 //E Physical brake switch 1 input pin
#define P_BRAKE_2 8 //E Physical brake switch 2 input pin

#define P_REVERSE_SWITCH 0 //E Physical reverse switch input pin
#define P_SET_REVERSE 12 //E Kelly reverse switch command output pin
	
#define P_MOTOR_PWM 9 //E Kelly motor command output pin
#define P_MOTOR_THROTTLE A0 //E Motor throttle input pin
#define MOTOR_THROTTLE_MIN 0.85 //E Throttle Ranges
#define MOTOR_THROTTLE_MAX 4.35

#define P_ENCODER 2 //E Encoder input pin

#define P_FIELD_PWM 11 //E Field driver command output pin
#define P_FIELD_THROTTLE A1 //E Field Throttle input pin
#define FIELD_THROTTLE_MIN 0.85 //E Throttle Ranges
#define FIELD_THROTTLE_MAX 4.35

#define P_THERMISTOR A3 //E Thermistor input pin
#define P_TEMP_INDICATOR 4 //E Overheating indicator output pin

#define P_SET_SERVO 13 //E MegaMoto enable command output pin
#define P_SERVO_PWM_A 6 //E Servo PWM A pin
#define P_SERVO_PWM_B 5 //E Servo PWM B pin
#define P_SERVO_POT A2 //E Servo pot input pin
	

class Acid {
public:
	Acid();
	void prep();
	void drop();
private:
	Throttle _motor_throttle, _field_throttle;
	MotorInterface _kelly;
	FieldInterface _field;
	PhysCommander _pcommander;
	Commander *_commander;

};
