/** 
	acid.h
	Electric Kool-Aid Turing Test Main Header
	

	@date 2017-07-08 creation


**/
#include <Arduino.h>

#include "commander.h"
#include "motorinterface.h"
// #include "throttle.h"

#define TRIPPING true

//E PWM Output pins
	//E On Arduino Uno: can only use 3, 5, 6, 9, 10, and 11
	//E On Arduino Mega: only 2-13 and 44-46
#define P_FIELD_PWM 11
#define P_MOTOR_PWM 3 
#define P_REGEN_PWM 5

//E Analog Input Pins (Throttles)
#define P_FIELD_THROTTLE A2
#define P_MOTOR_THROTTLE A3

//E Throttle Ranges
#define MOTOR_THROTTLE_MIN 0.85
#define MOTOR_THROTTLE_MAX 4.35
#define FIELD_THROTTLE_MIN 0.85
#define FIELD_THROTTLE_MAX 4.35

class Acid {
public:
	Acid();
	void prep();
	void drop();
private:
	Throttle _motor_throttle, _field_throttle;
	MotorInterface _kelly;
	FieldInterface _field;
	PhysCommander _commander;

};
