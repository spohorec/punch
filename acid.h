/** 
	acid.h
	Electric Kool-Aid Turing Test Main Header
	

	@date 2017-07-08 creation


**/

#include "commander.h"
#include "motorinterface.h"

#define TRIPPING true

#define P_FIELD_PWM 11
#define P_MOTOR_PWM 12 
#define P_REGEN_PWM 13

#define P_FIELD_THROTTLE 2
#define P_MOTOR_THROTTLE 3

class Acid {
public:
	Acid();
	void prep();
	void drop();
private:
	MotorInterface _kelly;
	FieldInterface _field;
	PhysCommander _commander;

};