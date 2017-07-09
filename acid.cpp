/** 
	acid.cpp
	Electric Kool-Aid Turing Test Main Control File
	

	@date 2017-07-08 creation



**/

#include "acid.h"

Acid::Acid():
	_motor_throttle(P_MOTOR_THROTTLE,MOTOR_THROTTLE_MIN,MOTOR_THROTTLE_MAX),
	_field_throttle(P_FIELD_THROTTLE,FIELD_THROTTLE_MIN,FIELD_THROTTLE_MAX),
	_kelly(P_MOTOR_PWM,P_REGEN_PWM,0),
	_field(),
	_commander(&_motor_throttle, &_field_throttle)
{
	//E Do nothing for now!
}

void Acid::prep() {
	_field.sendCmd(0); //E zero outputs
	_kelly.sendCmd(0);
}

void Acid::drop() {
	int counter=0;
	while (TRIPPING) {

		if (counter >= 100) {
			// counter = 0;
			unsigned char motor_cmd = _commander.getMotorCmd();
			unsigned char field_cmd = _commander.getFieldCmd();

			_kelly.sendCmd(motor_cmd);
			_field.sendCmd(field_cmd);

		} else {
			counter++;
		}
	}
}
