/** 
	acid.cpp
	Electric Kool-Aid Turing Test Main Control File
	

	@date 2017-07-08 creation



**/

Acid::Acid():
	_kelly(P_MOTOR_PWM,P_REGEN_PWM,0),
	_field(),
	_commander(P_MOTOR_THROTTLE, P_FIELD_THROTTLE) 
{
	//E Do nothing for now!
}

Acid::prep() {
	_field.sendCmd(0); //E zero outputs
	_kelly.sendCmd(0);
}

Acid::drop() {

	while (TRIPPING) {

		unsigned char field_cmd = _commander.getFieldCmd();
		unsigned char motor_cmd = _commander.getMotorCmd();

		_field.sendCmd(field_cmd);
		_kelly.sendCmd(motor_cmd);

	}
}