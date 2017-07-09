/*
motorinterface.cpp
Electric Kool-Aide Motor Controller-Controller

	@author Sarah Pohorecky <spohorec@mit.edu>

	@date 2017-07-08 creation.

*/

#define V_BATTERY_MAX 33.6 //E! TODO move elsewhere and confirm value	

MotorInterface::MotorInterface(int p_motor_pwm, int p_regen_pwm, int p_regen_switch
			/*int p_forward_switch, int p_reverse_switch,*/ ) {
	_p_motor_pin = p_motors_pin;
	_p_regen_pin = p_regen_pin;

	_p_forward_switch = p_forward_switch;
	_p_reverse_switch = p_reverse_switch;
	_p_regen_switch = p_regen_switch;

	// _reverse_on = false;
	_regen_on = false;
}

MotorInterface::SendCmd(int cmd) {
	int duty = floor(cmd / 255.0);
	if (duty > 255) duty = 255;
	if (duty < 0) duty = 0;
	analogWrite(_p_motor_pin,duty);
}

MotorInterface::SendRegenCmd(int cmd) {
	int duty = floor(cmd / 255.0);
	if (duty > 255) duty = 255;
	if (duty < 0) duty = 0;
	analogWrite(_p_regen_pin,duty);
}

// ----------------------------------------------------------------------------------

FieldInterface::FieldInterface(float min_field_v, float max_field_v) {
	_min_field_v = min_field_v;
	_max_field_v = max_field_v;

}

void FieldInterface::sendCmd(unsigned char cmd) {
	double v_desired = (cmd / 1023.0) * (_max_field_v - _min_field_v) + _min_field_v;
	double duty = v_desired / V_BATTERY_MAX; //E!? TODO would battery sensing be a good idea?
	int reg_value = floor(duty*256) - 1; //E duty cycle of PWM is given by (reg_value + 1)/256

	if (reg_value > 255) reg_value = 255;
	if (reg_value < 0) reg_value = 0; //E? Should this be -1 since val+1 gives duty?

	//E write reg value
	OCR2A = reg_value; //E! This needs to be changed if arduino platform is switched

}

void FieldInterface::getMaxVoltage() {
	//E do nothing right now
}

// ----------------------------------------------------------------------------------