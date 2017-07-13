/*
motorinterface.cpp
Electric Kool-Aide Motor Controller-Controller

	@author Sarah Pohorecky <spohorec@mit.edu>

	@date 2017-07-08 creation.
	@date 2017-07-13 update with reverse handling.

*/

#include "motorinterface.h"

MotorInterface::MotorInterface(int p_motor_pwm, int p_regen_pwm, int p_reverse_switch ) {
	_p_motor_pwm = p_motor_pwm;
	_p_regen_pwm = p_regen_pwm;

	_p_reverse_switch = p_reverse_switch;

	_reverse_on = false;
	_regen_on = false;
}

void MotorInterface::sendCmd(unsigned char cmd) {
	if (cmd < 0) {
		setReverseOn(true);
	} else {
		setReverseOn(false);
	}

	int duty = abs(cmd);
	if (duty > 255) duty = 255;
	if (duty < 0) duty = 0;

	analogWrite(_p_motor_pwm,duty);
}

void MotorInterface::sendRegenCmd(unsigned char cmd) {
	int duty = cmd;
	if (duty > 255) duty = 255;
	if (duty < 0) duty = 0;
	analogWrite(_p_regen_pwm,duty);
}


void MotorInterface::setReverseOn(bool reverse_on) {
	if 	(_reverse_on != reverse_on) { //E don't bother changing pin modes if we are still going in the same direction
		if (reverse_on) {
			pinMode(_p_reverse_switch,OUTPUT);
			digitalWrite(_p_reverse_switch,LOW); //E Kelly switches are ON when GROUNDED
		} else {
			pinMode(_p_reverse_switch,INPUT); //E this makes pin floating
		}
		_reverse_on = reverse_on;
	} 
}

// ----------------------------------------------------------------------------------

FieldInterface::FieldInterface(float min_field_v, float max_field_v) {
	_min_field_v = min_field_v;
	_max_field_v = max_field_v;

}

void FieldInterface::sendCmd(unsigned char cmd) {
	double v_desired = ((float) cmd / 255) * (_max_field_v - _min_field_v) + _min_field_v;
	double duty = v_desired / V_BATTERY_MAX;
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
