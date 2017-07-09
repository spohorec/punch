/*
motorinterface.h
Electric Kool-Aide Motor Controller-Controllers

	@author Sarah Pohorecky <spohorec@mit.edu>

	@date 2017-07-08 creation.

*/

/*
KELLY
	-2 pwm pins
		-motor speed
		-regen
	-3 switches
		-reverse
		-regen
		-forward
FIELD
	-1 pwm pin
		-field driver switch signal
SERVO
	-2 pwm (probably)
		-rotate right
		-rotate left

*/

#include <Arduino.h>
		
#define V_BATTERY_MAX 33.6 //E! TODO move elsewhere and confirm value	

class MotorInterface {
public:
	MotorInterface(int p_motor_pwm, int p_regen_pwm, int p_regen_switch);
	void sendCmd(unsigned char cmd);
	void sendRegenCmd(unsigned char cmd);


private:
	int _p_motor_pwm, _p_regen_pwm;
	// int _p_forward_switch, _p_reverse_switch, _p_regen_switch;
	int  _p_regen_switch;
	bool _regen_on; //_p_reverse_on,

};

// ----------------------------------------------------------------------------------


// class ServoInterface {
// public:
// 	ServoInterface();
// 	void sendCmd();

// private:
// 	int _p_servo_r, _p_servo_l;

// };

// ----------------------------------------------------------------------------------


class FieldInterface {
public:
	FieldInterface(float min_field_v=0.0, float max_field_v=16.0); //E! what should min actually be?
	void sendCmd(unsigned char cmd);
	void getMaxVoltage();

private:
	float _min_field_v, _max_field_v;

};