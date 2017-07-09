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

class MotorInterface {
public:
	MotorInterface();
	void sendCmd();
	void sendRegenCmd();


private:
	int _p_motor_pin, _p_regen_pin;
	int _p_forward_switch, _p_reverse_switch, _p_regen_switch;
	bool _p_reverse_on, _p_regen_on;

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
	void sendCmd();
	void getMaxVoltage();

private:
	float _min_field_v, _max_field_v;

};