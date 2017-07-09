/*
commander.cpp
Electric Kool-Aide Commander

	@author Sarah Pohorecky <spohorec@mit.edu>

	@date 2017-07-08 creation.

*/

//This means that it will map input voltages between 0 and 5 volts into integer values between 0 and 1023. 
	//This yields a resolution between readings of: 5 volts / 1024 units or, .0049 volts (4.9 mV) per unit.

PhysCommander::PhysCommander(int p_motor_thr, int p_field_thr) {//(Throttle* main_th, Throttle* field_th) {//, int reverse_pin) {
	_p_motor_thr = p_motor_thr;
	_p_field_thr = p_field_thr;

	pinMode(_p_motor_thr,INPUT);
	pinMode(_p_field_thr,INPUT);
	// _reverse_pin = reverse_pin;
}

unsigned char PhysCommander::getMotorCmd() {
	return analogRead(_p_motor_thr);
	// int reverse = digitalRead(_reverse_pin);

	// if (reverse) cmd *= 1;
}

unsigned char PhysCommander::getFieldCmd() {
	return analogRead(_p_field_thr);
}

unsigned char PhysCommander::getSteeringCmd() {
	return 0;
}

bool PhysCommander::getEstop() {
	return false;
}

// RCCommander::RCCommander(RCDecoder *motor_d, RCDecoder *field_d, RCDecoder *steer_d, RCDecoder *kill_d)
// unsigned char RCCommander::getMotorCmd() {}
// unsigned char RCCommander::getFieldCmd() {}
// int RCCommander::getSteeringCmd() {}
// int RCCommander::getEstop() {}

// JetsonCommander::JetsonCommander(ros::NodeHandle *nh) {}
// unsigned char JetsonCommander::getMotorCmd() {}
// unsigned char JetsonCommander::getFieldCmd() {}
// int JetsonCommander::getSteeringCmd() {}
// int JetsonCommander::getEstop() {}
