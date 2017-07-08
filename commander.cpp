/*
commander.cpp
Electric Kool-Aide Commander

	@author Sarah Pohorecky <spohorec@mit.edu>

	@date 2017-07-08 creation.

*/

//This means that it will map input voltages between 0 and 5 volts into integer values between 0 and 1023. 
	//This yields a resolution between readings of: 5 volts / 1024 units or, .0049 volts (4.9 mV) per unit.

PhysCommander::PhysCommander(Throttle* main_th, Throttle* field_th, int reverse_pin) {
	_main_th = main_th;
	_field_th = field_th;
	_reverse_pin = reverse_pin;
}

int PhysCommander::getMotorCmd() {
	int cmd = _main_th->getCmd();
	int reverse = digitalRead(_reverse_pin);

	if (reverse) {
		return cmd * -1;
	} else {
		return cmd;
	}
}
unsigned char PhysCommander::getFieldCmd() {
	return _field_th->getCmd()
}

int PhysCommander::getSteeringCmd() {
	return 0;
}

int PhysCommander::getKillCmd() {
	return 0;
}

bool PhysCommander::getEstop() {
	return false;
}

RCCommander::RCCommander(RCDecoder *motor_d, RCDecoder *field_d, RCDecoder *steer_d, RCDecoder *kill_d)
unsigned char RCCommander::getMotorCmd()
unsigned char RCCommander::getFieldCmd()
int RCCommander::getSteeringCmd()
int RCCommander::getKillCmd()
int RCCommander::getEstop()

JetsonCommander::JetsonCommander(ros::NodeHandle *nh)
unsigned char JetsonCommander::getMotorCmd()
unsigned char JetsonCommander::getFieldCmd()
int JetsonCommander::getSteeringCmd()
int JetsonCommander::getKillCmd()
int JetsonCommander::getEstop()
