/*
commander.cpp
Electric Kool-Aide Commander

	@author Sarah Pohorecky <spohorec@mit.edu>

	@date 2017-07-08 creation.
	@
*/

#include "commander.h"

//This means that it will map input voltages between 0 and 5 volts into integer values between 0 and 1023. 
	//This yields a resolution between readings of: 5 volts / 1024 units or, .0049 volts (4.9 mV) per unit.

PhysCommander::PhysCommander(Throttle *motor_throttle, Throttle *field_throttle, int p_reverse) {
	_motor_throttle = motor_throttle;
	_field_throttle = field_throttle;

	_p_reverse = p_reverse;
}

int PhysCommander::getMotorCmd() {
	int cmd = _motor_throttle->getThrottle();
	int direction = getDirection();
	return cmd * direction;
}

unsigned char PhysCommander::getFieldCmd() {
	unsigned char cmd = _field_throttle->getThrottle();

	return cmd;
}

unsigned char PhysCommander::getSteeringCmd() {
	return 0;
}

int PhysCommander::getDirection() {
	if digitalRead(_p_reverse) {
		return -1;
	} else {
		return 1;
	}
}

bool PhysCommander::getEstop() {
	return false;
}

// JetsonCommander::JetsonCommander(ros::NodeHandle *nh) {}
// unsigned char JetsonCommander::getMotorCmd() {}
// unsigned char JetsonCommander::getFieldCmd() {}
// int JetsonCommander::getSteeringCmd() {}
// int JetsonCommander::getEstop() {}
