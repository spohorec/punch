/*
commander.h
Electric Kool-Aide Commander

	@author Sarah Pohorecky <spohorec@mit.edu>

	@date 2017-07-08 creation.

*/

#include <Arduino.h>
#include "throttle.h"

class Commander {
public:
	virtual unsigned char getMotorCmd();
	virtual unsigned char getFieldCmd();
	virtual unsigned char getSteeringCmd();
	virtual bool getEstop();

};


class PhysCommander: public Commander {
public:
	PhysCommander(Throttle *motor_throttle, Throttle *field_throttle); //(Throttle* main_th, Throttle* field_th) //, int reverse_pin);
	unsigned char getMotorCmd();
	unsigned char getFieldCmd();
	unsigned char getSteeringCmd();
	bool getEstop();
private:
	// Throttle* _main_th, _field_th;
	// int _reverse_pin;
	Throttle *_motor_throttle, *_field_throttle;

};

// class RCCommander: public Commander {
// public:
// 	RCCommander(RCDecoder *motor_d, RCDecoder *field_d, RCDecoder *steer_d, RCDecoder *kill_d);
// 	unsigned char getMotorCmd();
// 	unsigned char getFieldCmd();
// 	unsigned char getSteeringCmd();
// 	bool getEstop();
// private:

// };

// class JetsonCommander: public Commander {
// public:
// 	JetsonCommander(ros::NodeHandle *nh);
// 	unsigned char getMotorCmd();
// 	unsigned char getFieldCmd();
// 	unsigned char getSteeringCmd();
// 	bool getEstop();

// private:

// };