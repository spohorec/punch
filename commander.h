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
	virtual int getMotorCmd();
	virtual unsigned char getFieldCmd();
	virtual unsigned char getSteeringCmd();
	virtual bool getEstop();

};


class PhysCommander: public Commander {
public:
	PhysCommander(Throttle *motor_throttle, Throttle *field_throttle, int p_reverse);
	int getMotorCmd();
	unsigned char getFieldCmd();
	unsigned char getSteeringCmd();
	int getDirection();
	bool getEstop();
private:
	Throttle *_motor_throttle, *_field_throttle;
	int _p_reverse;
};

// class JetsonCommander: public Commander {
// public:
// 	JetsonCommander(ros::NodeHandle *nh);
// 	unsigned char getMotorCmd();
// 	unsigned char getFieldCmd();
// 	unsigned char getSteeringCmd();
// 	bool getEstop();

// private:

// };