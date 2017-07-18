/**
 * commander.h
 * Electric Kool-Aide Commander
 * 
 * @author Sarah Pohorecky <spohorec@mit.edu>
 * 
 * @date 2017-07-08 	creation.
 * @date 2017-07-13 	update with reverse handling.
 * @date 2017-07-14 	outlined jetson commander and some additional methods
 * @date 2017-07-14 refactored with references. Bug fixing. Added header guards.
**/

#ifndef __COMMANDER_H
#define __COMMANDER_H

#include <Arduino.h>
#include "sensors.h"

// ----------------------------------------------------------------------------------

/**
 * @class Commander
 * @brief super-class of commander types. Gets inputs from human or computer
**/
class Commander {
public:
	virtual int getMotorCmd();
	virtual int getFieldCmd();
	virtual int getSteeringCmd();
	virtual int getRegenCmd();
	virtual bool getEstop();
	virtual int getMode();

};

// ----------------------------------------------------------------------------------

/**
 * @class PhysCommander
 * @brief commander that takes in human input
**/
class PhysCommander: public Commander {
public:
	PhysCommander(int p_reverse, int p_mode, int p_brake_1, int p_brake_2, Throttle& motor_throttle, Throttle& field_throttle);
	int getMotorCmd();
	int getFieldCmd();
	int getSteeringCmd();
	int getRegenCmd();
	bool getEstop();
	int getMode();
private:
	int getDirection();

	Throttle& _motor_throttle, _field_throttle;
	
	int _p_reverse, _p_mode, _p_brake_1, _p_brake_2;
};

// ----------------------------------------------------------------------------------

/**
 * @class JetsonCommander
 * @brief commander that takes in computer input
**/
class JetsonCommander: public Commander {
public:
	JetsonCommander();
	int getMotorCmd();
	int getFieldCmd();
	int getSteeringCmd();
	int getRegenCmd();
	bool getEstop();
	int getMode();

private:

};

#endif