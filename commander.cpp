/**
 * commander.cpp
 * Electric Kool-Aide Commander
 * 
 * @author Sarah Pohorecky <spohorec@mit.edu>
 * 
 * @date 2017-07-08 	creation.
 * @date 2017-07-13 	update with reverse handling.
 * @date 2017-07-14 	outlined jetson commander and some additional methods
 * @date 2017-07-14 	refactored with references. Bug fixing.
 * @date 2017-07-17		Implemented getRegenCmd, fixed pinmodes.
**/

#include "commander.h"


/**
 * @constr PhysCommander::PhysCommander
 * @brief sets up throttles and control switch pins
 * @param [int] <p_reverse> reverse switch input pin
 * @param [int] <p_brake_1> brake 1 switch input pin
 * @param [int] <p_brake_2> brake 2 switch input pin
**/
PhysCommander::PhysCommander(int p_reverse, int p_mode, int p_brake_1, int p_brake_2, Throttle& motor_throttle, Throttle& field_throttle) 
		: _motor_throttle(motor_throttle),
		  _field_throttle(field_throttle) {

	_p_reverse = p_reverse;
	_p_mode = p_mode;
	_p_brake_1 = p_brake_1;
	_p_brake_2 = p_brake_2;

	//E set pinModes
	pinMode(_p_reverse,INPUT_PULLUP); //E switches connected to ground, so require pullup resistors
	pinMode(_p_mode,INPUT_PULLUP);
	pinMode(_p_brake_1,INPUT_PULLUP);
	pinMode(_p_brake_2,INPUT_PULLUP);

}

/**
 * @func PhysCommander::getMotorCmd
 * @brief gets motor command from throttle and sets its direction
 * @returns [int] signed motor command (-255-255)
 **/
int PhysCommander::getMotorCmd() {
	int cmd = _motor_throttle.getThrottle();
	int direction = getDirection();
	return cmd * direction;
}

/**
 * @func PhysCommander::getFieldCmd
 * @brief gets field command from throttle
 * @returns [int] field command (0-255)
 **/
int PhysCommander::getFieldCmd() {
	int cmd = _field_throttle.getThrottle();

	return cmd;
}

/**
 * @func PhysCommander::getSteeringCmd
 * @brief no steering commands from PhysCommander, included for compatability with Commander
 * @returns [int] always returns 0
 **/
int PhysCommander::getSteeringCmd() {
	return 0; //E! TODO return 0 or centered command?
}

/**
 * @func PhysCommander::getRegenCmd
 * @brief reads brake switches and gives command
 * @returns [int] regen command (0-255)
 **/
int PhysCommander::getRegenCmd(){
	//E TODO
	int cmd;
	if (! digitalRead(_p_brake_1) && ! digitalRead(_p_brake_2)) { //E both switches pressed
		cmd = 255;
	}
	else if (! digitalRead(_p_brake_1) || ! digitalRead(_p_brake_2)) { //E one switch pressed
		cmd = 127;
	} else {
		cmd = 0;
	}
	return cmd;
}

/**
 * @func PRIVATE PhysCommander::getDirection
 * @brief reads reverse pin to get direction of command
 * @returns [int] +1 or -1 whether to go forwards or backwards
 **/
int PhysCommander::getDirection() {
	if (digitalRead(_p_reverse)) { //E sign reversed since pin uses pulldown resistor
		return 1;
	} else {
		return -1;
	}
}

/**
 * @func PhysCommander::getEstop
 * @brief no estop commands from PhysCommander, included for compatability with Commander
 * @returns [bool] always returns false
 **/
bool PhysCommander::getEstop() {
	return false;
}

/**
 * @func PhysCommander::getMode
 * @brief gets whether autonomous enable switch is set or not. Will not do anything unless also enabled from JetsonCommander.
 * @returns [int] mode 0 if human driven, 1 if autonomous
 **/
int PhysCommander::getMode(){
	if (digitalRead(_p_mode)) {
		return 0;
	} else {
		return 1; //E switch closed if groudned (pulldown)
	}
}

// ----------------------------------------------------------------------------------

/**
 * @constr JetsonCommander::JetsonCommander
 * @brief init. JetsonCommander (TODO)
**/
JetsonCommander::JetsonCommander() {
	//E! TODO
}

/**
 * @func JetsonCommander::getMotorCmd
 * @brief gets signed motor command from jetson
 * @returns [int] signed motor command (-255-255)
 **/
int JetsonCommander::getMotorCmd() {
	//E! TODO
	return 0;
}

/**
 * @func JetsonCommander::getFieldCmd
 * @brief gets signed motor command from jetson
 * @returns [int] field command (0-255)
 **/
int JetsonCommander::getFieldCmd() {
	//E! TODO
	return 0;
}

/**
 * @func JetsonCommander::getSteeringCmd
 * @brief gets steering command from jetson
 * @returns [int] signed steering command (-255-255)
 **/
int JetsonCommander::getSteeringCmd() {
	//E! TODO
	return 0;
}

/**
 * @func JetsonCommander::getRegenCmd
 * @brief gets regen command from jetson
 * @returns [int] regen command (0-255)
 **/
int JetsonCommander::getRegenCmd() {
	//E! TODO
	return 0;
}

/**
 * @func JetsonCommander::getEstop
 * @brief gets estop command from jetson
 * @returns [bool] true if estopped
 **/
bool JetsonCommander::getEstop() {
	//E! TODO
	return false;
}

/**
 * @func JetsonCommander::getMode
 * @brief gets mode command from jetson
 * @returns [int] mode 0 if human driven, 1 if autonomous
 **/
int JetsonCommander::getMode() {
	//E! TODO
	return 0;
}
