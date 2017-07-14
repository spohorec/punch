/** 
 * acid.cpp
 * Electric Kool-Aid Acid Test Main Control File
 * 
 * @author Sarah Pohorecky <spohorec@mit.edu> 
 *
 * @date 2017-07-08 creation
 * @date 2017-07-14 some outlining
**/

#include "acid.h"

/**
 * @constr Acid::Acid
 * @brief sets loop intervals and initializes objects
 * @param [int] <motor_interval> speed-control loop interval
 * @param [int] <steer_interval> steer-control loop interval
 * @param [int] <pub_interval> ROS publishing loop interval
**/
Acid::Acid(int motor_interval, int steer_interval, int pub_interval)
		: _pcommander(P_REVERSE_SWITCH, P_BRAKE_1, P_BRAKE_2),
		  _motor(P_MOTOR_PWM, P_REGEN_PWM, P_SET_REVERSE, MIN_REGEN_FIELD),
		  _servo(P_SERVO_PWM_A, P_SERVO_PWM_B) {

	_motor_interval = motor_interval;
	_steer_interval = steer_interval;
	_pub_interval = pub_interval;

	_commander = &_pcommander; //E this pointer allows for better control loop (switch pointer with autonomy mode, everything else stays the same)
	_mode = 0;
}

/**
 * @func Acid::prep
 * @brief any setup to do before running main loop
 **/
void Acid::prep() {
	//E be preppy
	//E! TODO (may be unneccesary?)
}

/**
 * @func Acid::drop
 * @brief (because I am hilarious) main control loop
 **/
void Acid::drop() {
	int field_cmd, motor_cmd, regen_cmd, steer_cmd;
	unsigned long t, t_last_motor, t_last_steer, t_last_pub;
	unsigned long dt_motor, dt_steer, dt_pub;

	while (TRIPPING) {
		t = millis(); //E current time

		//E time periods since that actions taken
		dt_motor = t - t_last_motor;
		dt_steer = t - t_last_steer;
		dt_pub = t - t_last_pub;

		//E handle autonomy changes
		setAutonomy(_commander->getMode());
	
		if (dt_motor > _motor_interval) { //E do some speed
			speed();
			t_last_motor = t;
		}

		if (dt_steer > _steer_interval) { //E do some steer	
			steer();
			t_last_steer = t;
		}

		if (dt_pub > _pub_interval) { //E do the publish thing
			publish();
			t_last_pub = t;
		}

	} //E Come down

}

/**
 * @func PRIVATE Acid::speed
 * @brief gets and sends motor (speed) commands
 **/
void Acid::speed() {
	field_cmd = _commander->getFieldCmd();
	motor_cmd = _commander->getMotorCmd();
	regen_cmd = _commander->getRegenCmd();
	_motor.handleCmds(motor_cmd, field_cmd, regen_cmd);
}

/**
 * @func PRIVATE Acid::steer
 * @brief gets and sends servo (steering) commands
 **/
void Acid::steer() {
	steer_cmd = _commander->getSteeringCmd();
	_servo.handleCmd(steer_cmd);
}

/**
 * @func PRIVATE Acid::publish
 * @brief gets messages and does ROS publishing
 **/
void Acid::publish() {
	//E publish things
	//E! TODO
}

/**
 * @func PRIVATE Acid::setAutonomy
 * @brief gets and sends motor (speed) commands
 * @param [int] <mode> human-driven mode (0) or autonomous (1)
 **/
void Acid::setAutonomy(int mode) {
	if (mode != _mode) {
		if (mode == 0) { //E physical mode

			_commander = &_pcommander;

			_motor.usePID(false); //E don't use PID in human mode
			_servo.usePID(false);

			_servo.useServo(false); //E turn off servo

			_mode = mode;

		} else if (mode == 1) { //E autonomous mode
			//E do some things at some point!
		}
	}
}