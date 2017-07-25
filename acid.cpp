/** 
 * acid.cpp
 * Electric Kool-Aid Acid Test Main Control File
 * 
 * @author Sarah Pohorecky <spohorec@mit.edu> 
 *
 * @date 2017-07-08 creation
 * @date 2017-07-14 some outlining
 * @date 2017-07-14 refactored with references. Bug fixing.
**/

#include "acid.h"


/**
 * @constr Acid::Acid
 * @brief sets loop intervals and initializes objects
 * @param [PhysCommander] <pcommander> gets human command input
 * @param [JetsonCommander] <jcommander> gets autonomous command input
 * @param [MotorInterface] <motor> interfaces with motor and its sensors & systems
 * @param [ServoInterface] <servo> interfaces with servo and its sensors & systems
 * @param [int] <motor_interval> speed-control loop interval
 * @param [int] <steer_interval> steer-control loop interval
 * @param [int] <pub_interval> ROS publishing loop interval
**/
 
Acid::Acid(ros::NodeHandle& nh, Messenger& messenger, PhysCommander& pcommander, JetsonCommander& jcommander, MotorInterface& motor, ServoInterface &servo, int motor_interval, int steer_interval, int pub_interval)
		: _nh(nh),
		  _messenger(messenger),
		  _pcommander(pcommander),
		  _jcommander(jcommander),
		  _motor(motor),
		  _servo(servo) {

	_nh.spinOnce();

	_motor_interval = motor_interval;
	_steer_interval = steer_interval;
	_pub_interval = pub_interval;

	_commander = &_pcommander; //E this pointer allows for better control loop (switch pointer with autonomy mode, everything else stays the same)
	_mode = 0;

	_t_last_motor = millis();
	_t_last_steer = _t_last_motor;
	_t_last_pub = _t_last_motor;

	_steer_cmd = 0;
	_motor_cmd = 0;
	_field_cmd = 0;
	_regen_cmd = 0;
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

	_t_last_motor = _t_last_steer = _t_last_pub = millis();

	while (TRIPPING) {
		_nh.spinOnce();

		unsigned long t = millis(); //E current time

		//E time periods since that actions taken
		unsigned long dt_motor = t - _t_last_motor;
		unsigned long dt_steer = t - _t_last_steer;
		unsigned long dt_pub = t - _t_last_pub;

		//E handle autonomy changes
		// setAutonomy(_commander->getMode()); //E Disabled for Detroit TODO have use_autonomy be a parameter?
	
		if (dt_motor > _motor_interval) { //E do some speed
			speed();
			_t_last_motor = t;
		}
		
		if (dt_steer > _steer_interval) { //E do some steer	
			steer();
			_t_last_steer = t;
		}

		if (dt_pub > _pub_interval) { //E do the publish thing
			publish();
			_t_last_pub = t;
		}

	} //E Come down

}

/**
 * @func PRIVATE Acid::speed
 * @brief gets and sends motor (speed) commands
 **/
void Acid::speed() {
	_field_cmd = _commander->getFieldCmd();
	_motor_cmd = _commander->getMotorCmd();
	_regen_cmd = _commander->getRegenCmd();
	// lg3(motor_cmd,field_cmd,regen_cmd);
	_motor.handleCmds(_motor_cmd, _field_cmd, _regen_cmd);
}

/**
 * @func PRIVATE Acid::steer
 * @brief gets and sends servo (steering) commands
 **/
void Acid::steer() {
	_steer_cmd = _commander->getSteeringCmd();
	_servo.handleCmd(_steer_cmd);
}

/**
 * @func PRIVATE Acid::publish
 * @brief gets messages and does ROS publishing
 **/
void Acid::publish() {

	int last_motor_inp = _motor.getLastMotorInput();
	unsigned char last_field_inp =_motor.getLastFieldInput();
	unsigned char last_regen_inp =_motor.getLastRegenInput();
	long last_rpm =_motor.getLastRPM();

	_messenger.sendMotorsMsg(_motor_cmd,_field_cmd,_regen_cmd,last_motor_inp,last_field_inp,last_regen_inp,last_rpm);
	_messenger.sendSteeringMsg(0,0);
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

/**
 * @func Acid::test
 * @brief random test code
 **/
void Acid::test() {
	while (TRIPPING) {
		int field_cmd = _commander->getFieldCmd();
		int motor_cmd = abs(_commander->getMotorCmd());
		// Serial.print("\t\t\t\t");
		// Serial.println(motor_cmd);
		_motor.handleCmds(motor_cmd,field_cmd,0);

		// _motor.logLastInps();
	// if (Serial.available() > 0) {
	// 	_last_test_inp = (int) Serial.parseInt();
	// 	Serial.println("got inp");
	// }
	// _motor.handleCmds(0,_last_test_inp,0);
	// Serial.println(_last_test_inp);
	}
}

//-----------------------------------------------------------------------------------------------------------




Messenger::Messenger(ros::NodeHandle& nh, const char* motors_topic, const char* steering_topic):
	_nh(nh),
	_motors_msg(new koolaid::Motors),
	_steering_msg(new koolaid::Steering),
	_motors_pub("motors",_motors_msg),
	_steer_pub("steering",_steering_msg)
{
	_nh.advertise(_motors_pub);
	_nh.advertise(_steer_pub);
}
void Messenger::sendMotorsMsg(int motor_command, unsigned char field_command, unsigned char regen_command, 
		int motor_input, unsigned char field_input, unsigned char regen_input, long rpm) {

	_motors_msg->motor_command = motor_command;
	_motors_msg->field_command = field_command;
	_motors_msg->regen_command = regen_command;

	_motors_msg->motor_input = motor_input;
	_motors_msg->field_input = field_input;
	_motors_msg->regen_input = regen_input;

	_motors_msg->rpm = rpm;

	_motors_pub.publish(_motors_msg);
}

void Messenger::sendSteeringMsg(unsigned char angle_command, unsigned char angle) {
	
	_steering_msg->angle_command = angle_command;
	_steering_msg->angle = angle;

	_steer_pub.publish(_steering_msg);
}
