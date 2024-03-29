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
Acid::Acid(PhysCommander& pcommander, JetsonCommander& jcommander, MotorInterface& motor,
            ServoInterface &servo, int motor_interval, int steer_interval, int pub_interval,
            SpeedSensor *motor_rpm_sensor, SpeedSensor *fl_rpm_sensor, SpeedSensor *fr_rpm_sensor, ros::NodeHandle *nh,
            std_msgs::Int8 *steer_angle_msg, 
            std_msgs::UInt8 *throttle_msg,
            std_msgs::UInt16 *motor_rpm_msg,
            std_msgs::UInt16 *fl_rpm_msg,
            std_msgs::UInt16 *fr_rpm_msg,
            std_msgs::Bool *drive_dir_msg,
            std_msgs::Bool *brake_left_msg,
            std_msgs::Bool *brake_right_msg,
            ros::Publisher *steer_angle_pub,
            ros::Publisher *throttle_pub, 
            ros::Publisher *motor_rpm_pub,
            ros::Publisher *fl_rpm_pub, 
            ros::Publisher *fr_rpm_pub,
            ros::Publisher *drive_dir_pub, 
            ros::Publisher *brake_left_pub, 
            ros::Publisher *brake_right_pub): 
      _pcommander(pcommander),
		  _jcommander(jcommander),
		  _motor(motor),
		  _servo(servo){
  
	_motor_interval = motor_interval;
	_steer_interval = steer_interval;
	_pub_interval = pub_interval;

	_commander = &_pcommander; //E this pointer allows for better control loop (switch pointer with autonomy mode, everything else stays the same)
	_mode = 0;

	_t_last_motor = millis();
	_t_last_steer = _t_last_motor;
	_t_last_pub = _t_last_motor;

  _motor_rpm_sensor = motor_rpm_sensor;
  _fl_rpm_sensor = fl_rpm_sensor;
  _fr_rpm_sensor = fr_rpm_sensor;
  
  _nh = nh;
  _steer_angle_msg = steer_angle_msg;
  _throttle_msg = throttle_msg;
  _motor_rpm_msg = motor_rpm_msg;
  _fl_rpm_msg = fl_rpm_msg;
  _fr_rpm_msg = fr_rpm_msg;
  _drive_dir_msg = drive_dir_msg;
  _brake_left_msg = brake_left_msg;
  _brake_right_msg = brake_right_msg;
  
  _steer_angle_pub = steer_angle_pub;
  _throttle_pub = throttle_pub;
  _motor_rpm_pub = motor_rpm_pub;
  _fl_rpm_pub = fl_rpm_pub;
  _fr_rpm_pub = fr_rpm_pub;
  _drive_dir_pub = drive_dir_pub;
  _brake_left_pub = brake_left_pub;
  _brake_right_pub = brake_right_pub;

}

/**
 * @func Acid::prep
 * @brief any setup to do before running main loop
 **/
void Acid::prep() {
	//E be preppy
	//$ clear messages
  _steer_angle_msg->data = 0;
  _throttle_msg->data = 0;
  _motor_rpm_msg->data = 0;
  _fl_rpm_msg->data = 0;
  _fr_rpm_msg->data = 0;
  _brake_left_msg->data = 0;
  _brake_right_msg->data = 0;
}

/**
 * @func Acid::drop
 * @brief (because I am hilarious) main control loop
 **/
void Acid::drop() {

	_t_last_motor = _t_last_steer = _t_last_pub = millis();
 
	while (TRIPPING) {
      _nh->spinOnce(); //$ spin node handle
    
		unsigned long t = millis(); //E current time
//    if(t%1000<10) Serial.println("ping!");

		//E time periods since that actions taken
		unsigned long dt_motor = t - _t_last_motor;
		unsigned long dt_steer = t - _t_last_steer;
		unsigned long dt_pub = t - _t_last_pub;
    //Serial.println(_commander->getMotorCmd());

		//E handle autonomy changes
		setAutonomy(_commander->getMode());
	
		if (dt_motor > _motor_interval) { //E do some speed
			doSpeed();
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
void Acid::doSpeed() {
	int motor_cmd = _commander->getMotorCmd();
	int regen_cmd = _commander->getRegenCmd();
	_motor.handleCmds(motor_cmd, regen_cmd);
}

/**
 * @func PRIVATE Acid::steer
 * @brief gets and sends servo (steering) commands
 **/
void Acid::steer() {
	int steer_cmd = _commander->getSteeringCmd();
	_servo.handleCmd(steer_cmd);
}

void Acid::printAll(){
  Serial.print(_motor_rpm);
  Serial.print(" ");
  Serial.print(_fl_rpm);
  Serial.print(" ");
  Serial.print(_fr_rpm);
  Serial.print(" ");
  Serial.print(analogRead(P_SERVO_POT)); 
  Serial.print(" ");
  Serial.print(_steer_angle);
  Serial.print(" ");
  Serial.print(analogRead(P_MOTOR_THROTTLE));
  Serial.print(" ");
  Serial.print(_throttle);
  Serial.print(" ");
  Serial.print(_drive_dir);
  Serial.print(" ");
  Serial.print(_brake_left);
  Serial.print(" ");
  Serial.print(_brake_right);
  Serial.println(" ");
}

/**
 * @func PRIVATE Acid::publishmotor revs between check times
 * @brief gets messages and does ROS publishing
 **/
void Acid::publish() {
  //TODO: Update these only when they are first read, not when we publish. 
  _motor_rpm = _motor_rpm_sensor->getRPM();
  _fl_rpm = _fl_rpm_sensor->getRPM();
  _fr_rpm = _fr_rpm_sensor->getRPM();
  _steer_angle = (int8_t)((-254.0*(constrain(analogRead(P_SERVO_POT),SERVO_POT_MIN, SERVO_POT_MAX)-SERVO_POT_MIN))/((float)(SERVO_POT_MAX-SERVO_POT_MIN))+127.5); //Minimum is -128 Max is 127

  
  int temp_throttle = _commander->getMotorCmd();
  _throttle = abs(temp_throttle); //Apparently putting functons inside of abs() is bad. Created a temp var instead.
  _drive_dir = (temp_throttle>0);
  _brake_left = !digitalRead(P_BRAKE_2);
  _brake_right = !digitalRead(P_BRAKE_1);
//  printAll();
  
	//E publish things
  _steer_angle_msg->data = _steer_angle;
  _throttle_msg->data = _throttle;
  _motor_rpm_msg->data = _motor_rpm;
  _fl_rpm_msg->data = _fl_rpm;
  _fr_rpm_msg->data = _fr_rpm;
  _drive_dir_msg->data = _drive_dir;
  _brake_left_msg->data = _brake_left;
  _brake_right_msg->data = _brake_right;
  
  _steer_angle_pub->publish(_steer_angle_msg);
  _throttle_pub->publish(_throttle_msg);
  _motor_rpm_pub->publish(_motor_rpm_msg);
  _fl_rpm_pub->publish(_fl_rpm_msg);
  _fr_rpm_pub->publish(_fr_rpm_msg);
  _drive_dir_pub->publish(_drive_dir_msg);
  _brake_left_pub->publish(_brake_left_msg);
  _brake_right_pub->publish(_brake_right_msg);
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
//			_servo.usePID(false);

//			_servo.useServo(false); //E turn off servo

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
		int motor_cmd = abs(_commander->getMotorCmd());
		// Serial.print("\t\t\t\t");
		// Serial.println(motor_cmd);
		_motor.handleCmds(motor_cmd,0);

		// _motor.logLastInps();
	// if (Serial.available() > 0) {
	// 	_last_test_inp = (int) Serial.parseInt();
	// 	Serial.println("got inp");
	// }
	// _motor.handleCmds(0,_last_test_inp,0);
	// Serial.println(_last_test_inp);
	}
}
