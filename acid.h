/** 
 * acid.h
 * Electric Kool-Aid Acid Test Main Header
 * 
 * @author Sarah Pohorecky <spohorec@mit.edu> 
 *
 * @date 2017-07-08 creation
 * @date 2017-07-13 updates with new pinouts
 * @date 2017-07-14 some outlining
 * @date 2017-07-14 refactored with references. Bug fixing. Added header guards.
**/

#ifndef __ACID_H
#define __ACID_H

#include <Arduino.h>
#include "commander.h"
#include "motorinterface.h"
#include "sensors.h"
#include "pidcontroller.h"

#include <ros.h>
#include <geometry_msgs/Vector3.h>  //$ for gain adjustment
#include <std_msgs/Int16.h>         //$ for mode publishing
#include <std_msgs/UInt16.h>         //$ for mode publishing
#include <std_msgs/Int8.h>         //$ for mode publishing
#include <std_msgs/UInt8.h>         //$ for mode publishing
#include <std_msgs/Bool.h>          //$ for estop subscriber

#define TRIPPING true //@def I am funny hi

#define P_AUTONOMY_SWITCH 6 //@def Physical mode switch input pin

#define P_REGEN_PWM 10 //@def Regen output pin
#define P_BRAKE_1 7 //@def Physical brake switch 1 input pin
#define P_BRAKE_2 8 //@def Physical brake switch 2 input pin

#define P_REVERSE_SWITCH 5 //@def Physical reverse switch input pin
#define P_SET_REVERSE 12 //@def Kelly reverse switch command output pin

//E Motor parameters	
#define P_MOTOR_PWM 9 ///@def Kelly motor command output pin
#define P_MOTOR_THROTTLE A0 //@def Motor throttle input pin
#define MOTOR_THROTTLE_MIN_V 1.23 //E Updated after adding pulldown resistor //0.85 //@def Motor throttle min voltage
#define MOTOR_THROTTLE_MAX_V 4.35 //E Updated after adding pulldown resistor //4.35 //@def Motor throttle max voltage

//E Encoder parameters
#define P_ENCODER 2 //@def Rear Encoder input pin
#define ENCODER_INTERRUPT 0 //@def Interrupt attached to motor encoder
#define PULSES_PER_REV 20.0 //@def Number of encoder pulses per full motor revolution //E! TODO Confirm this value for Kool-Aid
#define PULSES_PER_REV_FRONT 6.0 //@def Number of encoder pulses per full motor revolution //E! TODO Confirm this value for Kool-Aid

#define P_ENCODER_FL 13//@def Front LeftEncoder input pin
#define P_ENCODER_FR 4//@def Front Right Encoder input pin
#define ENCODER_INTERRUPT_FL 1 //@def Interrupt attached to motor encoder
#define ENCODER_INTERRUPT_FR 2 //@def Interrupt attached to motor encoder

//E Servo parameters
#define P_SET_SERVO A3 //@def MegaMoto enable command output pin 
#define P_SERVO_PWM_A 11 ///@def Servo PWM A pin
#define P_SERVO_PWM_B 3 //@def Servo PWM B pin

#define P_SERVO_POT A2 //@def Servo pot input pin
#define SERVO_POT_MIN 724 //472@def Minimum reading on pot //E! TODO
#define SERVO_POT_MID 876 //@def Centered reading on pot //E! TODO
#define	SERVO_POT_MAX 963 //662@def Maximum reading on pot //E! TODO

//E PID parameters
#define MOTOR_PID_OUT_MIN -255 //@def Minimum motor PID output cmd //E! TODO
#define MOTOR_PID_OUT_MAX 255 //@def Maximum motor PID output cmd //E! TODO
#define KP_MOTOR 1 //@def Motor PID proportional gain //E! TODO
#define KI_MOTOR 1 //@def Motor PID integral gain //E! TODO
#define KD_MOTOR 1 //@def Motor PID differential gain //E! TODO

#define SERVO_PID_OUT_MIN -255 //@def Minimum servo PID output cmd //E! TODO
#define SERVO_PID_OUT_MAX 255 //@def Maximum servo PID output cmd //E! TODO
#define KP_SERVO 1 //@def Motor PID proportional gain //E! TODO
#define KI_SERVO 1 //@def Motor PID integral gain //E! TODO
#define KD_SERVO 1 //@def Motor PID differential gain //E! TODO

//E Control parameters
#define MOTOR_LOOP 5 //@def [ms] period of motor loop
#define STEER_LOOP 5 //@def [ms] period of steering loop
#define PUB_LOOP 10 //@def [ms] period of publishing loop

//E TXRX Parameters
#define P_RC_STEERING 40
#define P_RC_THROTTLE 41
#define P_RC_MODE     42
#define P_RC_OTHER    43


/**
 * @class Acid
 * @brief Main control class. Ties everything together.
**/
class Acid {
public:
	Acid(PhysCommander& pcommander, JetsonCommander& jcommander, MotorInterface& motor,
            ServoInterface &servo, int motor_interval, int steer_interval, int pub_interval, 
            SpeedSensor *motor_rpm_sensor,
            SpeedSensor *fl_rpm_sensor,
            SpeedSensor *fr_rpm_sensor,
            ros::NodeHandle *nh,
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
            ros::Publisher *brake_right_pub);
	void prep();
	void drop();
	void test();
  void printAll();
private:
	void doSpeed();
	void steer();
	void publish();
	void setAutonomy(int mode);

	int _motor_interval, _steer_interval, _pub_interval;

	PhysCommander& _pcommander;
	JetsonCommander& _jcommander;

	Commander* _commander;

	MotorInterface& _motor;
	ServoInterface& _servo;

	int _mode;

	unsigned long _t_last_motor, _t_last_steer, _t_last_pub;
  
  SpeedSensor *_motor_rpm_sensor;
  SpeedSensor *_fl_rpm_sensor;
  SpeedSensor *_fr_rpm_sensor;
  ros::NodeHandle *_nh;
  std_msgs::Int8 *_steer_angle_msg;
  std_msgs::UInt8 *_throttle_msg;
  std_msgs::UInt16 *_motor_rpm_msg;
  std_msgs::UInt16 *_fl_rpm_msg;
  std_msgs::UInt16 *_fr_rpm_msg;
  std_msgs::Bool *_drive_dir_msg;
  std_msgs::Bool *_brake_left_msg;
  std_msgs::Bool *_brake_right_msg;
  ros::Publisher *_steer_angle_pub;
  ros::Publisher *_throttle_pub;
  ros::Publisher *_motor_rpm_pub;
  ros::Publisher *_fl_rpm_pub;
  ros::Publisher *_fr_rpm_pub;
  ros::Publisher *_drive_dir_pub;
  ros::Publisher *_brake_left_pub;
  ros::Publisher *_brake_right_pub;
  
  //Read Sensors
  int8_t _steer_angle;
  unsigned int _throttle;
  unsigned int _motor_rpm;
  unsigned int _fl_rpm;
  unsigned int _fr_rpm;
  bool _drive_dir;
  bool _brake_left;
  bool _brake_right;
};

#endif
