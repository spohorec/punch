/** 
	koolaid.ino
	Electric Kool-Aid Turing Test Arduino Code
	

	@date 2017-06-18 creation
  @author  Daniel Gonzalez   <dgonz@mit.edu>


**/

#define USE_USBCON
#include <ros.h>
//#include <Arduino.h>
//#include <digitalWriteFast.h>
#include "acid.h"

/*  ROS Messages:
 *   Feedback: 
 *   - Steer Angle int
 *   - Throttle    int
 *   - motor speed int
 *   - left brake  bool
 *   - right brake bool
 */
//#include <geometry_msgs/Vector3.h>  //$ for gain adjustment
#include <std_msgs/Int16.h>         //$ for mode publishing
#include <std_msgs/UInt16.h>         //$ for mode publishing
#include <std_msgs/Int8.h>         //$ for mode publishing
#include <std_msgs/UInt8.h>         //$ for mode publishing
#include <std_msgs/Bool.h>          //$ for estop subscriber

void setup(){
//  delay(1000); 
  Serial.begin(57600);
//  delay(500); 
//  Serial.print("AYY"); // LEAVE THIS HERE, for some reason, it crashes if there's no print statement around here.
//  delay(500); 
  ros::NodeHandle nh;       //$ node handle

  std_msgs::Int8 steer_angle_msg;
  std_msgs::UInt8 throttle_msg;
  std_msgs::UInt16 motor_rpm_msg;
  std_msgs::UInt16 fl_rpm_msg;
  std_msgs::UInt16 fr_rpm_msg;
  std_msgs::Bool drive_dir_msg;
  std_msgs::Bool brake_left_msg;
  std_msgs::Bool brake_right_msg;
 
  //set up publishers
  ros::Publisher steer_angle_pub("arduino/steer_angle", &steer_angle_msg);
  ros::Publisher throttle_pub("arduino/throttle", &throttle_msg);
  ros::Publisher motor_rpm_pub("arduino/motor_rpm", &motor_rpm_msg);
  ros::Publisher fl_rpm_pub("arduino/fl_rpm", &fl_rpm_msg);
  ros::Publisher fr_rpm_pub("arduino/fr_rpm", &fr_rpm_msg);
  ros::Publisher drive_dir_pub("arduino/drive_dir", &drive_dir_msg);
  ros::Publisher brake_left_pub("arduino/brake_left", &brake_left_msg);
  ros::Publisher brake_right_pub("arduino/brake_right", &brake_right_msg);
  
  nh.initNode();
  nh.advertise(steer_angle_pub);
  nh.advertise(throttle_pub);
  nh.advertise(motor_rpm_pub);
  nh.advertise(fl_rpm_pub);
  nh.advertise(fr_rpm_pub);
  nh.advertise(drive_dir_pub);
  nh.advertise(brake_left_pub);
  nh.advertise(brake_right_pub);

  pinMode(40,INPUT);
  pinMode(41,INPUT);
  pinMode(42,INPUT);
  pinMode(43,INPUT);

  /***********What is this?*****************/
//	TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20); //E = 10000011
//													//E datasheet pg. 158 COM2 = 10 --> Output A (pin 3?) to non-inverted PWM
//													//E WGM2 to 011 --> Output A to fast PWM
//	
//	TCCR2B = _BV(CS21);								//E = 00000010
//													//E datasheet pg. 162 (CS prescaler bit settings)
//													//E 100 = prescaler 64; 010 = prescaler 8
//	
//	//E Frequency = 16 MHz / Prescaler / 256 = 7.8 kHz for prescaler 8
//
//	OCR2A = 0;					//E 0 < OCR2A < 255
//								//E D = (OCR2A + 1)/256
               
// MotorInferface kelly(P_MOTOR_PWM,P_REGEN_PWM,0);
// PhysCommander commander(P_MOTOR_THROTTLE);

	AngleSensor servo_pot(P_SERVO_POT,SERVO_POT_MIN,SERVO_POT_MID,SERVO_POT_MAX);
 
	SpeedSensor speed_sensor(P_ENCODER,ENCODER_INTERRUPT,PULSES_PER_REV);
 
  SpeedSensor fl_speed_sensor(P_ENCODER_FL, ENCODER_INTERRUPT_FL, PULSES_PER_REV_FRONT);
  
  SpeedSensor fr_speed_sensor(P_ENCODER_FR, ENCODER_INTERRUPT_FR, PULSES_PER_REV_FRONT);

//	Thermistor thermistor(P_THERMISTOR,THERM_REF_RES,THERM_T0,THERM_R0,THERM_B);

	Throttle motor_throttle(P_MOTOR_THROTTLE,MOTOR_THROTTLE_MIN_V,MOTOR_THROTTLE_MAX_V);

	PIDController motor_pid(KP_MOTOR,KI_MOTOR,KD_MOTOR,MOTOR_PID_OUT_MAX,MOTOR_PID_OUT_MIN);
	PIDController steering_pid(KP_SERVO,KI_SERVO,KD_SERVO,SERVO_PID_OUT_MAX,SERVO_PID_OUT_MIN);
	
  MotorInterface motor(P_MOTOR_PWM,P_REGEN_PWM,P_SET_REVERSE,motor_pid,speed_sensor);
	ServoInterface servo(P_SERVO_PWM_A,P_SERVO_PWM_B,P_SET_SERVO,servo_pot,steering_pid);

	PhysCommander pcommander(P_REVERSE_SWITCH, P_AUTONOMY_SWITCH ,P_BRAKE_1,P_BRAKE_2,motor_throttle);
	JetsonCommander jcommander;


	Acid acid(pcommander,jcommander,motor,servo,MOTOR_LOOP,STEER_LOOP,PUB_LOOP, 
	                                           &speed_sensor,
	                                           &fl_speed_sensor, &fr_speed_sensor, 
	                                           &nh,
	                                           &steer_angle_msg, &throttle_msg, &motor_rpm_msg, 
	                                           &fl_rpm_msg, &fr_rpm_msg, 
	                                           &drive_dir_msg, &brake_left_msg, &brake_right_msg, &steer_angle_pub, &throttle_pub, &motor_rpm_pub, 
	                                           &fl_rpm_pub, &fr_rpm_pub, 
	                                           &drive_dir_pub, &brake_left_pub, &brake_right_pub);
	acid.prep();  
//  delay(1000); 
//  Serial.println("YEEEEEEEEEEE "); // LEAVE THIS HERE, for some reason, it crashes if there's no print statement around here. 
	acid.drop();

	// acid.test();
}


void loop(){
	//E Do nothing!
//  Serial.print("LMAO");
//  delay(1000);
}



