/*
commander.h
Electric Kool-Aide Commander

	@author Sarah Pohorecky <spohorec@mit.edu>

	@date 2017-07-08 creation.

*/

class Commander {
public:
	virtual unsigned char getMotorCmd();
	virtual unsigned char getFieldCmd();
	virtual int getSteeringCmd();
	virtual int getKillCmd();
	virtual int getEstop();

};


class PhysCommander: public Commander {
public:
	PhysCommander(Throttle* main_th, Throttle* field_th, int reverse_pin);
	int getMotorCmd();
	unsigned char getFieldCmd();
	int getSteeringCmd();
	int getKillCmd();
	int getEstop();
private:
	Throttle* _main_th, _field_th;
	int _reverse_pin;

};

class RCCommander: public Commander {
public:
	RCCommander(RCDecoder *motor_d, RCDecoder *field_d, RCDecoder *steer_d, RCDecoder *kill_d);
	unsigned char getMotorCmd();
	unsigned char getFieldCmd();
	int getSteeringCmd();
	int getKillCmd();
	int getEstop();
private:

};

class JetsonCommander: public Commander {
public:
	JetsonCommander(ros::NodeHandle *nh);
	unsigned char getMotorCmd();
	unsigned char getFieldCmd();
	int getSteeringCmd();
	int getKillCmd();
	int getEstop();

private:

};