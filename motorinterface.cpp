/**
 * motorinterface.cpp
 * Electric Kool-Aide Motor Controller-Controllers
 * 
 * 	@author Sarah Pohorecky <spohorec@mit.edu>
 * 
 * 	@date 2017-07-08 creation.
 * 	@date 2017-07-13 update with reverse handling.
 * 	@date 2017-07-13 refactor, hopefully more reasonably?
 *	@date 2017-07-13 added ServoInterface outline.
 *  @date 2017-07-14 refactored with references. Bug fixing.
**/

#include "motorinterface.h"

/**
 * @constr MotorInterface::MotorInterface
 * @brief initializes settings, sensors, and readings for motor.
 * @param [int] <p_motor_pwm> pin tied to input to motor controller
 * @param [int] <p_regen_pwm> pin tied to input to field controller
 * @param [int] <p_reverse_switch> pin tied to Kelly KEB reverse switch
 * @param [int] <regen_min_field> minimum field inp needed for regen to function
 * @param [FieldInterface] <field> field interface object to control field
 * @param [PIDController] <motor_pid> PID Controller for the motors (speed loop)
 * @param [SpeedSensor] <encoder> motor encoder interface
**/
MotorInterface::MotorInterface(int p_motor_pwm, int p_regen_pwm, int p_reverse_switch, int regen_min_field, FieldInterface& field, PIDController& motor_pid, SpeedSensor& encoder):
		_encoder(encoder),
		_motor_pid(motor_pid),
		_field(field) {

	_p_motor_pwm = p_motor_pwm;
	_p_regen_pwm = p_regen_pwm;

	_p_reverse_switch = p_reverse_switch;
	pinMode(_p_reverse_switch,INPUT); //E Kelly switches are ON when GROUNDED. This makes switch floating (off)

	_regen_min_field = regen_min_field;

	_reverse_on = false;
	_regen_on = false;
	_pid_on = false;

	_last_motor_cmd = 0; //E last commands sent to interface by commander
	_last_field_cmd = 0;
	_last_regen_cmd = 0;

	_last_motor_input = 0; //E last actual inputs interface sent to motors (may differ from commands)
	_last_field_input = 0;
	_last_regen_input = 0;

	_last_rpm = 0; //E last RPM reading from _encoder

}

/**
 * @func MotorInterface::handleCmds
 * @brief takes in commands from commander, runs handling routines.
 * @param [int] <motor_cmd> command to main motor (0-255)
 * @param [int] <field_cmd> command to field controller (0-255)
 * @param [int] <regen_cmd> command to regen (0-255)
**/
void MotorInterface::handleCmds(int motor_cmd, int field_cmd, int regen_cmd) {
	_last_motor_cmd = motor_cmd;
	_last_field_cmd = field_cmd;
	_last_regen_cmd = regen_cmd;

	_last_rpm = _encoder.getRPM();

	handleField(); //E handleField() is called first since the command has to be adjusted based on other inputs
	handleRegen(); 
	handleMotor();
}

std::array<int,3> MotorInterface::getLastInps(){
	return {_last_motor_input, _last_field_input, _last_regen_input};
}

/**
 * @func PRIVATE MotorInterface::handleField
 * @brief increases field if needed for regen, calls adjustment on field limits, sends field command to field_interface
**/
void MotorInterface::handleField() {
	int field_input = _last_field_cmd;

	if (_last_regen_cmd > 0 && field_input < _regen_min_field) { //E increase field voltage if too low for regen
		field_input = _regen_min_field;
	}

	_field.setMaxVoltage(_last_rpm); //E command field_interface to set speed-based voltage limits

	if (field_input > 255) field_input = 255; //E check range
	if (field_input < 0 ) field_input = 0;

	_field.sendCmd(field_input);
	_last_field_input = field_input;
}

/**
 * @func PRIVATE MotorInterface::handleRegen
 * @brief gets regen command, verifies range, and writes command
**/
void MotorInterface::handleRegen() {
	int regen_input = _last_regen_cmd;

	if (regen_input > 255) regen_input = 255; //E check range
	if (regen_input < 0) regen_input = 0;

	analogWrite(_p_regen_pwm,regen_input);
	_last_regen_input = regen_input;
}

/**
 * @func PRIVATE MotorInterface::handleMotor
 * @brief gets motor command, handles direction, calls PID if enabled, writes command
**/
void MotorInterface::handleMotor() {
	int motor_input = _last_motor_cmd;

	//E check direction
	if (motor_input < 0) {
		setReverseOn(true);
		motor_input = abs(motor_input); //E can only write pos speed to motor
	} else {
		setReverseOn(false);
	}
	
	if (_pid_on) { //E use PID controller if enabled
		motor_input = _motor_pid.update(motor_input,_last_rpm);
	}

	if (motor_input > 255) motor_input = 255; //E check range
	if (motor_input < 0) motor_input = 0;

	analogWrite(_p_motor_pwm,motor_input);
	_last_motor_input = motor_input;
}

/**
 * @func MotorInterface::usePID
 * @brief turns PID controller on or off for motor. Resets integrator when re-enabled
 * @param [bool] <pid_on> whether pid should be on (true) or off (false)
**/
void MotorInterface::usePID(bool pid_on) {
	if (pid_on) {
		_pid_on = true;
		_motor_pid.resetIntegrator(); //E reset integrator to prevent weirdness
	} else {
		_pid_on = false;
	}
}

/**
 * @func PRIVATE MotorInterface::setReverseOn
 * @brief writes to kelly switch that controls motor directionality
 * @param [bool] <reverse_on> whether reverse is on (true) or off (false)
**/
void MotorInterface::setReverseOn(bool reverse_on) {
	if 	(_reverse_on != reverse_on) { //E don't bother changing pin modes if we are still going in the same direction
		if (reverse_on) {
			pinMode(_p_reverse_switch,OUTPUT);
			digitalWrite(_p_reverse_switch,LOW); //E Kelly switches are ON when GROUNDED
		} else {
			pinMode(_p_reverse_switch,INPUT); //E this makes pin floating
		}
		_reverse_on = reverse_on;
	} 
}

// ----------------------------------------------------------------------------------

/**
 * @constr FieldInterface::FieldInterface
 * @brief initializes settings, sensors, and readings for field controller.
 * @param [double] <min_v> minimum voltage of field
 * @param [double] <slow_max_v> maximum voltage of field at low speed
 * @param [double] <fast_max_v> maximum voltage of field at high speed
 * @param [long] <rpm_lower_limit> rpm below which max_field_v should be set to slow_max_v 
 * @param [long] <rpm_upper_limit> rpm above which max_field_v should be set to fast_max_v 
 * @param [double] <overheat_temperature> temperature at which to set temperature warning indicator
 * @param [int] <p_temp_indicator> pin attached to input of temperature warning indicator 
 * @param [Thermistor] <thermistor> thermistor object for getting motor temperature
**/
FieldInterface::FieldInterface(double min_v, 
	double slow_max_v, double fast_max_v, 
	long rpm_lower_limit, long rpm_upper_limit, 
	double overheat_temperature, int p_temp_indicator, Thermistor& thermistor)
	 : _thermistor(thermistor) {

	_slow_max_v = slow_max_v;
	_fast_max_v = fast_max_v;

	_min_v = min_v;
	_max_v = _slow_max_v;

	_rpm_lower_limit = rpm_lower_limit;
	_rpm_upper_limit = rpm_upper_limit;

	_temperature = 0;
	_overheat_temperature = overheat_temperature;
	_p_temp_indicator = p_temp_indicator;

}

/**
 * @func FieldInterface::sendCmd
 * @brief takes cmd (interpreted as % max_field_voltage), calculates PWM and writes fast PWM
 * @param [int] <cmd> (0-255) desired % max field voltage
**/
void FieldInterface::sendCmd(int cmd) {
	checkOverheat(); //E check for overheating

	double v_desired = ((float) cmd / 255) * (_max_v - _min_v) + _min_v;
	double duty = v_desired / V_BATTERY_MAX;
	int reg_value = floor(duty*256) - 1; //E duty cycle of PWM is given by (reg_value + 1)/256

	if (reg_value > 255) reg_value = 255;
	if (reg_value < 0) reg_value = 0; //E? Should this be -1 since val+1 gives duty?

	//E write reg value
	OCR2A = reg_value; //E! This needs to be changed if arduino platform is switched

}

/**
 * @func FieldInterface::setMaxVoltage
 * @brief takes rpm of motor and sets speed-based field voltage limits
 * @param [int] <motor_rpm> rpm of motor
**/
void FieldInterface::setMaxVoltage(int motor_rpm) {
	if (motor_rpm <= _rpm_lower_limit) {
		_max_v = _slow_max_v;
	} else if (motor_rpm >= _rpm_upper_limit) {
		_max_v = _fast_max_v;
	}
}

/**
 * @func PRIVATE FieldInterface::checkOverheat
 * @brief reads temperature from thermistor and sets warning indicator if applicable
**/
void FieldInterface::checkOverheat() {
	_temperature = _thermistor.getTemperature();

	if (_temperature >= _overheat_temperature) {
		digitalWrite(_p_temp_indicator,HIGH);
	} else {
		digitalWrite(_p_temp_indicator,LOW);
	}
}

// ----------------------------------------------------------------------------------

/**
 * @constr ServoInterface::ServoInterface
 * @brief initializes interface and angle sensor. Servo starts disabled
 * @param [int] <p_servo_pwmA> pin connected to PWM A input of MegaMoto Controller
 * @param [int] <p_servo_pwmB> pin connected to PWM B input of MegaMoto Controller
 * @param [int] <p_servo_enable> pin connected to enable input of MegaMoto Controller
 * @param [AngleSensor] <servo_pot> servo feedback potentiometer
 * @param [PIDController] <servo_pid> servo PID Controller
**/
ServoInterface::ServoInterface(int p_servo_pwmA, int p_servo_pwmB, int p_servo_enable, AngleSensor& servo_pot, PIDController& servo_pid)
		: _servo_pot(servo_pot),
		  _servo_pid(servo_pid) {
	_p_servo_pwmA = p_servo_pwmA;
	_p_servo_pwmB = p_servo_pwmB;
	_p_servo_enable = p_servo_enable;

	_last_angle_command = 0;
	_last_angle_input = 0;

	_last_angle = 0;

	_servo_on = false;
	_pid_on = false;

	//E! TODO Initialize pins
}

/**
 * @func ServoInterface::handleCmd
 * @brief takes angle command from commander, reads pot, adjusts with pid if enabled, and writes to servo (if enabled) 
 * @param [int] <cmd> angle command to servo (0-255)
**/
void ServoInterface::handleCmd(int cmd) {
	//E TODO
}

/**
 * @func MotorInterface::useServo
 * @brief enables or disables servo input (since servo should not be enabled when a human is driving)
 * @param [bool] <servo_on> whether servo should be enabled (true) or disabled (false)
**/
void ServoInterface::useServo(bool servo_on) {
	//E TODO
}

/**
 * @func ServoInterface::usePID
 * @brief turns PID controller on or off for servo. Resets integrator when re-enabled
 * @param [bool] <pid_on> whether pid should be on (true) or off (false)
**/
void ServoInterface::usePID(bool pid_on) {
	if (pid_on) {
		_pid_on = true;
		_servo_pid.resetIntegrator(); //E reset integrator to prevent weirdness
	} else {
		_pid_on = false;
	}
}