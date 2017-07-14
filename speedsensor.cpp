/**
 * speedsensor.cpp
 * Electric Kool-Aid Motor Encoder Handling
 * [Ported from 5yler/gigabug/speedsensor.cpp]

 * @author  Bayley Wang       <bayleyw@mit.edu>
 * @author  Chris Desnoyers   <cjdesno@mit.edu>
 * @author  Daniel Gonzalez   <dgonz@mit.edu>
 * @author  Syler Wagner      <syler@mit.edu>
 * @author  Sarah Pohorecky   <spohorec@mit.edu>
 *
 * @date    2016-01-10    syler   moved to separate .cpp file, header is in classes.h
 * @date    2016-03-27    syler   fixed RPM calculation for new quadrature encoders and cleaned up encoder interrupt pin setup
 * @date    2017-07-09    sarah   ported from gigabug, minor changes
 * @date    2017-07-13    sarah   adapted for Kool-Aid setup (single encoder, not quadrature)
 * @date    2017-07-13    sarah   more general RPM calc and interrupt setup, commented
 **/

#include "speedsensor.h"

#define PULSES_PER_REV 600.0 //$ number of encoder pulses per full motor revolution //E! TODO Confirm this value for Koolaid and put elsewhere

volatile long encoder_ticks;  //E number of ticks recorded on the encoder

/**
 * @func EncoderISR
 * @brief Encoder Interrupt Service Routine. Increments tick counter.
**/
void EncoderISR() { 
	encoder_ticks+=1;
}

/**
 * @constr SpeedSensor::SpeedSensor
 * @brief clears encoder reading and sets up interrupt routine for encoder
 * @param [int] <p_encoder> pin attached to the output of the encoder
 * @param [int] <interrupt> interrupt to attach to encoder (on Uno, either 0(D2) or 1(D3))
 * @param [double] <pulses_per_rev> number of encoder pulses per revolution of motor
 **/
SpeedSensor::SpeedSensor(int p_encoder, int interrupt, double pulses_per_rev) {
	_p_encoder = p_encoder;
	_interrupt = interrupt;
	_pulses_per_rev = pulses_per_rev;

	pinMode(_p_encoder, INPUT_PULLUP); //E encoder requires a pullup resistor
	attachInterrupt(_interrupt, EncoderISR, FALLING); //E! TODO update this to attachPinToInterrupt()?
	encoder_ticks = 0;
	
	_t_last_read = millis();
}

/**
 * @func SpeedSensor::getTicks
 * @brief gets number of ticks from encoder since last call 
 * @returns [long] ticks since last call
**/
long SpeedSensor::getTicks() {
	long ticks;

	ticks = encoder_ticks;
	encoder_ticks = 0;

	_t_last_read = millis(); //E updates last read time

	return ticks;
}

/**
 * @func SpeedSensor::getRPM
 * @brief gets rpm of motor from recorded encoder ticks
 * @returns [long] rpm of motor
**/
long SpeedSensor::getRPM() {

	long t_last_read = _t_last_read; //E get time of last read since getTicks() will overwrite it
	long ticks = getTicks();

	long t_current_read = _t_last_read;
	long dt = t_current_read - t_last_read; //E length of time since last read [ms]


	double motor_revs = (double) ticks / _pulses_per_rev;

	double rpm = motor_revs / dt * (1000 * 60.0); //E calculates revs/ms and converts to revs/min

	return (long) rpm; 

}

