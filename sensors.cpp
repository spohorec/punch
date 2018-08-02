/**
 * sensors.cpp
 * Electric Kool-Aid Sensors
 * [SpeedSensor adapted from 5yler/gigabug/speedsensor.cpp]

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
 * @date	2017-07-13	  sarah   renamed speedsensor-->sensors, added AngleSensor outline
 * @date	2017-07-13 	  sarah   moved Throttle and Thermistor defs, minor type/name updates and doc
 * @date 	2017-07-14    sarah   refactored with references. Bug fixing.
 **/
 
#include "sensors.h"
#include <Arduino.h>


// ----------------------------------------------------------------------------------------------

volatile unsigned long encoder_ticks;  //E number of ticks recorded on the encoder
volatile unsigned long encoder_period;  //E period between encoder ticks, for low speed sensing. [microseconds]
volatile unsigned long encoder_time;  //E temp variable for low speed sensing. [microseconds]
volatile unsigned long encoder_time_prev;  //E temp variable for low speed sensing. [microseconds]

volatile unsigned long encoder_r_ticks;  //E number of ticks recorded on the encoder
volatile unsigned long encoder_r_period;  //E period between encoder ticks, for low speed sensing. [microseconds]
volatile unsigned long encoder_r_time;  //E temp variable for low speed sensing. [microseconds]
volatile unsigned long encoder_r_time_prev;  //E temp variable for low speed sensing. [microseconds]

/**
 * @func EncoderISR 
 * @brief Encoder Interrupt Service Routine. Increments tick counter. Works if motor RPM is high enough
**/
//void EncoderISR() { 
//  encoder_ticks+=1;
//}

/**
 * @func EncoderISR2
 * @brief Encoder Interrupt Service Routine. Resets a timer when a tick is detected. 
**/
void EncoderISR2() {
  encoder_r_time = micros();
  encoder_r_period = encoder_r_time - encoder_r_time_prev;
  encoder_r_time_prev = encoder_r_time;
}

// ----------------------------------------------------------------------------------------------

volatile unsigned long encoder_fr_ticks;  //E number of ticks recorded on the encoder
volatile unsigned long encoder_fr_period;  //E period between encoder ticks, for low speed sensing. [microseconds]
volatile unsigned long encoder_fr_time;  //E temp variable for low speed sensing. [microseconds]
volatile unsigned long encoder_fr_time_prev;  //E temp variable for low speed sensing. [microseconds]

/**
 * @func EncoderISR 
 * @brief Encoder Interrupt Service Routine. Increments tick counter. Works if motor RPM is high enough
**/
//void EncoderFRISR() { 
//  encoder_ticks+=1;
//}

/**
 * @func EncoderISR2
 * @brief Encoder Interrupt Service Routine. Resets a timer when a tick is detected. 
**/
void EncoderFRISR2() { 
  encoder_fr_time = micros();
  encoder_fr_period = encoder_fr_time - encoder_fr_time_prev;
  encoder_fr_time_prev = encoder_fr_time;
}
// ----------------------------------------------------------------------------------------------

volatile unsigned long encoder_fl_ticks;  //E number of ticks recorded on the encoder
volatile unsigned long encoder_fl_period;  //E period between encoder ticks, for low speed sensing. [microseconds]
volatile unsigned long encoder_fl_time;  //E temp variable for low speed sensing. [microseconds]
volatile unsigned long encoder_fl_time_prev;  //E temp variable for low speed sensing. [microseconds]

/**
 * @func EncoderISR 
 * @brief Encoder Interrupt Service Routine. Increments tick counter. Works if motor RPM is high enough
**/
//void EncoderFLISR() { 
//  encoder_ticks+=1;FALLING
//}

/**
 * @func EncoderISR2
 * @brief Encoder Interrupt Service Routine. Resets a timer when a tick is detected. 
**/
void EncoderFLISR2() { 
  encoder_fl_time = micros();
  encoder_fl_period = encoder_fl_time - encoder_fl_time_prev;
  encoder_fl_time_prev = encoder_fl_time;
}


// ----------------------------------------------------------------------------------------------


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
  encoder_period = 745000; //What is this?
  encoder_fr_period = 745000;
  encoder_fl_period = 745000;
  encoder_r_period = 745000;

  pinMode(_p_encoder, INPUT_PULLUP); //E encoder requires a pullup resistor
  switch(_interrupt){
    case 0:
      attachInterrupt(digitalPinToInterrupt(p_encoder), EncoderISR2, RISING);
      encoder_r_ticks = 0;
      break;
    case 1:
      attachInterrupt(digitalPinToInterrupt(p_encoder), EncoderFLISR2, FALLING);
      encoder_fl_ticks = 0;
      break;
    case 2:
      attachInterrupt(digitalPinToInterrupt(p_encoder), EncoderFRISR2, FALLING);
      encoder_fr_ticks = 0;
      break;
  }
  encoder_ticks = 0;
  _t_last_read = micros();
}

/**
 * @func SpeedSensor::getTicks
 * @brief gets number of ticks from encoder since last call 
 * @returns [long] ticks since last call
**/
long SpeedSensor::getTicks() {
  long ticks;
  prepTicks();
  ticks = encoder_ticks;
  encoder_ticks = 0;
  clearTicks();
  _t_last_read = millis(); //E updates last read time

  return ticks;
}

void SpeedSensor::prepTicks(){
  switch(_interrupt){
    case 0:
      encoder_ticks = encoder_r_ticks;
      break;
    case 1:
      encoder_ticks = encoder_fl_ticks;
      break;
    case 2:
      encoder_ticks = encoder_fr_ticks;
      break;
  }
}

void SpeedSensor::clearTicks(){
  switch(_interrupt){
    case 0:
      encoder_r_ticks = 0;
      break;
    case 1:
      encoder_fl_ticks = 0;
      break;
    case 2:
      encoder_fr_ticks = 0;
      break;
  }
}

void SpeedSensor::prepPeriod(){
  switch(_interrupt){
    case 0:
      encoder_period = encoder_r_period;
      break;
    case 1:
      encoder_period = encoder_fl_period;
      break;
    case 2:
      encoder_period = encoder_fr_period;
      break;
  }
}

/**
 * @func SpeedSensor::getRPM
 * @brief gets rpm of motor from recorded encoder ticks
 * @returns [long] rpm of motor
**/
long SpeedSensor::getRPM() {

  //MAYBE TODO FOR LOW SPEED SENSING: If ticks is 0, that means motor is spinning slower than 
  //  can be read. Shift to mode where we start a timer and end the timer when we read the 
  //  next tick. So that instead of #ticks/setPeriod it becomes 1tick/timeBetweenTicks. 
//      long t_last_read = _t_last_read; //E get time of last read since getTicks() will overwrite it
//      long ticks = getTicks();
//      long t_current_read = _t_last_read;
//      long dt = t_current_read - t_last_read; //E length of time since last read [ms]
//      double motor_revs = (double) ticks / _pulses_per_rev;

  //UPDATE: We need to do the other thing:

  double dt;
  double motor_revs = (double) 1.0/_pulses_per_rev;
  
  switch(_interrupt){
    case 0:
      dt = (double) encoder_r_period/1000.0; // encoder period in milliseconds
//      Serial.println(encoder_r_period);
      break;
    case 1:
      dt = (double) encoder_fl_period/1000.0; // encoder period in milliseconds
      break;
    case 2:
      dt = (double) encoder_fr_period/1000.0; // encoder period in milliseconds
      break;
  }
  
  double rpm = motor_revs / dt * (1000.0 * 60.0); //E calculates revs/ms and converts to revs/min
  if(rpm<100){
    rpm = 0;
  }
  return (long) rpm; 
}

// ----------------------------------------------------------------------------------------------

/**
 * @constr AngleSensor::AngleSensor
 * @brief initializes interface and angle sensor. Servo starts disabled
 * @param [int] <p_angle_sensor> pin connected to output of sensor
 * @param [double] <sensor_min_v> minimum sensor voltage reading
 * @param [double] <sensor_mid_v> mid-point (no angle) sensor voltage reading
 * @param [double] <sensor_max_v> maximum sensor voltage reading
**/
AngleSensor::AngleSensor(int p_angle_sensor, double sensor_min_v, 
    double sensor_mid_v, double sensor_max_v) {
  //E TODO
}

/**
 * @func AngleSensor::getAngle
 * @brief returns angle measurement from angle sensor
 * @returns [int] angle measurement from sensor
**/
int AngleSensor::getAngle() {
  //E TODO
  return 0;
}

// ----------------------------------------------------------------------------------------------

/**
 * @constr Throttle::Throttle
 * @brief initializes throttle, sets throttle limits
 * @param [int] <p_throttle> pin connected to output of throttle
 * @param [double] <throttle_min_v> minimum throttle voltage reading
 * @param [double] <throttle_max_v> maximum throttle voltage reading
**/
Throttle::Throttle(int p_throttle, double throttle_min_v, double throttle_max_v) {
  _p_throttle = p_throttle;
  _throttle_min = (int) (throttle_min_v / 5.0 * 1023); 
  _throttle_max = (int) (throttle_max_v / 5.0 * 1023);
}

/**
 * @func Throttle::getThrottle
 * @brief returns scaled throttle measurement
 * @returns [unsigned char] scaled throttle reading (0-255)
**/
unsigned char Throttle::getThrottle() {
  int raw_throttle = getRawThrottle();
  
  //E scales the throttle to an input between 0 and 255
  int scaled_throttle = (int) (double((raw_throttle - _throttle_min)) / (_throttle_max - _throttle_min) * (255.0-42.0))+42;

  if (scaled_throttle > 255) scaled_throttle = 255;
  if (scaled_throttle < 42) scaled_throttle = 42;

  return (unsigned char) scaled_throttle;
}

/**
 * @func Throttle::getRawThrottle
 * @brief returns unscaled throttle measurement
 * @returns [int] unscaled throttle reading (0-1023)
**/
int Throttle::getRawThrottle() {
  return analogRead(_p_throttle);
}

// ----------------------------------------------------------------------------------------------

/*
 * @constr Thermistor::Thermistor
 * @brief initializes conversion params and calculates R_infinity for temperature approximation function
 * @param [int] <p_thermistor> pin in center of voltage divider w/ thermistor to ground
 * @param [double] <reference_res> reference resistance (resistance of other resistor in divider)
 * @param [double] <T0> reference temperature (room temperature)
 * @param [double] <R0> reference resistance (resistance at room temperature)
 * @param [double] <B> thermistor B parameter
**/
//Thermistor::Thermistor(int p_thermistor, double reference_res, double T0, double R0, double B){
//	_p_thermistor = p_thermistor;
//
//	_reference_res = reference_res;
//
//	_T0 = T0;
//	_R0 = R0;
//	_B = B;
//
//	_rInf = _R0 * exp((-_B / _T0)); // See <https://en.wikipedia.org/wiki/Thermistor#B_or_.CE.B2_parameter_equation> for equation info
//
//}

/*
 * @func Thermistor::getTemperature
 * @brief reads voltage on thermistor divider pin and calculates its resistance, then approximates its temperature using B-parameter equation
 * @returns [double] approximated temperature of the motor
**/
//double Thermistor::getTemperature(){
//	double voltage = analogRead(_p_thermistor)/1023.0 * 5.0; //E! TODO this could totally be a helper function since I use it so much...
//	double res = (voltage * _reference_res) / (5.0 - voltage); //E thermistor is in resistor divider with reference resistance 
//																//E 5V--->[reference]--->(pin connected here)--->[therm]--->GND
//
//	double temp = _B / log(res / _rInf);
//
//	return temp;
//}
