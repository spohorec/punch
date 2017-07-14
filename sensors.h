/**
 * sensors.cpp
 * Electric Kool-Aid Sensors
 * [SpeedSensor adapted from 5yler/gigabug/speedsensor.cpp]
 * 
 * @author  Bayley Wang 		<bayleyw@mit.edu>
 * @author 	Syler Wagner 		<syler@mit.edu>
 * @author  Sarah Pohorecky   	<spohorec@mit.edu>

 * @date  2016-01-10	syler   refactored header files
 * @date  2017-07-09	sarah   ported from gigabug, minor changes
 * @date	2017-07-13	sarah 	more general RPM calc and interrupt setup, commented
 * @date	2017-07-13	sarah 	renamed speedsensor-->sensors, added AngleSensor outline
 * @date  2017-07-13  sarah   moved Throttle and Thermistor defs, minor type/name updates and doc
 * @date 2017-07-14   sarah   refactored with references. Bug fixing. Added header guards.
 **/

#ifndef __SENSORS_H
#define __SENSORS_H

// ----------------------------------------------------------------------------------------------


extern volatile long encoder_ticks;

void EncoderISR();

// ----------------------------------------------------------------------------------------------

/**
 * @class SpeedSensor
 * @brief reads values from motor encoder and calculates RPM
**/
class SpeedSensor {
public:
  SpeedSensor(int p_encoder, int interrupt, double pulses_per_rev);
  long getTicks();
  long getRPM();
private:
  int _p_encoder, _interrupt;
  long _t_last_read;
  double _pulses_per_rev;
};

// ----------------------------------------------------------------------------------------------

/**
 * @class AngleSensor
 * @brief angle sensor interface (ie. potentiometer)
**/
class AngleSensor {
public:
	AngleSensor(int p_angle_sensor, double sensor_min_v, 
		double sensor_mid_v, double sensor_max_v);
	int getAngle();
private:
	int _p_angle_sensor;
	int _sensor_min, _sensor_mid, _sensor_max;

};

// ----------------------------------------------------------------------------------------------

/**
 * @class Throttle
 * @brief reads throttle input and scales range
**/
class Throttle {
public:
  Throttle(int p_throttle, double throttle_min_v, double throttle_max_v);
  unsigned char getThrottle();
  int getRawThrottle();
private:
  int _p_throttle;
  int _throttle_min, _throttle_max;
};

// ----------------------------------------------------------------------------------------------

/**
 * @class Thermistor
 * @brief handles readings from thermistor and converts them into mesaured temperature
**/
class Thermistor {
public:
  Thermistor(int p_thermistor, double reference_res, double T0, double R0, double B);
  double getTemperature();
private:
  int _p_thermistor;
  double _reference_res;
  double _T0, _R0, _B, _rInf;
};

#endif