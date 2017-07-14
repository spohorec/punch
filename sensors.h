/**
 * sensors.cpp
 * Electric Kool-Aid Sensors
 * [SpeedSensor adapted from 5yler/gigabug/speedsensor.cpp]
 * 
 * @author  Bayley Wang 		<bayleyw@mit.edu>
 * @author 	Syler Wagner 		<syler@mit.edu>
 * @author  Sarah Pohorecky   	<spohorec@mit.edu>

 * @date    2016-01-10	syler   refactored header files
 * @date    2017-07-09	sarah   ported from gigabug, minor changes
 * @date	2017-07-13	sarah 	more general RPM calc and interrupt setup, commented
 * @date	2017-07-13	sarah 	renamed speedsensor-->sensors, added AngleSensor outline
 **/

#include <Arduino.h>

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