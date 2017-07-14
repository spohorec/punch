/**
 * pidcontroller.cpp
 * Electric Kool-Aid Motor Encoder Handling
 * [Ported from 5yler/gigabug/speedsensor.cpp]
 * 
 * @author  Bayley Wang 		<bayleyw@mit.edu>
 * @author 	Syler Wagner 		<syler@mit.edu>
 * @author  Sarah Pohorecky   	<spohorec@mit.edu>

 * @date    2016-01-10	syler   refactored header files
 * @date    2017-07-09	sarah   ported from gigabug, minor changes
 * @date	2017-07-13	sarah 	more general RPM calc and interrupt setup, commented
 **/

#include <Arduino.h>

extern volatile long encoder_ticks;

void EncoderISR();

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

