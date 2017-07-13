/**
 * pidcontroller.cpp
 * Electric Kool-Aid Motor Encoder Handling
 * [Ported from 5yler/gigabug/speedsensor.cpp]
 * 
 * @author  Bayley Wang 		<bayleyw@mit.edu>
 * @author 	Syler Wagner 		<syler@mit.edu>
 * @author  Sarah Pohorecky   	<spohorec@mit.edu>

 * @date    2016-01-10    syler   refactored header files
 * @date    2017-07-09    sarah   ported from gigabug, minor changes
 **/

#include <Arduino.h>

extern volatile long encoder_ticks;

void EncoderISR();  //$ //E encoder interrupt service routine

class SpeedSensor {
public:
  SpeedSensor(int interrupt, int interval);
  long GetTicks();
  long GetRPM();
private:
  int _interrupt;
  int _interval;
};

