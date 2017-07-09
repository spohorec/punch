/**
 * pidcontroller.cpp
 * Gigatron motor control Arduino code for class definitions.
 * 
 * @author  Bayley Wang   <bayleyw@mit.edu>
 *
 * @date    2016-01-10    syler   refactored header files
 *
 **/

#include <Arduino.h>

class SpeedSensor {
public:
  SpeedSensor(int interrupt, int interval);
  long GetTicks();
  long GetRPM();
private:
  int _interrupt;
  int _interval;
};

