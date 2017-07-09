/**
 * speedsensor.cpp
 * Gigatron motor control Arduino code for Hall Effect sensors.
 * 
 * @author  Bayley Wang       <bayleyw@mit.edu>
 * @author  Chris Desnoyers   <cjdesno@mit.edu>
 * @author  Daniel Gonzalez   <dgonz@mit.edu>
 * @author  Syler Wagner      <syler@mit.edu>
 *
 * @date    2016-01-10    syler   moved to separate .cpp file, header is in classes.h
 * @date    2016-03-27    syler   fixed RPM calculation for new quadrature encoders and cleaned up encoder interrupt pin setup
 *
 **/

#include "speedsensor.h"
#include "isr.h"

#define PULSES_PER_REV 600.0 //$ number of encoder pulses per full motor revolution

SpeedSensor::SpeedSensor(int interrupt, int interval) {
  _interrupt = interrupt;
  _interval = interval;
  //interval is set in gigatron.ino; it is the interval at which the context loop runs, 
  //and not related to any property of the encoders

  //see https://www.arduino.cc/en/Reference/AttachInterrupt for interrupt documentation
  //in summary, attachInterrupt(4) likely attaches an interrupt to pin 18, and does not attach one to pin 4

  /*$ actually, the link above says the Mega2560 has the following mapping:
      Interrupt     0   1   2   3   4   5
      Mega2560 pin  2   3   21  20  19  18
  */

  pinMode(P_ENCODER_A, INPUT_PULLUP);
  pinMode(P_ENCODER_B, INPUT_PULLUP);

  attachInterrupt(ENCODER_INTERRUPT, EncoderISR, FALLING);
  
  encoder_ticks = 0;
}

//$ returns number of ticks per S_LOOP_INTERVAL
long SpeedSensor::GetTicks() {
  long ticks;

    ticks = encoder_ticks;
    encoder_ticks = 0;

  return ticks;
}


//$ TODO: this is unsigned, need to fix!
long SpeedSensor::GetRPM() {
  long ticks;

    ticks = encoder_ticks;
    encoder_ticks = 0;
  
  double motor_revs = (double) ticks / PULSES_PER_REV;
  double rpm = motor_revs * (60.0 * 1000) / _interval;

  return (long) rpm; 

}

