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
 **/

#include "speedsensor.h"

#define PULSES_PER_REV 600.0 //$ number of encoder pulses per full motor revolution //E! TODO Confirm this value for Koolaid

volatile long encoder_ticks;  //$ number of ticks for each encoder

//E Encoder Interrupt Service Routine
void EncoderISR() { 
  encoder_ticks+=1;
}

SpeedSensor::SpeedSensor(int interrupt, int interval) {
  _interrupt = interrupt; //E The Uno only has two interrupts (Int0 --> D2, Int1 --> D3)
  _interval = interval; // Interval at which the main loop runs, not related to any property of the encoders

  pinMode(P_ENCODER, INPUT_PULLUP);

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


//$ TODO: this is unsigned, need to fix! //E! TODO still need to do this, but maybe not here...
long SpeedSensor::GetRPM() {
  long ticks;

    ticks = encoder_ticks;
    encoder_ticks = 0;
  
  double motor_revs = (double) ticks / PULSES_PER_REV;
  double rpm = motor_revs * (60.0 * 1000) / _interval;

  return (long) rpm; 

}

