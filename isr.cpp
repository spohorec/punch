  /**
 * isr.cpp
 * Electric Koolaid interrupts.
 * 
 * @author  Bayley Wang       <bayleyw@mit.edu>
 * @author  Syler Wagner      <syler@mit.edu>
 * @author  Chris Desnoyers   <cjdesno@mit.edu>
 * @author Sarah Pohorecky   <spohorec@mit.edu>

 * @date    2017-07-09    sarah   ported from 5yler/gigabug/isr.cpp
 *
 **/

#include <Arduino.h>
#include <digitalWriteFast.h>
#include "isr.h"

volatile long encoder_ticks;  //$ number of ticks for each encoder
volatile bool read_encoder;    //$ state of digitalRead(encoder pin B) for each encoder

//$ right encoder interrupt service routine
void EncoderISR() { 
  read_encoder = digitalReadFast(P_ENCODER_B); //$ read encoder input pin B
  //$ increment counter if A leads B
  #ifdef ENCODER_REVERSED //$ if left encoder is reversed
    //$ increment counter if B leads A
    encoder_ticks += read_encoder ? -1 : +1; //$ if (_read_left) {_ticks_left--;} else {_ticks_left++;}
  #else
    //$ increment counter if A leads B
    encoder_ticks += read_encoder ? +1 : -1; //$ if (_read_left) {_ticks_left++;} else {_ticks_left--;}
  #endif
}