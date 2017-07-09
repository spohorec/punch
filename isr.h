  /**
 * isr.h
 * Electric Koolaid interrupts.
 * 
 * @author  Bayley Wang       <bayleyw@mit.edu>
 * @author  Syler Wagner      <syler@mit.edu>
 * @author  Chris Desnoyers   <cjdesno@mit.edu>
 * @author	Sarah Pohorecky	  <spohorec@mit.edu>
 *
 * @date    2017-07-09    sarah   ported from 5yler/gigabug/isr.h
 *
 **/

#ifndef __ISR_H
#define __ISR_H

//$ left encoder
#define ENCODER_INTERRUPT 4 //E! TODO change to better standard
#define P_ENCODER_A 19 //E! update this
#define P_ENCODER_B 19 //E! update this
// #define ENCODER_REVERSED
 
extern volatile long encoder_ticks;

void EncoderISR();  //$ //E encoder interrupt service routine

#endif


