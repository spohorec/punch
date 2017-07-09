/** 
	koolaid.ino
	Electric Kool-Aid Turing Test Arduino Code
	

	@date 2017-06-18 creation



**/

#include "acid.h"
#include <Arduino.h>

void setup(){

	Serial.begin(9600); 

	//E Setup Fast PWM on pin 11
	pinMode(P_FIELD_PWM,OUTPUT);
		TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20); //E = 10000011
														//E datasheet pg. 158 COM2 = 10 --> Output A (pin 3?) to non-inverted PWM
														//E WGM2 to 011 --> Output A to fast PWM
		
		TCCR2B = _BV(CS21);								//E = 00000010
														//E datasheet pg. 162 (CS prescaler bit settings)
														//E 100 = prescaler 64; 010 = prescaler 8
		
		//E Frequency = 16 MHz / Prescaler / 256 = 7.8 kHz for prescaler 8

		OCR2A = 0;					//E 0 < OCR2A < 255
									//E D = (OCR2A + 1)/256



	// MotorInferface kelly(P_MOTOR_PWM,P_REGEN_PWM,0);
	// FieldInterface field();
	// PhysCommander commander(P_MOTOR_THROTTLE, P_FIELD_THROTTLE);

	Acid acid;
	acid.prep();

	acid.drop();

}


void loop(){
}



