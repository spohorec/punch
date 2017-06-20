/** 
	koolaid.ino
	Electric Kool-Aid Turing Test Arduino Code
	

	@date 2017-06-18 creation



**/

int mag_throttle_pin = 2; //E TODO make param

float max_bat_voltage = 34.0;

int min_mag_voltage = 5;

int max_mag_voltage = max_bat_voltage; //E TODO this needs to be variable somewhere... This whole thing should probably be encapsulated somewhere.

int ocr2a_val = 0;
int mag_throttle = 0;
float duty = 0.0;

float visible_voltage = 0.0;

float v_in = 0.0;

void setDuty() {
	visible_voltage = ((max_mag_voltage - min_mag_voltage) / 5.0 ) * v_in + min_mag_voltage;
	duty = visible_voltage / max_bat_voltage;
}

void setOCR2A(){
	 ocr2a_val = floor((duty * 256) - 1);
	 
	 if (ocr2a_val > 255) {
	 	ocr2a_val = 255;
	 } else if (ocr2a_val < 0) {
	 	ocr2a_val = 0;
	 }

	 OCR2A = ocr2a_val;
}
void setup(){

	Serial.begin(9600); 

	pinMode(11,OUTPUT);
		TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20); 	//E = 10000011
														//E datasheet pg. 158 COM2 = 10 --> Output A (pin 3?) to non-inverted PWM
														//E WGM2 to 011 --> Output A to fast PWM
		
		TCCR2B = _BV(CS21);								//E = 00000010
														//E datasheet pg. 162 (CS prescaler bit settings)
														//E 100 = prescaler 64; 010 = prescaler 8
		
		//E Frequency = 16 MHz / Prescaler / 256 = 7.8 kHz for prescaler 8

		OCR2A = 127;					//E 0 < OCR2A < 255
									//E D = (OCR2A + 1)/256

}

void loop(){
	mag_throttle = analogRead(mag_throttle_pin);
	v_in = mag_throttle / 1023.0 * 5;
	Serial.print(v_in);
    Serial.print("  ");
	setDuty();
	setOCR2A();
        Serial.print(duty);
            Serial.print("  ");
        Serial.println(ocr2a_val);
}



