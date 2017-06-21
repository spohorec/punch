/** 
	koolaid.ino
	Electric Kool-Aid Turing Test Arduino Code
	

	@date 2017-06-18 creation



**/

const int FIELD_THROTTLE_PIN = 2; //E TODO make param

const double MAX_BATTERY_VOLTAGE = 34.0;
const int PWM_PIN = 11;

float minFieldVoltage = 5.0;
float maxFieldVoltage = MAX_BATTERY_VOLTAGE;

int fieldThrottle, ocr2a_val;
double fieldVoltage,duty;

void setFieldPWM() {
	fieldThrottle = analogRead(FIELD_THROTTLE_PIN); //E reads field throttle

	fieldVoltage = ((maxFieldVoltage - minFieldVoltage) / 1023.0) * fieldThrottle + minFieldVoltage; //E maps throttle position to desired field voltage

	duty = fieldVoltage / MAX_BATTERY_VOLTAGE;

	ocr2a_val = floor((duty*256) - 1);

	 if (ocr2a_val > 255) {
	 	ocr2a_val = 255;
	 } else if (ocr2a_val < 0) {
	 	ocr2a_val = 0;
	 }
         Serial.println(duty);
	 OCR2A = ocr2a_val;
}

void setup(){

	Serial.begin(9600); 

	//E Setup Fast PWM on pin 11
	pinMode(PWM_PIN,OUTPUT);
		TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20); //E = 10000011
														//E datasheet pg. 158 COM2 = 10 --> Output A (pin 3?) to non-inverted PWM
														//E WGM2 to 011 --> Output A to fast PWM
		
		TCCR2B = _BV(CS21);								//E = 00000010
														//E datasheet pg. 162 (CS prescaler bit settings)
														//E 100 = prescaler 64; 010 = prescaler 8
		
		//E Frequency = 16 MHz / Prescaler / 256 = 7.8 kHz for prescaler 8

		OCR2A = 0;					//E 0 < OCR2A < 255
									//E D = (OCR2A + 1)/256

}

void loop(){
	setFieldPWM();
}



