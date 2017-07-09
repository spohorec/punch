/*
throttle.cpp
Electric Kool-Aide Throttle class.

	@author Sarah Pohorecky <spohorec@mit.edu>

	@date 2017-07-08 creation.
	@date 2017-07-09 updating for non-zero input endpoints (ie. throttle reading does not go from 0-5V)

*/

#include "throttle.h"

Throttle::Throttle(int throttle_pin, float throttle_min_v, float throttle_max_v) {
	_throttle_pin = throttle_pin;
	_throttle_min = (int) (throttle_min_v / 5.0 * 1023); 
	_throttle_max = (int) (throttle_max_v / 5.0 * 1023);
}

unsigned char Throttle::getThrottle() {
	int raw_throttle = getRawThrottle();

	//E scales the throttle to an input between 0 and 255
	int scaled_throttle = (int) (float((raw_throttle - _throttle_min)) / (_throttle_max - _throttle_min) * 255.0);

	if (scaled_throttle > 255) scaled_throttle = 255;
	if (scaled_throttle < 0) scaled_throttle = 0;

	return scaled_throttle;
}

int Throttle::getRawThrottle() {
	return analogRead(_throttle_pin);
}