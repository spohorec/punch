/*
throttle.h
Electric Kool-Aide Throttle class.

	@author Sarah Pohorecky <spohorec@mit.edu>

	@date 2017-07-08 creation.
	@date 2017-07-09 updating for non-zero input endpoints (ie. throttle reading does not go from 0-5V)

*/

#include <Arduino.h>

class Throttle {
public:
	Throttle(int throttle_pin, float throttle_min_v, float throttle_max_v);
	unsigned char getThrottle();
	int getRawThrottle();
private:
	int _throttle_pin;
	int _throttle_min, _throttle_max;
};