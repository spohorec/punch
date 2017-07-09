/*
throttle.cpp
Electric Kool-Aide Throttle with curves.

	@author Sarah Pohorecky <spohorec@mit.edu>

	@date 2017-07-08 creation.

*/

class Throttle {
public:
	Throttle(int throttle_pin);
	getCmd();
	getRawThrottle();
private:
	int _throttle_pin;
};

Throttle::Throttle(int throttle_pin, int (*map) (int)) {
	_throttle_pin = throttle_pin;
	_map = map;
}

Throttle::getCmd() {
	int raw_throttle = getRawThrottle();
	return _map(raw_throttle);
}

Throttle::getRawThrottle() {
	return analogRead(_throttle_pin);
}