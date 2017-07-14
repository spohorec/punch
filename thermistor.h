/*
thermistor.h
Electric Kool-Aide Temperature Monitor

	@author Sarah Pohorecky <spohorec@mit.edu>

	@date 2017-07-13 creation.

*/

/*
 * @class Thermistor
 * @brief handles readings from thermistor and converts them into mesaured temperature
**/
class Thermistor {
public:
	Thermistor(int p_thermistor, double T0, double R0, double B);
	int getTemperature();
private:
	int _p_thermistor;
	double _T0, _R0, _B, _rInf;
};