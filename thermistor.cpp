/*
thermistor.cpp
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

/*
 * @constr Thermistor::Thermistor
 * @brief initializes conversion params and calculates R_infinity for temperature approximation function
 * @param [int] <p_thermistor> pin in center of voltage divider w/ thermistor to ground
 * @param [double] <reference_res> reference resistance (resistance of other resistor in divider)
 * @param [double] <T0> reference temperature (room temperature)
 * @param [double] <R0> reference resistance (resistance at room temperature)
 * @param [double] <B> thermistor B parameter
**/
Thermistor::Thermistor(int p_thermistor, double reference_res, double T0, double R0, double B){
	_p_thermistor = p_thermistor;

	_reference_res = reference_res;

	_T0 = T0;
	_R0 = R0;
	_B = B;

	_rInf = _R0 * exp((-_B / _T0)); // See <https://en.wikipedia.org/wiki/Thermistor#B_or_.CE.B2_parameter_equation> for equation info

}

/*
 * @func Thermistor::getTemperature
 * @brief reads voltage on thermistor divider pin and calculates its resistance, then approximates its temperature using B-parameter equation
**/
int Thermistor::getTemperature(){
	double voltage = analogRead(_p_thermistor)/1023.0 * 5.0; //E! TODO this could totally be a helper function since I use it so much...
	double res = (voltage * _reference_res) / (5.0 - voltage); //E thermistor is in resistor divider with reference resistance 
																//E 5V--->[reference]--->(pin connected here)--->[therm]--->GND

	double temp = _B / log(res / _rInf);

	return temp;
}