# punch
Arduino Code for Electric Kool-aid Acid Test
------

### What's What?

* `punch.ino`
  - Main arduino file.
* `acid.cpp`
  - Where the magic happens. The main loop that takes commander inputs and sends commands to the motors lives here. Most constants are defined in its header. 
* `commander.cpp`
  - Interface for whatever is giving commands to the car. This currently includes PhysCommander for a real human driver and JetsonCommander for getting commands from the computer.
* `motorinterface.cpp`
  - Interface classes for the 3 different motor-like devices on the car (servo, field generator, and main motor). Takes inputs from commander and feedback sensor readings (hall sensor, servo pot, temperature monitor). Adjusts commands accordingly (PID if autonomous, power limiting, etc.).
* `pidcontroller.cpp`
  - PID implementation.
* `sensors.cpp`
  - Where input-device classes live. Includes Throttle, Thermistor, SpeedSensor, AngleSensor


### Some Standards (whatever those are...)

* classes are capital camel-case `SomeCoolClass`
* functions are lower camel-case `someGreatFunction()` (including member functions)
* variables are underscored `a_useful_var`
* private member variables have preceeding underscore `_do_not_touch`
* constants are #defined and capitalized `#define CRITICAL_VALUE`
* pin names are preceeded with "p_" `p_sensor_input`
* no bit shifting for math: use `var = inp * 256` NOT ~~`var = inp << 8`~~ (because it's gross)
* accompany register setting with a comment describing outcome and platform `REGA1 |= val //E makes clock INFINITELY FAST (ATmega328)`

x	