/// main.cpp
/// Runs arduino setup and loop

#include <Arduino.h>
#include <scheduler.h>
#include <Wire.h>
#include "defines.h"

void setup()
{
	Serial.begin(serialBaudrate);
    Serial1.begin(bluetoothBaudrate);
#ifdef SSC32
    Serial3.begin(ssc32Baudrate);
#endif
#ifdef SD21
	Wire.begin(); // start I2C
	for (int i = 0; i <= 51; i = i + 3)
	{
		Wire.beginTransmission(sd21AddressI2C);
		Wire.write(i); //Servo1 Speed register 0
		Wire.write(0); // Set speed Servo1
		Wire.endTransmission();
	}
#endif
}

void loop()
{
	Scheduler stateCommand;

	stateCommand.run();
}
