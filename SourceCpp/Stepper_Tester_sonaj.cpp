#include "Stepper_Tester_sonaj.h"

int main
{
	Adafruit_MotorHAT shield = Adafruit_MotorHAT(/*addr, freq, i2c, i2c_bus */);
	Stepper_Tester_sonaj proba = Stepper_Tester_sonaj(&shield, /*num, steps*/);
	proba.turn_fi_degrees(/*int fi*/);
}