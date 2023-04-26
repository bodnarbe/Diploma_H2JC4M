#include "Adafruit_MotorHAT.h" 
#include <exception>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "PWM.h"
#include <iostream>
#include "Adafruit_MotorHAT.cpp"

class Stepper_Tester_sonaj : public Adafruit_StepperMotor
{
public:
	Stepper_Tester_sonaj(Adafruit_MotorHAT* _hat, int num, int steps);//konstruktor
	void turn_fi_degrees(int fi, Direction _direction, Style _style);
};

// Ctrl-C, Ctrl-V, biztosan letezik jobb megoldas
Stepper_Tester_sonaj::Stepper_Tester_sonaj(Adafruit_MotorHAT* _hat, int num, int steps)
{
	hat = _hat;
	revsteps = steps;
	motornum = num;
	sec_per_step = 0.1;
	steppingcounter = 0;
	currentstep = 0;

	num -= 1;

	if (num == 0) {
		PWMA = 8;
		AIN2 = 9;
		AIN1 = 10;
		PWMB = 13;
		BIN2 = 12;
		BIN1 = 11;
	}
	else if (num == 1) {
		PWMA = 2;
		AIN2 = 3;
		AIN1 = 4;
		PWMB = 7;
		BIN2 = 6;
		BIN1 = 5;
	}
	else throw "MotorHAT Stepper must be between 1 and 2 inclusive";
};
Stepper_Tester_sonaj::turn_fi_degrees(int fi, Direction _direction, Style _style) //most fok-ban van
{
	int fi_to_steps = (int)((double)fi * revsteps / 360); //jobb megoldas kell a kerekitesre
	step(fi_to_steps, _direction, _style);
};