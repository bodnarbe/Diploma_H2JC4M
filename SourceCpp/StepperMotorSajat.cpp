#include "Adafruit_MotorHAT.h"
#include "stdio.h"
#include <iostream>
#include <signal.h>

Adafruit_MotorHAT hat = Adafruit_MotorHAT(0x61);

void ctrl_c_handler(int s){
  std::cout << "Caught signal " << s << std::endl;
  hat.resetAll();
  exit(1); 
}

int main(int argc, char** argv) {
  signal(SIGINT, ctrl_c_handler);

  auto& myStepper = hat.getStepper(2); // # motor port #1
  std::cout << "Got stepper" << std::endl;
  myStepper.setSpeed(30); //  # 30 RPM

  while (true) {
    
    std::cout << "Valoban mas" << std::endl;
    std::cout << "Double coil steps" << std::endl;
    myStepper.step(100, BACKWARD,  DOUBLE);
    myStepper.step(100, FORWARD, DOUBLE);

  }
}
