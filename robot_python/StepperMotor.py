from adafruit_motorkit import MotorKit
from adafruit_motor import stepper
import time

kit = MotorKit(address=0x61)

for i in range(1000):

    kit.stepper1.onestep(direction = stepper.FORWARD, style = stepper.DOUBLE)