from adafruit_motorkit import MotorKit
from adafruit_motor import stepper

kit1 = MotorKit()
kit2 = MotorKit(address=0x61)

kit1.stepper1.release()
kit1.stepper2.release()
kit2.stepper1.release()
kit2.stepper2.release()