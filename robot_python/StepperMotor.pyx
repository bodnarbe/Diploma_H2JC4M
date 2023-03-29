from adafruit_motorkit import MotorKit
from adafruit_motor import stepper
import time

kit = MotorKit(address=0x61)

cdef int i
cdef double time1

for i in range(150):

    kit.stepper2.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)

time1 = time.time()
kit.stepper2.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)

print(time.time() - time1)

kit.stepper2.release()    