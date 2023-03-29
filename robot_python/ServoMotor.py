import time
from adafruit_servokit import ServoKit

# Set channels to the number of servo channels on your kit.
# 8 for FeatherWing, 16 for Shield/HAT/Bonnet.
kit = ServoKit(address = 0x42, channels=16)
kit.servo[1].set_pulse_width_range(800, 2580)

kit.servo[1].angle = 180

