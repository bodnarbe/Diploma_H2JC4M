import spidev
import time
import os
 
# Open SPI bus
spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz=200000

# Function to read SPI data from MCP3008 chip
# Channel must be an integer 0-7
def ReadAngle(channel):
  adc = spi.xfer2([1,(8+channel)<<4,0])
  data = ((adc[1]&3) << 8) + adc[2]
  angle = (data * 270) / float(1023)
  return angle
 
# Define delay between readings
delay = 0.1

StartTime = time.time()

while True:

    PrevTime = StartTime
    StartTime = time.time()   

    
    print("K1: {:.3f},   K2: {:.3f},   K3: {:.3f},   K4: {:.3f},   K5: {:.3f},   K6: {:.3f},    K7: {:.3f}".format(ReadAngle(1),ReadAngle(2),ReadAngle(3),ReadAngle(4),ReadAngle(5),ReadAngle(6),ReadAngle(7)))
    time.sleep(delay)
    # Wait before repeating loop
#     print(StartTime - PrevTime)
