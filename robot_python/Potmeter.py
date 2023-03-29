import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn
import time
import RPi.GPIO as GPIO

GPIO.cleanup()

startTime = time.time()

# create the spi bus 
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI) 
# create the cs (chip select) 
cs = digitalio.DigitalInOut(board.D22) 
# create the mcp object 
mcp = MCP.MCP3008(spi, cs)
# create an analog input channel on pin 0
chan1 = AnalogIn(mcp, MCP.P6)

while time.time() - startTime < 10:
    print(round(chan1.voltage/3.3*270))
    time.sleep(0.1)