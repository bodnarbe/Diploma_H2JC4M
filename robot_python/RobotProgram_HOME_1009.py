############  Megjegyzések ##########################
#
#Az előre irányuló mozgások a követkőképpen valósulnak meg:
#       kit(address=0x61) -> stepper 2   ->  BACKWARD
#       kit(address=0x60) -> stepper 1   ->  BACKWARD
#                        -> stepper 2   ->  FORWARD
#A forgatómű óramutató járásával ellentétesen forgat (felülről nézve), ha: kit(address=0x61) -> stepper 1 -> BACKWARD
#
#
#A LED színkód sorrendje: PIROS, KÉK, ZÖLD   (az importált modul megnevezése hibás)
#
#####################################################


#######################################################################################################################################################################
##############################################                   Modulok importálása                                 ##################################################
#######################################################################################################################################################################


import time, busio, digitalio, board
import RPi.GPIO as GPIO

import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

from adafruit_motorkit import MotorKit
from adafruit_motor import stepper
from adafruit_servokit import ServoKit
import neopixel


#######################################################################################################################################################################
##############################################                   Változók definiálása                                ##################################################
#######################################################################################################################################################################

##### Szabályozást befolyásoló konstansok (PID) (a csuklóknak megfelelő sorszámozással)  ########

P0 = 1
I0 = 0
D0 = 0

P1 = 1
I1 = 0
D1 = 0

P2 = 1
I2 = 0
D2 = 0




##### Végálláskapcsoló PIN számok ########

lws1 = 16
lws2 = 24

##### Nyomógomb PIN szám ######

nyg = 23


##### Idők ########

HomeTime = time.time()
HomeStepTime = 0.5
HomeLEDTime = time.time()
HomeLEDStepTime = 0.1

TimeStepJ1 = 0   #Csukló lépésének ideje
TimeStepJ2 = 0

StepSizeGen = 0.5

#### LED Home futófény szín Lista  ####

HomeLedColour = [255, 100, 0, 0, 0]
HomeLedColourFill = HomeLedColour
pixelSzamLista = [0, 1, 2, 3, 4]


######    Mechanikai jellemzők  #####

i1 = 16   #Áttételek
i2 = 4

rA2 = 189.9   #Robot karhosszak
rA3 = 169.2

rJ10 = 90     #Home csuklószög
rJ20 = 90

cA2 = 259   #Vezérlő karhosszak
cA3 = 215


####  Pozíciók #####

Cur1 = rJ10
Cur2 = rJ20
Home = False

#######################################################################################################################################################################
##############################################                            SETUP                                      ##################################################
#######################################################################################################################################################################

GPIO.setmode(GPIO.BCM)
GPIO.setup(lws1, GPIO.IN)
GPIO.setup(lws2, GPIO.IN)
GPIO.setup(nyg, GPIO.IN)

kit1 = MotorKit()
kit2 = MotorKit(address=0x61)

pixels = neopixel.NeoPixel(board.D18, 5, pixel_order=neopixel.GRB)


#######################################################################################################################################################################
##############################################                          HOME-olás                                    ##################################################
#######################################################################################################################################################################

#######    Első csukló     ####

kit2.stepper2.onestep(direction = stepper.BACKWARD, style = stepper.DOUBLE)

while not GPIO.input(lws1):
    if time.time() - HomeTime > HomeStepTime/(1.25*i1):
        kit1.stepper1.onestep(direction = stepper.FORWARD, style = stepper.DOUBLE)
        kit1.stepper2.onestep(direction = stepper.BACKWARD, style = stepper.DOUBLE)
        HomeTime = time.time()

    #### Futófény effektus  #####
    if time.time() - HomeLEDTime > HomeLEDStepTime:
        pixels[0] = (0, HomeLedColour[0], 0)
        pixels[1] = (0, HomeLedColour[1], 0)
        pixels[2] = (0, HomeLedColour[2], 0)
        pixels[3] = (0, HomeLedColour[3], 0)
        pixels[4] = (0, HomeLedColour[4], 0)
        HomeLedColour.append(HomeLedColour.pop(0))
        pixelSzamLista.append(pixelSzamLista.pop(0))
        HomeLEDTime = time.time()

HomeLedColourFill = HomeLedColour

for i in range(5):
    pixels[pixelSzamLista[-i]] = (0, 255, 0)
    time.sleep(0.1)

time.sleep(1)
pixels.fill((0, 0, 0))

#####  Második csukló   ####

while not GPIO.input(lws2):
    if time.time() - HomeTime > HomeStepTime/(4*i2):
        kit2.stepper2.onestep(direction = stepper.FORWARD, style = stepper.DOUBLE)
        HomeTime = time.time()

    #### Futófény effektus  #####
    if time.time() - HomeLEDTime > HomeLEDStepTime:
        pixels[0] = (0, HomeLedColour[0], 0)
        pixels[1] = (0, HomeLedColour[1], 0)
        pixels[2] = (0, HomeLedColour[2], 0)
        pixels[3] = (0, HomeLedColour[3], 0)
        pixels[4] = (0, HomeLedColour[4], 0)
        HomeLedColour.append(HomeLedColour.pop(0))
        pixelSzamLista.append(pixelSzamLista.pop(0))
        HomeLEDTime = time.time()

HomeLedColourFill = HomeLedColour

for i in range(5):
    pixels[pixelSzamLista[-i]] = (0, 255, 0)
    time.sleep(0.1)

time.sleep(1)
pixels.fill((0, 0, 0))

####  HOME pozícióba állás ####

TimeStepJ1 = time.time()
TimeStepJ2 = time.time()

while not Home:

    if time.time() - TimeStepJ1 > StepSizeGen/(2*i1) and Cur1 > 60:
        kit1.stepper1.onestep(direction = stepper.BACKWARD, style = stepper.DOUBLE)
        kit1.stepper2.onestep(direction = stepper.FORWARD, style = stepper.DOUBLE)
        TimeStepJ1 = time.time()
        Cur1 -= 0.9/i1

    if time.time() - TimeStepJ2 > StepSizeGen/(4*i2) and Cur2 > 30:
        kit2.stepper2.onestep(direction = stepper.BACKWARD, style = stepper.DOUBLE)
        TimeStepJ2 = time.time()
        Cur2 -= 0.9/i2

    if Cur1 <= 60 and Cur2 <= 30:
        Home = True

    #### Futófény effektus  #####
    if time.time() - HomeLEDTime > HomeLEDStepTime:
        pixels[0] = (0, HomeLedColour[0], 0)
        pixels[1] = (0, HomeLedColour[1], 0)
        pixels[2] = (0, HomeLedColour[2], 0)
        pixels[3] = (0, HomeLedColour[3], 0)
        pixels[4] = (0, HomeLedColour[4], 0)
        HomeLedColour.append(HomeLedColour.pop(0))
        pixelSzamLista.append(pixelSzamLista.pop(0))
        HomeLEDTime = time.time()

HomeLedColourFill = HomeLedColour

for i in range(5):
    pixels[pixelSzamLista[-i]] = (0, 255, 0)
    time.sleep(0.1)

time.sleep(1)
for i in range(5):
    pixels[pixelSzamLista[-i]] = (0, 0, 255)
    time.sleep(0.1)

time.sleep(2)


kit1.stepper1.release()
kit1.stepper2.release()
kit2.stepper2.release()




