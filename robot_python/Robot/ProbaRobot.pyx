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
#A potmétereknél megadott értékek kitapasztalt tartománybeli értékek
#A nyomógombok esetén a felső határ nem 270 deg, hanem 265 deg, hogy legyen egy biztonságos tartomány
#
#
#
#
#####################################################

print("Jot importal")

#######################################################################################################################################################################
##############################################                   Modulok importálása                                 ##################################################
#######################################################################################################################################################################


from time import time, sleep
import busio, digitalio, board
import RPi.GPIO as GPIO

import spidev, os

from adafruit_motorkit import MotorKit
from adafruit_motor import stepper
from adafruit_servokit import ServoKit
import neopixel

from math import pi, sin, cos, acos, asin, sqrt


#######################################################################################################################################################################
##############################################                   Változók definiálása                                ##################################################
#######################################################################################################################################################################

##### Szabályozást befolyásoló konstansok (PID) (a csuklóknak megfelelő sorszámozással)  ########

cdef double P0 = 1.0
cdef double I0 = 0.0
cdef double D0 = 0.0

cdef double P1 = 40.0
cdef double I1 = 0.0
cdef double D1 = 0.0

cdef double P2 = 10.0
cdef double I2 = 0.0
cdef double D2 = 0.0




##### Végálláskapcsoló PIN számok ########

cdef int lws1 = 16
cdef int lws2 = 24

##### Nyomógomb PIN szám ######

cdef int nyg = 23

##### Vezérlő változóinak definiálása

cdef double cJ1, cJ2, cJ3, cTalp, cProp, cForgas, cMegfogo
cJ1 = cJ2 = cJ3 = cTalp = cProp = cForgas = cMegfogo = 0.0

####   Offsetek  #####

cdef int potJ1Offset = -77
cdef int potJ2Offset = 257
cdef int potTOffset = 172
cdef int potJ3Offset = 167-90
##### Idők ########

cdef double HomeTime = time()
cdef double HomeStepTime = 0.5
cdef double HomeLEDTime = time()
cdef double HomeLEDStepTime = 0.1

cdef double TimeStepJ1 = 0.0   #Csukló lépésének ideje
cdef double TimeStepJ2 = 0.0
cdef double TimeStepJ0 = 0.0

cdef double StepSizeGen = 0.5

cdef double PrevTime, CurTime, dT1, dT2, dT0
PrevTime = CurTime = 0.0
dT1 = dT2 = dT0 = 0.0

#### LED Home futófény szín Lista  ####

HomeLedColour = [255, 100, 0, 0, 0]
HomeLedColourFill = HomeLedColour
pixelSzamLista = [0, 1, 2, 3, 4]


######    Mechanikai jellemzők  #####

cdef int i1 = 16   #Áttételek
cdef int i2 = 4
cdef double i0 = 38.4

cdef double rA1 = 189.9   #Robot karhosszak
cdef double rA2 = 169.2

cdef double rJ10 = 0.0     #Home csuklószög
cdef double rJ20 = 0.0

cdef int cA1 = 259   #Vezérlő karhosszak
cdef int cA2 = 215

####  Pozíciók #####

cdef double Cur1 = rJ10
cdef double Cur2 = rJ20
cdef double Cur0 = 0.0
cdef double Prev1, Prev2
Prev1 = Prev2 = 0.0
cdef double Err1, Err2, PErr1, PErr2, Goal1, Goal2, Err0
Err0 = Err1 = Err2 = PErr1 = PErr2 = Goal1 = Goal2 = 0.0
cdef double RGoalX, RGoalY, CCurX, CCurY, CPrevX, CPrevY
RGoalX = RGoalY = CCurX = CCurY = CPrevX = CPrevY = 0.0
cdef bint Home = 0

cdef double cDx = 0.0
cdef double cDy = 0.0

##########   Váltószámok    #########

cdef double deg = 3.14159265/180.0
cdef double rad = 1.0/deg

###### Prop szám   #####

cdef double prop = 0
cdef double propM = (1.0-0.0)/(217.0-265.0)
cdef double propK = 1.0-propM*217.0

# print(propM)
# print(propK)

###### Megfogó számok #####

cdef double megM = (120.0 - 20.0)/(265.0 - 217.0)
cdef double megK = 120.0 - megM * 265.0
cdef double megC = 0.0

# print(megM)
# print(megK)

######  Forgás számok  ####

cdef double forC = 0.0
cdef double forM = (1.0-0.0)/(217.0-265.0)
cdef double forK = 1.0-forM*217.0

print(forM)
print(forK)

#####  Orientáció számok  ###

cdef double Orient = 0.0
cdef double oriC = 0.0
cdef double oriM = (0.0-180.0)/(-57.0-78.0)
cdef double oriK = 0.0 - oriM*(-57.0)

### Számításhoz használt változók

cdef double xFuto, yFuto, a0, a1, a2, kszi, nu, omega, theta, D
xFuto = yFuto = 0.0
a0 = a1 = a2 = 0.0
kszi = nu = omega = theta = 0.0
cdef bint talalat = 0
D = 0

#### Korlátok

cdef int Limit1 = 85
cdef int Limit2 = 120
cdef int Limit3 = 0
cdef bint Limit = 0

#####  Integráló, derivált változók

cdef Int1, Int2, Der1, Der2

Int1 = Int2 = 0
Der1 = Der2 = 0

#########  Ciklus változó  ######

cdef int i = 0

#################    Függvények    ###########

def ReadAngle(channel):

  adc = spi.xfer2([1,(8+channel)<<4,0])
  data = ((adc[1]&3) << 8) + adc[2]
  angle = data/1023*270
  return angle



#######################################################################################################################################################################
##############################################                            SETUP                                      ##################################################
#######################################################################################################################################################################

# GPIO.cleanup()

GPIO.setmode(GPIO.BCM)         # Digitális ki- és bemenetek
GPIO.setup(lws1, GPIO.IN)
GPIO.setup(lws2, GPIO.IN)
GPIO.setup(nyg, GPIO.IN)
# 
kit1 = MotorKit()    #Stepper motorok
kit2 = MotorKit(address=0x61)
# 
pixels = neopixel.NeoPixel(board.D18, 5, pixel_order=neopixel.GRB)   #LED szalag
#
sKit = ServoKit(address = 0x42, channels=16)
sKit.servo[0].set_pulse_width_range(800, 2500) #Megfogo
sKit.servo[1].set_pulse_width_range(800, 2580)  #Csuklo
#sKit.servo[2].set_pulse_width_range(800, 2580) #Forgas
#


spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz=1953000

# 

#######################################################################################################################################################################
##############################################                          HOME-olás                                    ##################################################
#######################################################################################################################################################################

sKit.servo[1].angle = 79

#######    Első csukló     ####

kit2.stepper2.onestep(direction = stepper.BACKWARD, style = stepper.DOUBLE)

while not GPIO.input(lws1):
    if time() - HomeTime > HomeStepTime/(1.25*i1):
        kit1.stepper1.onestep(direction = stepper.FORWARD, style = stepper.DOUBLE)
        kit1.stepper2.onestep(direction = stepper.BACKWARD, style = stepper.DOUBLE)
        HomeTime = time()

    #### Futófény effektus  #####
    if time() - HomeLEDTime > HomeLEDStepTime:
        pixels[0] = (0, HomeLedColour[0], 0)
        pixels[1] = (0, HomeLedColour[1], 0)
        pixels[2] = (0, HomeLedColour[2], 0)
        pixels[3] = (0, HomeLedColour[3], 0)
        pixels[4] = (0, HomeLedColour[4], 0)
        HomeLedColour.append(HomeLedColour.pop(0))
        pixelSzamLista.append(pixelSzamLista.pop(0))
        HomeLEDTime = time()

HomeLedColourFill = HomeLedColour

for i in range(5):
    pixels[pixelSzamLista[-i]] = (0, 255, 0)
    sleep(0.1)

sleep(1)
pixels.fill((0, 0, 0))

#####  Második csukló   ####

while not GPIO.input(lws2):
    if time() - HomeTime > HomeStepTime/(4*i2):
        kit2.stepper2.onestep(direction = stepper.FORWARD, style = stepper.DOUBLE)
        HomeTime = time()

    #### Futófény effektus  #####
    if time() - HomeLEDTime > HomeLEDStepTime:
        pixels[0] = (0, HomeLedColour[0], 0)
        pixels[1] = (0, HomeLedColour[1], 0)
        pixels[2] = (0, HomeLedColour[2], 0)
        pixels[3] = (0, HomeLedColour[3], 0)
        pixels[4] = (0, HomeLedColour[4], 0)
        HomeLedColour.append(HomeLedColour.pop(0))
        pixelSzamLista.append(pixelSzamLista.pop(0))
        HomeLEDTime = time()

HomeLedColourFill = HomeLedColour

for i in range(5):
    pixels[pixelSzamLista[-i]] = (0, 255, 0)
    sleep(0.1)

sleep(1)
pixels.fill((0, 0, 0))

####  HOME pozícióba állás ####

TimeStepJ1 = time()
TimeStepJ2 = time()

while Home == 0:

    if time() - TimeStepJ1 > StepSizeGen/(2*i1) and Cur1 < 60:
        kit1.stepper1.onestep(direction = stepper.BACKWARD, style = stepper.DOUBLE)
        kit1.stepper2.onestep(direction = stepper.FORWARD, style = stepper.DOUBLE)
        TimeStepJ1 = time()
        Cur1 += 0.9/i1

    if time() - TimeStepJ2 > StepSizeGen/(4*i2) and Cur2 < 60:
        kit2.stepper2.onestep(direction = stepper.BACKWARD, style = stepper.DOUBLE)
        TimeStepJ2 = time()
        Cur2 += 0.9/i2

    if Cur1 >=30 and Cur2 >= 60:
        Home = True

    #### Futófény effektus  #####
    if time() - HomeLEDTime > HomeLEDStepTime:
        pixels[0] = (0, HomeLedColour[0], 0)
        pixels[1] = (0, HomeLedColour[1], 0)
        pixels[2] = (0, HomeLedColour[2], 0)
        pixels[3] = (0, HomeLedColour[3], 0)
        pixels[4] = (0, HomeLedColour[4], 0)
        HomeLedColour.append(HomeLedColour.pop(0))
        pixelSzamLista.append(pixelSzamLista.pop(0))
        HomeLEDTime = time()

HomeLedColourFill = HomeLedColour

for i in range(5):
    pixels[pixelSzamLista[-i]] = (0, 255, 0)
    sleep(0.1)

sleep(1)
for i in range(5):
    pixels[pixelSzamLista[-i]] = (0, 0, 255)
    sleep(0.1)

sleep(2)

##########  Felesleges változók törélése   #######

# del Home, HomeLedColour, HomeLedColourFill, HomeLEDStepTime, HomeLEDTime, HomeStepTime, HomeTime, pixelSzamLista
del HomeLedColour, HomeLedColourFill, pixelSzamLista




#######################################################################################################################################################################
##############################################                          Főprogram                                    ##################################################
#######################################################################################################################################################################




# Cur1 = 30
# Cur2 = 60

cJ1 = (ReadAngle(4)+potJ1Offset)*deg
cJ2 = (ReadAngle(7)*(-1)+potJ2Offset)*deg


CCurX = cA2*(cos(cJ1)*sin(cJ2)+sin(cJ1)*cos(cJ2)) + cA1*sin(cJ1)
CCurY = cA2*(cos(cJ1)*cos(cJ2)-sin(cJ1)*sin(cJ2)) + cA1*cos(cJ1)

RGoalX = rA2*(cos((Cur1)*deg)*sin((Cur2)*deg)+sin((Cur1)*deg)*cos((Cur2)*deg)) + rA1*sin((Cur1)*deg)
RGoalY = rA2*(cos((Cur1)*deg)*cos((Cur2)*deg)-sin((Cur1)*deg)*sin((Cur2)*deg)) + rA1*cos((Cur1)*deg)

CurTime = time()
sleep(0.1)

TimeStepJ1 = time()
TimeStepJ2 = time()

potTOffset = ReadAngle(2)

#print("Cs1: {},   Cs2: {}".format(Cur1, Cur2))

while True:

#     sleep(0.1)
        
    talalat = 0
    Limit = 0
    
    cJ1 = (ReadAngle(4)+potJ1Offset)*deg
    cJ2 = (ReadAngle(7)*(-1)+potJ2Offset)*deg
    cProp = ReadAngle(5)
    cMegfogo = ReadAngle(6)
    cForgas = ReadAngle(3)
    cJ3 = ReadAngle(1)
    cTalp = ReadAngle(2)
    
    CPrevX = CCurX
    CPrevY = CCurY

    Prev1 = Cur1
    Prev2 = Cur2

    PrevTime = CurTime
    CurTime = time()

    CCurX = cA2*(cos(cJ1)*sin(cJ2)+sin(cJ1)*cos(cJ2)) + cA1*sin(cJ1)
    CCurY = cA2*(cos(cJ1)*cos(cJ2)-sin(cJ1)*sin(cJ2)) + cA1*cos(cJ1)
        
    cDx = CCurX - CPrevX
    cDy = CCurY - CPrevY   ####  Vezérlő elmozdulása   ###
    
#     print("cDx: {},   cDy: {}".format(cDx, cDy))
#     print("Prop: {}".format(prop))
#     print("cProp: {}".format(cProp))
#     print("RGoalX: {},   RGoalY: {}".format(RGoalX, RGoalY))
    
#     prop = 1
    
    ####  Arányosság meghatározása ####
    if cProp>265:
        prop = 0
    else:
        prop = propM * cProp + propK

#     RGoalX = 200
#     RGoalY = 200

    RGoalX += cDx*prop
    RGoalY += cDy*prop

    ######## Szögek meghatározása #########

    a0 = ((rA2*rA2 - rA1*rA1 - RGoalX*RGoalX - RGoalY*RGoalY)*(rA2*rA2 - rA1*rA1 - RGoalX*RGoalX - RGoalY*RGoalY))/(4*RGoalY*RGoalY) - rA1*rA1
    a1 = (rA2*rA2 * RGoalX - rA1*rA1 * RGoalX - RGoalX*RGoalX*RGoalX - RGoalX*RGoalY*RGoalY)/(RGoalY*RGoalY)
    a2 = (RGoalX/RGoalY)*(RGoalX/RGoalY) + 1

    ###########################################    Gyökkeresés        ##########################
#     print(RGoalX,"/n", RGoalY)

    try:
    
        D = sqrt(a1 * a1 - 4 * a0 * a2)
        
    except ValueError:
        
        RGoalX -= cDx*prop
        RGoalY -= cDy*prop
        
        a0 = ((rA2*rA2 - rA1*rA1 - RGoalX*RGoalX - RGoalY*RGoalY)*(rA2*rA2 - rA1*rA1 - RGoalX*RGoalX - RGoalY*RGoalY))/(4*RGoalY*RGoalY) - rA1*rA1
        a1 = (rA2*rA2 * RGoalX - rA1*rA1 * RGoalX - RGoalX*RGoalX*RGoalX - RGoalX*RGoalY*RGoalY)/(RGoalY*RGoalY)
        a2 = (RGoalX/RGoalY)*(RGoalX/RGoalY) + 1
        
        D = sqrt(a1 * a1 - 4 * a0 * a2)
        
        if Cur2 < 2:
        
            pixels.fill((255, 0, 0))
            sleep(2)
            pixels.fill((0, 0, 255))   
    

    #### Első próba
    xFuto = (-a1 + D)/(2 * a2)
    yFuto = sqrt(rA1*rA1 - xFuto*xFuto)

    if abs((xFuto - RGoalX)*(xFuto - RGoalX) + (yFuto - RGoalY)*(yFuto - RGoalY) - rA2*rA2) < 0.001:

        omega = asin(xFuto/rA1)
        if yFuto < 0:
            omega = 180*deg - omega
        nu = (RGoalX - rA1 * sin(omega)) * cos(omega) - (RGoalY - rA1 * cos(omega)) * sin(omega)
        if nu >= 0:
            kszi = (RGoalX - rA1 * sin(omega)) * sin(omega) + (RGoalY - rA1 * cos(omega)) * cos(omega)
            theta = acos(kszi/sqrt(kszi**2 + nu**2))
            talalat = 1
            print("Elso")
            
    if talalat == 0: 

        yFuto = -yFuto

        if abs((xFuto - RGoalX)*(xFuto - RGoalX) + (yFuto - RGoalY)*(yFuto - RGoalY) - rA2*rA2) < 0.001:

            omega = asin(xFuto/rA1)
            if yFuto < 0:
                omega = 180*deg - omega
            nu = (RGoalX - rA1 * sin(omega)) * cos(omega) - (RGoalY - rA1 * cos(omega)) * sin(omega)
            if nu >= 0:
                kszi = (RGoalX - rA1 * sin(omega)) * sin(omega) + (RGoalY - rA1 * cos(omega)) * cos(omega)
                theta = acos(kszi/sqrt(kszi**2 + nu**2))
                talalat = 1
                print("Masodik")
          
        if talalat == 0:

            xFuto = (-a1 - D)/(2 * a2)
            yFuto = sqrt(rA1*rA1 - xFuto*xFuto)

            if abs((xFuto - RGoalX)*(xFuto - RGoalX) + (yFuto - RGoalY)*(yFuto - RGoalY) - rA2*rA2) < 0.001:

                omega = asin(xFuto/rA1)
                if yFuto < 0:
                    omega = 180*deg - omega
                nu = (RGoalX - rA1 * sin(omega)) * cos(omega) - (RGoalY - rA1 * cos(omega)) * sin(omega)
                if nu >= 0:
                    kszi = (RGoalX - rA1 * sin(omega)) * sin(omega) + (RGoalY - rA1 * cos(omega)) * cos(omega)
                    theta = acos(kszi/sqrt(kszi**2 + nu**2))
                    talalat = 1
                    print("Harmadik")
      
            if talalat == 0:
      
                yFuto = -yFuto

                if abs((xFuto - RGoalX)*(xFuto - RGoalX) + (yFuto - RGoalY)*(yFuto - RGoalY) - rA2*rA2) < 0.001:
    
                    omega = asin(xFuto/rA1)
                    if yFuto < 0:
                        omega = 180*deg - omega
                    nu = (RGoalX - rA1 * sin(omega)) * cos(omega) - (RGoalY - rA1 * cos(omega)) * sin(omega)
                    if nu >= 0:
                        kszi = (RGoalX - rA1 * sin(omega)) * sin(omega) + (RGoalY - rA1 * cos(omega)) * cos(omega)
                        theta = acos(kszi/sqrt(kszi**2 + nu**2))
                        talalat = 1
                        print("Negyedik")




    ##############   Célértékek megadása   ##############
    
    Goal1 = omega * rad
    Goal2 = theta * rad

#     print("Cs1: {},   Cs2: {}".format(Goal1, Goal2))

    #############     Hibák számítása    #####################

    PErr1 = Err1
    PErr2 = Err2

    Err1 = Goal1 - Cur1
    Err2 = Goal2 - Cur2
    Err0 = cTalp - Cur0 - potTOffset

    #############     Integrálás   ##############x

    Int1 += (PErr1 + Err1)/2 * (CurTime - PrevTime)
    Int2 += (PErr2 + Err2)/2 * (CurTime - PrevTime)

    #############    Deriválás    ################

    Der1 = (Err1 - PErr1)/(CurTime - PrevTime)
    Der2 = (Err2 - PErr2)/(CurTime - PrevTime)

    #############   Lépésidők meghatározása   ########
    if Err1 == 0:
        Err1 = 0.0001
    if Err2 == 0:
        Err2 = 0.0001
    if abs(Err0) < 0.1:
        Err0 = 0.0001
        kit2.stepper1.release()
    dT1 = 1/(P1 * Err1 + I1 * Int1 + D1 * Der1)
    dT2 = 1/(P2 * Err2 + I2 * Int2 + D2 * Der2)
    dT0 = 1/(P0 * Err0)

#     dT1 = 0.02 * dT1/abs(dT1)
#     dT2 = 0.02 * dT2/abs(dT2)

    ############   Motorok léptetése    #########

##    if abs(Err1) > abs(Err2):
##        dT2 = abs(dT1)*abs(Err1/Err2)*i1/i2*dT2/abs(dT2)
##
##    else:
##        dT1 = abs(dT2*Err2/Err1)*i2/i1*dT1/abs(dT1)

    if CurTime - TimeStepJ1 > abs(dT1):
        if dT1 > 0 and Cur1 < Limit1:
            kit1.stepper1.onestep(direction = stepper.BACKWARD, style = stepper.DOUBLE)
            kit1.stepper2.onestep(direction = stepper.FORWARD, style = stepper.DOUBLE)
            Cur1 += 0.9 / i1
        elif dT1 <= 0 and Cur1 > 2:
            kit1.stepper1.onestep(direction = stepper.FORWARD, style = stepper.DOUBLE)
            kit1.stepper2.onestep(direction = stepper.BACKWARD, style = stepper.DOUBLE)
            Cur1 -= 0.9 / i1
        else:
            Limit = 1
        TimeStepJ1 = CurTime

    if Limit == 0:
        if CurTime - TimeStepJ2 > abs(dT2):
            if dT2 > 0 and Cur2 < Limit2:
                kit2.stepper2.onestep(direction = stepper.BACKWARD, style = stepper.DOUBLE)
                Cur2 += 0.9 / i2
            elif dT2 <= 0 and Cur2 > 2:
                kit2.stepper2.onestep(direction = stepper.FORWARD, style = stepper.DOUBLE)
                Cur2 -= 0.9 / i2
            else:
                Limit = 1
            TimeStepJ2 = CurTime
            
        if CurTime - TimeStepJ0 > abs(dT0):
            if dT0 > 0:
                kit2.stepper1.onestep(direction = stepper.BACKWARD, style = stepper.DOUBLE)
                Cur0 += 0.9 / i0
            else:
                kit2.stepper1.onestep(direction = stepper.FORWARD, style = stepper.DOUBLE)
                Cur0 -= 0.9 / i0    
            
    if Limit == 1:
        
        pixels.fill((255, 0, 0))
        
        RGoalX = rA2*(cos((Cur1)*deg)*sin((Cur2)*deg)+sin((Cur1)*deg)*cos((Cur2)*deg)) + rA1*sin((Cur1)*deg)
        RGoalY = rA2*(cos((Cur1)*deg)*cos((Cur2)*deg)-sin((Cur1)*deg)*sin((Cur2)*deg)) + rA1*cos((Cur1)*deg)
        
        sleep(2)
        pixels.fill((0, 0, 255))         
      
    if cMegfogo > 265:
         megC = 120.0     
    else:   
        megC = megM * cMegfogo + megK
    if megC < 20:
        megC = 0
        
    if cForgas > 265:
        forC = 0
    else:
        forC = forM * cForgas + forK
#     print(forC)
    if GPIO.input(nyg):
        forC = (-1)*forC
#     print(forC)    
    
    Orient = (-1)*cJ3+77+cJ1*rad+cJ2*rad-Cur1-Cur2
    
    if Orient < -57.0:
        sKit.servo[1].angle = 0
    elif Orient < 78.0:
        sKit.servo[1].angle = oriM*Orient + oriK
    else:
        sKit.servo[1].angle = 180
    
    
    sKit.servo[0].angle = megC   
    sKit.continuous_servo[2].throttle = forC  
    
        
        
#     print(CurTime - PrevTime)
#     print("Cs1: %3.4f, Cs2: %3.4f, dT1: %3.4f, dT2: %3.4f" % (Goal1, Goal2, dT1, dT2))
#     print(cMegfogo)
#     print(megC)
    print(" ")
        
        

            
            
            
# kit1.stepper1.release()
# kit1.stepper2.release()
# kit2.stepper2.release()




