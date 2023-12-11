[image1]: ./docs/dual_robo_comp.png "dual-robot-kompozicio"
[image2]: ./docs/kalibrácios_allas.jpg "Telemanipulator"

## Működést bemutató videó:

<a href="https://www.youtube.com/watch?v=R7zg5lOlKYE"><img width="400" src="./docs/youtube.png"></a>

# Robotrendszerek laboratóriuma beadandó feladat

  **Készítette:**
  - **Bodnár Bence Tibor(H2JC4M)**
  - **Nagy Attila Tamás(F1IE6E)**
  
## Tartalomjegyzék
1. [Feladatkiírás](#Feladatkiírás)  
2. [Telepítés](#Telepítés)  
2.1. [Git repo lehúzása és csomag telepítése](#Git-repo-lehúzása-és-csomag-telepítése)  
2.2. [Felhasznált 3rd party repok és eszközök](#Felhasznált-3rd-party-repok-és-eszközök)  
3. [Szimuláció indítása](#Szimuláció-indítása)  
3.1. [Telemanipulátor csatlakoztatás](#Telemanipulátor-csatlakoztatás)  
3.1.2. [Windows port forward](#Windows-port-forward)  
3.2. [Gazebo indítás](#Gazebo-indatás)  
3.3. [Moveit indítás](#Moveit-indítása)  
3.4. [Commander](#Commander)  
4. [Használat](#Használat)
5. [Fejlesztési lehetőségek](#Fejlesztési-lehetőségek)  

# Feladatkiírás

A feladatunk két darab UR3-as robot mozgatása volt szimulációs környezetben. A robotoknak meghatározott munkatérben kellett mozgatni a TCP pontot, amit egy külső telemanipulátorral határoztunk meg.
Részfeladatok:
- Helyezzen 2 darab UR3 robotot gazebo környezetbe
- Hozzon létre egy kontollert szimultán vezérléshez
- Valósítsa meg a telemanipulátoros vezérlést
- Tegye elérhetővé a vezérlést billentyű gombbal végzett kiválasztással

# Telepítés

Telepítse a következő python csomagokat:
```console
pip install pynput
```
Illetve a ROS-hoz szükséges package-eket:
```console
sudo apt install linux-tools-virtual hwdata -y
sudo apt install ros-noetic-position-controller -y
sudo apt install ros-noetic-position-controllers -y
sudo apt install ros-noetic-hardware-interface -y
sudo apt install ros-noetic-gripper-action-controller -y
sudo apt install ros-noetic-moveit-kinematics -y
sudo apt install ros-noetic-gazebo-plugins -y
sudo apt install ros-noetic-gazebo-grasp -y
sudo apt install ros-noetic-moveit-core -y
```

## Git repo lehúzása és csomag telepítése

Menjen a következő mappába:
```console
cd ~/catkin_ws/src
```
*Amennyiben ez nem létezik hozzon létre egy* **catkin_ws** *-t (* `mkdir -p ~/catkin_ws/src` *) munka mappát a Ros packegek számára.*

A következő repozitorit töltse le:
```console
git clone https://github.com/bodnarbe/Robot_rendszer_beo
```

Lépjünk be a catkin munkaterületre:
```console
cd ~/catkin_ws
```

Fordítsuk le a letöltött csomagot:
```console
catkin_make
```
Valamint source-oljunk
```console
source ~/catkin_ws/devel/setup.bash
```
*Ha nem fordult le és hibát ír akkor valószínüsíthetően vagy létezik már bizonyos node vagy hiányzik valamilyen Cmake-hez köthető package.*

## Felhasznált 3rd party repok és eszközök
Röviden szeretnék kitérni és felsorolni azokat a repo-kat amelyeket felhasználtunk:
- Felhasznált robot kompozíció alapja: https://github.com/cambel/dual_ur3e
- Telemanipulátor által küldött jelek feldolgozása: https://github.com/dudasdavid/mogi_haptic_device
- Move-it commander alapja: https://github.com/MOGI-ROS/Week-11-12-Robot-arms/tree/main/ur_moveit_commander
- Telepmanipulátor: Bodnár Bence Tibor - Hat szabadságfokú telemanipulátor sebészeti eszközök számára
Minden fejlesztőnek köszönjük a támogatást :) 

# Szimuláció indítása  

Első lépésként indítsuk el a *Roscore*-t
```console
roscore
```

## Telemanipulátor csatlakoztatás  

A képenlátható eszközzel fogjuk a robotokat vezérelni
![alt text][image2]

Csatlakoztassa a telemanipulátort a számítógéphez és győzödjön megróla, hogy felismerte a rendszert az. (Amennyiben WSL-t használ hajtsa végre a port [forward](#Windows-port-forward) -olást)

Indítsuk el a haptic-device nevű node-ot
```console
roslaunch mogi_haptic_device haptic.launch 
```
Ha ez sikeresen elindult akkor a következő feliratot látjuk a terminálban legalul **[starting]**

### Windows port forward  
Ha WSL-t használ szükség van a *COM* portot forwardolni a *Windows*-tól.

Telepítse az usbipd programot: https://learn.microsoft.com/en-us/windows/wsl/connect-usb

Nyisson egy *Power Shell terminal*-t és listázza ki a csatlakoztatott usb eszközöket
```console
usbipd wsl list
```
A * STMicroelectronics Virtual COM Port (COM2)* eszköz **BUSID**-ját jegyezze fel és `-b` paraméter után írja be
```console
usbipd wsl attach -d Ubuntu-20.04 -b 1-12
```
Ezt követően ellenőrizze, hogy WSL-ben megjelent-e az eszköz. ( **ttyACM0** port )
```console
ls /dev/tty*
```
Amennyiben igen, adjunk meg minden jogosultságot a user-nek
```console
sudo chmod 777 /dev/ttyACM0  
```

## Gazebo indítás  
Szükségünk lesz egy új terminálra.

-Ha WSL-t használunk indítsuk el **két darab** X11-et és terminálba adjuk hozzá a környezeti változót indítsuk el a szimulációt.
```console
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):1
```
Ha ezt megcsináltuk indítsuk el a gazebo szimulációt
```console
roslaunch ur3e_dual_gazebo dual_ur3e_hlab.launch
```
Ha minden jól ment a következő ablakot fogjuk látni

![alt text][image1]

## Moveit indítás  
Nyissunk egy új terminált és indítsunk még egy X11-et. Adjuk hozzá az új X11-et a változókhoz.
```console
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```
A move-it controller lehetőséget fog teremteni arra, hogy irányítsuk a robotot a szimuláción belül. Ezenfelül elindít egy RVIZ-t, de ezt nem használjuk, inkább csak ellenörzésre szolgál.
```console
 roslaunch ur3e_dual_moveit_config start_sim_dual_ur3e_moveit.launch 2>&1 | grep -v TF_REPEATED_DATA   
 ```
 A sikeres indítást a RVIZ és a terminálban olvasható zöld nyugtázó üzenet jelzi.

## Commander  

A ur_commander egy általunk a célre létrehozott vezérlő a telemanipulátortól kapott szögértékekből forward kinematikával meghatározzuk a TCP-pontot és mindkét robotkarra létrehozott publisher-rel kiküldjük a kívánt TCP-pontot.

A továbbított TCP pont nem egyezik meg egy az egyben a telemanipulátoréval, mert a manipulált koordináta rendszer elhelyezkedése a *0 koordináta rendszer*-hez teljesen más mint a robotok elhelyezkedése ugyan ehhez. Röviden a telemanipulátor munkatere eltérő helyen van mint a robotké. Ezért szükség van  ` [x,y,z] ` vektorral elmozgatott koordináta transzformációra jobb és a bal robot kar esetében külön-külön.

Indítsuk el a ur_commandert.
```console
rosrun mogi_haptic_device ur_moveit_commander.py 2>&1 | grep -v TF_REPEATED_DATA 
```
Ha minden jól ment akkor semmilyen `ERROR` üznetet nem fogunk kapni. Ezt követően a két robotból álló rendszer készen áll a használatra.

# Használat

Miután sikeresen elindítottuk a szükséges node-okat (gazebo szimuláció, haptic device és moveit controller), elkezdhetjük használni az eszközt.

A vezérlő indulásakor egyik robotkart sem irányítja, ugyanis kikapcsolt állapotban indul el.
Ha be szeretnénk kapcsolni a vezérlést, akkor kattintsunk a Gazebo szimuláció ablakára, majd nyomjuk meg az `a` vagy `d` billentyűgombok egyikét.
Amennyiben az `a` gombot nyomjuk meg úgy a bal oldali robotkart, a `d` gomb segítségével pedig a jobb oldali robortkart irányíthatjuk a hardwares eszköz segítségével.
Ha le szeretnénk állítani a vezérlést, nyomjuk meg az `s` gombot, amely újra kikapcsolt állapotba helyezi az beviteli eszközt.

Természetesen a robot üzemeltetése során bármikor válthatunk az egyes robotkarok illetve a kikapcsolt állapot között.

# Fejlesztési lehetőségek
- Ros szinten fejlesztési lehetőségek lehetnek, hogy a robotokat több különböző orientációba áthelyezzük így számos kompozíciót kilehetne próbálni.
- Node szinten egy egyedi megfogót lehetne létrehozni, hogy pontosan a telemanipulátornak megfelelő mozgást lehessen szimulálni
- Telemanipulátor vezérlő szintjén pedig predikciós algoritmust lehetne fejleszteni, mivel az ipari robotok nagy százaléke a következő fel veendő ponthoz igazítja a mozgását, sebességét. Ezzel ugyan lépés hátrányba kerül a robot a telemanipulátorhoz képest, de a robot mozgása sokkal biztonságosabb lesz.
