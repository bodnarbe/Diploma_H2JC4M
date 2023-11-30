#! /usr/bin/env python3

import time
import serial
import sys

def usage():
    return "%s [serial_port (e.g. ttyACM0)]"%sys.argv[0]

if __name__ == "__main__":

    if len(sys.argv) == 2:
        port = sys.argv[1]

    else:
        print(usage())
        sys.exit(1)

    # configure the serial connections (the parameters differs on the device you are connecting to)
    ser = serial.Serial(
        port='/dev/' + port,
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS
    )

    ser.close()
    ser.open()
    ser.isOpen()

    print('Enter your commands below.\r\nInsert "exit" to leave the application.')

    inp=1
    while 1 :
        # get keyboard input
        inp = input(">> ")
            # Python 3 users
            # input = input(">> ")
        if inp == 'exit':
            ser.close()
            exit()

        else:
            # send the character to the device
            # (note that I happend a \r\n carriage return and line feed to the characters - this is requested by my device)
            ser.write(b"%s\r\n" % inp.encode('ascii','ignore'))
            out = b''
            # let's wait one second before reading output (let's give device time to answer)
            time.sleep(0.1)
            while ser.inWaiting() > 0:
                out += ser.read(1)

            if out != '':
                print(">> %s" % out)