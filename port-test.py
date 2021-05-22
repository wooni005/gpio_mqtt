#!/usr/bin/python3

import glob
import serial
import time
import signal

import settings


def signal_handler(_signal, frame):
    global exit

    print('You pressed Ctrl+C!')
    exit = True


current_sec_time = lambda: int(round(time.time()))
exit = False

# All available serial ports
ports = glob.glob('/dev/tty[A-Za-z]*')

# Specific type serial ports
portsACM = glob.glob('/dev/ttyACM*')

# Init signal handler, because otherwise Ctrl-C does not work
signal.signal(signal.SIGINT, signal_handler)

openPorts = {}

for port in portsACM:
    try:
        ser = serial.Serial(port=port,
                            baudrate=settings.SERIAL_PORT_BAUDRATE,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS,
                            timeout=1)  # 1=1sec 0=non-blocking None=Blocked

        if ser.isOpen():
            print(("Connected to serial port %s, now testing which board is connected:" % (port)))
            found = False
            timeoutTimer = current_sec_time()
            while not found:
                ser.write("v".encode())
                serInLine = ser.readline().decode()
                if serInLine != "":
                    serInLine = serInLine.rstrip("\r\n")
                    # print('GPIO: board: %d: %s' % (boardId, serInLine))
                    msg = serInLine.split(' ')
                    # print(msg)
                    if msg[0][0] == '[':
                        boardName = msg[0]
                        if boardName[0:11] == '[STM32-GPIO':
                            openPorts[boardName] = ser
                            print(" - GPIO board %s found on device %s" % (boardName, ser.name))
                            found = True
                    if exit or (current_sec_time() - timeoutTimer) > 5:
                        break
            if not found:
                print(' - Not the correct board found: %s' % boardName)

    # Handle other exceptions and print the error
    except Exception:
        print("Unable to open port %s, trying next one" % port)

for board in openPorts:
    print("Closed port %s for board %s" % (openPorts[board].name, board))
    if openPorts[board] is not None:
        openPorts[board].close()
