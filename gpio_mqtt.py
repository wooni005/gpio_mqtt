#!/usr/bin/python3

import sys
import os
import signal
import time
# from timeit import default_timer as timer
import serial
import _thread
import traceback
import json
import glob
from queue import Queue
import paho.mqtt.publish as mqtt_publish
import paho.mqtt.client as mqtt_client

# external files/classes
import logger
import serviceReport
import settings
import db

oldInputPinStatus = []
gpioTopics = {}
pinStatus = {}

exit = False
pinsEnabled = False
allPinsSendReady = False
sendQueueBoard1 = Queue(maxsize=0)
sendQueueBoard2 = Queue(maxsize=0)
sendQueueBoard3 = Queue(maxsize=0)
current_sec_time = lambda: int(round(time.time()))
openPorts = {}


def signal_handler(_signal, frame):
    global exit

    print('You pressed Ctrl+C!')
    exit = True


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("%s: MQTT Client connected successfully" % (time.ctime(time.time())))
        client.subscribe([(settings.MQTT_TOPIC_OUT, 1), (settings.MQTT_TOPIC_COMMAND, 1), (settings.MQTT_TOPIC_CHECK, 1)])
    else:
        print("%s: ERROR: MQTT Client connected with result code %s " % (time.ctime(time.time()), str(rc)))


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msgJson):
    print(('WARNING: MQTT bericht op on_message: ' + msgJson.topic + " " + str(msgJson.payload)))


def on_message_command(client, userdata, msg):
    global allPinsSendReady
    print(("on_message_command:" + msg.topic + " " + str(msg.payload)))
    print(pinStatus)
    # pinStatus is already set by get all pins 'a' command
    mqtt_publish.single("huis/GPIO/Get-All-Pins/gpio", json.dumps(pinStatus), hostname=settings.MQTT_ServerIP)
    allPinsSendReady = True


def getOutputIndex(topic, status):
    # Check if the deviceIdx is stored in the array: outputDeviceIDX
    # print("msg.topic" + topic)
    outputIndex = None
    for pinIndex, pinTopic in gpioTopics.items():
        if pinTopic == topic:
            outputIndex = pinIndex
            if outputIndex >= 24:
                # From boardId 2: reduce index
                outputIndex -= 24
            break
    if outputIndex:
        pass
        # print("pinIndex: " + str(pinIndex) + " :" + str(status))
    return outputIndex


def setOutputPin(pinNr, val):
    if val != 0:
        sendQueueBoard2.put("1,%do".encode() % pinNr)
    else:
        sendQueueBoard2.put("0,%do".encode() % pinNr)


def setDimmingOutput(phase, pulseOn, pulseOff):
    #12,3,2R
    sendQueueBoard3.put("%d,%d,%dr".encode() % (pulseOn, pulseOff, phase))


def on_message_output(client, userdata, msg):
    global pinsEnabled
    global gpioTopics
    global pinStatus

    try:
        if pinsEnabled:
            topic = msg.topic
            deviceName = topic.split('/')[2]
            if deviceName[0:10] == 'Boiler-SSR': # Topic name
                msgData = json.loads(msg.payload.decode())
                phase = int(deviceName[-1])
                pulseOn = int(msgData['pulseOn'])
                pulseOff = int(msgData['pulseOff'])
                # print("Boiler-SSR: phase %d on: %d off: %d" % (phase, pulseOn, pulseOff))
                setDimmingOutput(phase, pulseOn, pulseOff)
            else:
                # Output pin
                status = int(msg.payload)
                outputIndex = getOutputIndex(topic, status)
                if not outputIndex:
                    print(("ERROR: OutputPin Topic not found: " + topic))
                    return
                else:
                    pinStatus[topic] = status
                    if int(status) != 0:
                        #print("Pin High")
                        setOutputPin(outputIndex, 1)
                    else:
                        #print("Pin Low")
                        setOutputPin(outputIndex, 0)
                    # print(("on_message_output: " + topic + " " + str(status)))

    except Exception as e:
        print("on_message_output: er gaat iets fout, reden: %s" % str(e))
        traceback.print_exc()
        return


def getTopic(boardId, pinIndex):
    if boardId == 1:
        inputIndex = pinIndex
    else: # boardId: 2
        inputIndex = pinIndex + 24

    if inputIndex in gpioTopics:
        return gpioTopics[inputIndex]
    else:
        return None


def processGPIOinput(boardId, pinIndex, pinName, value):
    topic = getTopic(boardId, pinIndex)

    # Check if the pin is existing in gpioTopics
    if topic:
        pinStatus[topic] = value
        # Pins are disable when starting up the daemon, enabled after 2 sec
        if pinsEnabled:
            if value == 0:
                # Skip off MQTT message for pir and pirb
                #topc = topic.split('/')
                #if (topc[3] != 'pir') and (topc[3] != 'pirb'):
                mqtt_publish.single(topic, value, qos=1, hostname=settings.MQTT_ServerIP)
            else:
                mqtt_publish.single(topic, value, qos=1, hostname=settings.MQTT_ServerIP)

            #print(str(inputTopics[pinIndex]).encode('ascii', 'ignore')+str(status))
    else:
        print("processGPIOinput: Unused pin! (boardId:%d, pinIndex:%d, pinName: %s)" % (boardId, pinIndex, pinName))


def processBMP085(pressureAtStation, temp):
    altituteInCm = 50000  # 500 m in cm
    pressureSealevel = pressureAtStation / pow((1.0 - float(altituteInCm) / float(4433000)), 5.255)

    sensorData = {}
    sensorData['Temperature'] = temp
    mqtt_publish.single("huis/GPIO/Temp-Technische-ruimte/temp", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)

    sensorData = {}
    sensorData['Sealevel'] = round(pressureSealevel / 100.0, 2)
    sensorData['AtStation'] = round(pressureAtStation / 100.0, 2)
    mqtt_publish.single("huis/GPIO/Luchtdruk/luchtdruk", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)

    # print("Barometer: %d mbar" % sensorData['Sealevel'])
    # print("Temperature: %.2f C" % temp)


def openSerialPorts():
    global exit
    global openPorts

    # Get a list of to be opened serial ports
    ports = glob.glob(settings.SERIAL_PORT_DEVICE_BASE)

    for port in ports:
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

    nrOfFoundPorts = len(openPorts)
    if nrOfFoundPorts != settings.NR_OF_BOARDS:
        #Report failure to Home Logic system check
        serviceReport.sendFailureToHomeLogic(serviceReport.ACTION_RESTART, 'Not enough GPIO ports found. Found %d and need %d serial ports' % (nrOfFoundPorts, settings.NR_OF_BOARDS))
        # # Suppress restart loops from systemd if something is wrong
        time.sleep(780) # 13 min
        exit = True


def serialPortThread(boardName, dummy):
    global exit
    global checkMsg
    global somethingWrong
    global openPorts

    # Wait a while, the OS is probably testing what kind of device is there
    # with sending 'ATEE' commands and others
    time.sleep(2)
    serialPort = openPorts[boardName]
    serialPort.reset_input_buffer()

    # Ask for board Id
    serialPort.write("v".encode())  # Get BoardName and nodeId
    boardNameOk = False
    toggleLed = False
    boardId = 0
    print("serialPortThread started for %s" % boardName)

    while not exit:
        try:
            if serialPort.isOpen():
                serInLine = serialPort.readline().decode()
            else:
                serInLine = ""
                time.sleep(5)
                print("Serial Port not open")

            if serInLine != "":
                serInLine = serInLine.rstrip("\r\n")
                # print('GPIO: board: %d: %s' % (boardId, serInLine))
                msg = serInLine.split(' ')
                #print(msg)
                # Board is started: Init it
                if not boardNameOk:
                    # [ESP-GPIO.%d]
                    boardName = msg[0]
                    if boardName[0:11] == '[STM32-GPIO':
                        if boardName[-2] == '1':
                            boardId = 1
                            boardNameOk = True
                            serialPort.write("0u".encode()) # Disable all pullups on inputs and set all inputs back to input
                            serialPort.write("a".encode())  # Get all pin statusses
                        elif boardName[-2] == '2':
                            boardId = 2
                            boardNameOk = True
                            serialPort.write("0u".encode()) # Disable all pullups on inputs and set all inputs back to input
                            serialPort.write("a".encode())  # Get all pin statusses
                        elif boardName[-2] == '3':
                            boardId = 3
                            boardNameOk = True
                            # Enable pullups, because most of the pins are not used
                            serialPort.write("1u".encode()) # Enable all pullups on inputs and set all inputs back to input
                        if boardNameOk:
                            print("Adapter OK! Init Board: %s" % boardName)
                else:
                    if msg[0] == boardName:
                        pass
                        #print("Adapter OK! %s is alive" % boardName)

                # Only handle messages starting with 'OK' from JeeLink
                # Input msg:  OK I 10 26 1 (OK I <pinIndex> <pinName> <value>)
                # Output msg: OK O 10 26 1 (OK O <pinIndex> <pinName> <value>)
                if boardNameOk and (msg[0] == 'OK'):
                    # Toggle the led with every I/O change
                    if toggleLed:
                        serialPort.write("1l".encode())  # Led on
                    else:
                        serialPort.write("0l".encode())  # Led off
                    toggleLed = not toggleLed

                    # print 'OK found!'
                    del msg[0]  # remove 'OK' from list

                    if msg[0] == 'I': #Input
                        # OK I 19 10 1
                        pinIndex    = int(msg[1]) # pinIndex
                        pinName     = msg[2]      # pinName
                        value       = int(msg[3]) # value
                        processGPIOinput(boardId, pinIndex, pinName, value)
                        # if value != 0:
                        #     print("Input: boardId: %d, pinIndex=%d, value=%d (%s)" % (boardId, pinIndex, value, getTopic(boardId, pinIndex)))

                    # elif msg[0] == 'O': #Output
                    #     # OK O 13 24 1
                    #     pinIndex    = int(msg[1]) # pinIndex
                    #     pinName    = int(msg[2]) # pinName
                    #     value       = int(msg[3]) # value

                        # print("Output: boardId: %d, pinIndex=%d, value=%d" % (boardId, pinIndex, value))
                    elif msg[0] == 'P': #Barometer
                        # OK 96641 20.90
                        processBMP085(int(msg[1]), float(msg[2]))

                    # elif msg[0] == 'R': #Relais/SSR for dimming
                    #     # OK R 1 20 30
                    #     phaseNr    = int(msg[1]) # phaseNr
                    #     pulseOn    = int(msg[2]) # pulseOn  [10ms]
                    #     pulseOff   = int(msg[3]) # pulseOff [10ms]

            # Reset Rx timers and check if there is any message to send
            if boardId == 1:
                # Reset the Rx timeout timer
                serviceReport.systemWatchTimerBoard1 = current_sec_time()

                if not sendQueueBoard1.empty():
                    sendMsg = sendQueueBoard1.get_nowait()
                    # print(("SendMsg: %s" % sendMsg))
                    if sendMsg != "":
                        serialPort.write(sendMsg)
            elif boardId == 2:
                # Reset the Rx timeout timer
                serviceReport.systemWatchTimerBoard2 = current_sec_time()

                if not sendQueueBoard2.empty():
                    sendMsg = sendQueueBoard2.get_nowait()
                    # print(("SendMsg: %s" % sendMsg))
                    if sendMsg != "":
                        serialPort.write(sendMsg)
            elif boardId == 3:
                # Reset the Rx timeout timer
                serviceReport.systemWatchTimerBoard3 = current_sec_time()

                if not sendQueueBoard3.empty():
                    sendMsg = sendQueueBoard3.get_nowait()
                    # print(("SendMsg: %s" % sendMsg))
                    if sendMsg != "":
                        serialPort.write(sendMsg)

        # In case the message contains unusual data
        except ValueError as arg:
            print(arg)
            traceback.print_exc()
            time.sleep(1)

        # Quit the program by Ctrl-C
        except KeyboardInterrupt:
            print("Program aborted by Ctrl-C")
            exit()

        # Handle other exceptions and print the error
        except Exception as arg:
            print("Exception in serialPortThread boardName %s on serialPort: %s" % (boardName, openPorts[boardName].name))
            print("%s" % str(arg))
            traceback.print_exc()
            time.sleep(120)


###
# Initalisation ####
###

# Check if the script runs with sudo/root rights,
# otherwise exec sudo with this script. GPIO needs root rights
euid = os.geteuid()

if euid != 0:
    print("Script not started as root. Running sudo..")
    args = ['sudo', sys.executable] + sys.argv + [os.environ]
    # the next line replaces the currently-running process with the sudo
    os.execlpe('sudo', *args)

logger.initLogger(settings.LOG_FILENAME)

# Init signal handler, because otherwise Ctrl-C does not work
signal.signal(signal.SIGINT, signal_handler)

# Make the following devices accessable for user
os.system("sudo chmod 666 %s" % settings.SERIAL_PORT_DEVICE_BASE)

# Give Home Assistant and Mosquitto the time to startup
time.sleep(1)

# Get MQTT Topics for input pins
cur = db.openDB()
cur.execute("SELECT * FROM GPIOpins ORDER BY pin ASC")
gpioTopicsTabel = cur.fetchall()

for pinNr, topic in gpioTopicsTabel:
    gpioTopics[pinNr] = topic
    pinStatus[topic] = 0 # Only create, value will be set leater
    print("pinNr: %d, Topic: %s" % (pinNr, topic))

# First start the MQTT client
client = mqtt_client.Client()
client.message_callback_add(settings.MQTT_TOPIC_OUT,     on_message_output)
client.message_callback_add(settings.MQTT_TOPIC_COMMAND, on_message_command)
client.message_callback_add(settings.MQTT_TOPIC_CHECK,   serviceReport.on_message_check)
client.on_connect = on_connect
client.on_message = on_message
client.connect("192.168.5.248", 1883, 60)
client.loop_start()

# Search for the correct ports and open the ports
openSerialPorts()

# Create the serialPortThread for board 1
try:
    # thread.start_new_thread( print_time, (60, ) )
    _thread.start_new_thread(serialPortThread, (settings.BOARD_1_NAME, 0))
except Exception as e:
    print("Error: unable to start the serialPortThread for board %s" % settings.BOARD_1_NAME)
    print("Exception: %s" % str(e))
    traceback.print_exc()

# Create the serialPortThread for board 2
try:
    # thread.start_new_thread( print_time, (60, ) )
    _thread.start_new_thread(serialPortThread, (settings.BOARD_2_NAME, 0))
except Exception as e:
    print("Error: unable to start the serialPortThread for board %s" % settings.BOARD_2_NAME)
    print("Exception: %s" % str(e))
    traceback.print_exc()

# Create the serialPortThread for board 3
try:
    # thread.start_new_thread( print_time, (60, ) )
    _thread.start_new_thread(serialPortThread, (settings.BOARD_3_NAME, 0))
except Exception as e:
    print("Error: unable to start the serialPortThread for board %s" % settings.BOARD_3_NAME)
    print("Exception: %s" % str(e))
    traceback.print_exc()

time.sleep(10)

readSensorTimer = 170  # Hold the pinsEnabled for 10 sec

while not exit:
    readSensorTimer = readSensorTimer + 1

    # Read and send the sensor data every 180s=3min
    if (readSensorTimer >= 180) or allPinsSendReady:
        readSensorTimer = 0

        allPinsSendReady = False # Enable pinsEnabled directly when all pins are send
        if pinsEnabled is False:
            pinsEnabled = True
            sendQueueBoard1.put("a".encode()) # Get board name and id 1 as watchdog for the serial adapter
            sendQueueBoard2.put("a".encode()) # Get board name and id 2 as watchdog for the serial adapter
            print("Pins are enabled")

        sendQueueBoard1.put("v".encode()) # Get board name and id 1 as watchdog for the serial adapter
        sendQueueBoard2.put("v".encode()) # Get board name and id 2 as watchdog for the serial adapter
        sendQueueBoard2.put("p".encode()) # Get barometer and temperature from board 2
        sendQueueBoard3.put("v".encode()) # Get board name and id 3 as watchdog for the serial adapter

    time.sleep(1)

for board in openPorts:
    print("Closed port %s for board %s" % (openPorts[board].name, board))
    if openPorts[board] is not None:
        openPorts[board].close()

print("Clean exit!")
