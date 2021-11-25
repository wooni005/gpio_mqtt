#!/usr/bin/python3

MQTT_ServerIP        = "192.168.5.248"
MQTT_ServerPort      = 1883

# On these ports the right boards will be attached
SERIAL_PORT_DEVICE_BASE = '/dev/ttyACM*'
# SERIAL_PORT_DEVICE_1 = '/dev/ttyACM0'
# SERIAL_PORT_DEVICE_2 = '/dev/ttyACM1'
# SERIAL_PORT_DEVICE_3 = '/dev/ttyACM2'

BOARD_1_NAME = '[STM32-GPIO.1]'
BOARD_2_NAME = '[STM32-GPIO.2]'
BOARD_3_NAME = '[STM32-GPIO.3]'
NR_OF_BOARDS = 3

#SERIAL_PORT_BAUDRATE = 57600
SERIAL_PORT_BAUDRATE = 9600
LOG_FILENAME = "/home/pi/log/gpio_mqtt.log"
#LOG_FILENAME = "gpio_mqtt.log"

DB_FILENAME = "/home/pi/db/doma.db"

MQTT_TOPIC_OUT       = 'huis/GPIO/+/out'
MQTT_TOPIC_COMMAND   = 'huis/GPIO/+/command'
MQTT_TOPIC_CHECK     = "huis/GPIO/RPiIO/check"
MQTT_TOPIC_REPORT    = "huis/GPIO/RPiIO/report"
