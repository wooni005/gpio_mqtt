#!/usr/bin/python

MQTT_ServerIP        = "192.168.5.248"
MQTT_ServerPort      = 1883
SERIAL_PORT_DEVICE_1 = '/dev/ttyACM0'
SERIAL_PORT_DEVICE_2 = '/dev/ttyACM1'
SERIAL_PORT_DEVICE_3 = '/dev/ttyACM2'

#SERIAL_PORT_DEVICE_2 = ''
SERIAL_PORT_BAUDRATE = 57600
LOG_FILENAME = "/home/pi/log/gpio_mqtt.log"
#LOG_FILENAME = "gpio_mqtt.log"

DB_FILENAME = "/home/pi/db/doma.db"

MQTT_TOPIC_OUT       = 'huis/GPIO/+/out'
MQTT_TOPIC_COMMAND   = 'huis/GPIO/+/command'
MQTT_TOPIC_CHECK     = "huis/GPIO/RPiHome/check"
MQTT_TOPIC_REPORT    = "huis/GPIO/RPiHome/report"
