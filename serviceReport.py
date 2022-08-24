#!/usr/bin/python

import time
import paho.mqtt.publish as mqtt_publish
import json

# external files/classes
import settings

#System check
ACTION_NOTHING   = 0
ACTION_RESTART   = 1

checkMsg = 'OK'
checkFail = False
checkAction = ACTION_NOTHING
checkReport = {}


def current_sec_time():
    return int(round(time.time()))


systemWatchTimerBoard1 = current_sec_time()
systemWatchTimerBoard2 = current_sec_time()
systemWatchTimerBoard3 = current_sec_time()


#Called by mqtt
def on_message_check(client, userdata, msgJson):

    if (current_sec_time() - systemWatchTimerBoard1) > 300:
        sendCheckReportToHomeLogic(True, ACTION_RESTART, "Timeout systemWatchTimerBoard1, 5 min no reaction on watchdog 'v' message in serialPortThread1-thread loop, restart service")
    elif (current_sec_time() - systemWatchTimerBoard2) > 300:
        sendCheckReportToHomeLogic(True, ACTION_RESTART, "Timeout systemWatchTimerBoard2, 5 min no reaction on watchdog 'v' message in serialPortThread2-thread loop, restart service")
    elif (current_sec_time() - systemWatchTimerBoard3) > 300:
        sendCheckReportToHomeLogic(True, ACTION_RESTART, "Timeout systemWatchTimerBoard3, 5 min no reaction on watchdog 'v' message in serialPortThread3-thread loop, restart service")
    else:
        #print("on_message_check: " + msgJson.topic + ": " + str(msgJson.payload))
        sendCheckReportToHomeLogic(checkFail, checkAction, checkMsg)


#Send the report to the Home Logic system checker
def sendCheckReportToHomeLogic(fail, action, msg):
    global checkMsg
    global checkFail

    checkMsg = msg
    checkFail = fail
    checkReport['checkFail']   = checkFail
    checkReport['checkAction'] = checkAction
    checkReport['checkMsg']    = checkMsg
    mqtt_publish.single(settings.MQTT_TOPIC_REPORT, json.dumps(checkReport), qos=1, hostname=settings.MQTT_ServerIP)


#Don't wait for the Home Logic system checker, report it directly
def sendFailureToHomeLogic(checkAction, checkMsg):
    sendCheckReportToHomeLogic(True, checkAction, checkMsg)
