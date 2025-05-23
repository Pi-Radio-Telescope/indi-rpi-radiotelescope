#!/usr/bin/env python3
# coding=utf-8

"""
"""

import os, sys, time, logging, configparser
import PyIndi
import paho.mqtt.client as mqtt
import signal
import json
import argparse
import random
import string

__author__ = ''
__copyright__ = ''
__license__ = ''
__version__ = ''

# Default options
LOG_LEVEL = logging.WARNING
LIST_TOPICS = False

# INDI
INDI_HOST = 'localhost'
INDI_PORT = 7624

# MQTT
MQTT_HOST = ''
MQTT_PORT = 0
MQTT_USER = None
MQTT_PASS = None
MQTT_ROOT = ''
MQTT_POLLING = 1

# Init logging
logger = logging.getLogger('indi-mqtt')
logging.basicConfig(format='%(name)s: %(message)s', level=LOG_LEVEL, stream=sys.stdout)
logger.setLevel(LOG_LEVEL)

# if user/pass available prepare auth data
if MQTT_USER and MQTT_PASS:
	MQTT_AUTH = {'username': MQTT_USER, 'password': MQTT_PASS}
else:
	MQTT_AUTH = None

# INDI states
def strISState(s):
	if (s == PyIndi.ISS_OFF):
		return "OFF"
	else:
		return "ON"

def strIPState(s):
	if (s == PyIndi.IPS_IDLE):
		return "IDLE"
	elif (s == PyIndi.IPS_OK):
		return "OK"
	elif (s == PyIndi.IPS_BUSY):
		return "BUSY"
	elif (s == PyIndi.IPS_ALERT):
		return "ALERT"

def strDeviceType(s):
	if s & 0:
		return "GENERAL"
	elif s & (1 << 0):
		return "TELESCOPE"
	elif s & (1 << 1):
		return "CCD"
	elif s & (1 << 2):
		return "GUIDER"
	elif s & (1 << 3):
		return "FOCUSER"
	elif s & (1 << 4):
		return "FILTER"
	elif s & (1 << 5):
		return "DOME"
	elif s & (1 << 6):
		return "GPS"
	elif s & (1 << 7):
		return "WEATHER"
	elif s & (1 << 8):
		return "AO"
	elif s & (1 << 9):
		return "DUSTCAP"
	elif s & (1 << 10):
		return "LIGHTBOX"
	elif s & (1 << 11):
		return "DETECTOR"
	elif s & (1 << 12):
		return "ROTATOR"
	elif s & (1 << 13):
		return "SPECTROGRAPH"
	elif s & (1 << 15):
		return "AUX"
	else:
		return "UNKNOWN"

def shutdown():
	indiclient.disconnectServer()
	mqttclient.disconnect()
	logger.info('Exiting\nGood Bye.\n')
	sys.exit()

def term_handler(signum, frame):
        raise KeyboardInterrupt

class IndiClient(PyIndi.BaseClient):
	def __init__(self):
		super(IndiClient, self).__init__()
		logger.info('Creating an instance of INDI client')
	def newDevice(self, d):
		pass
	def newProperty(self, p):
		pass
	def removeProperty(self, p):
		pass
	def newBLOB(self, bp):
		pass
	def newSwitch(self, svp):
		pass
	def newNumber(self, nvp):
		pass
	def newText(self, tvp):
		pass
	def newLight(self, lvp):
		pass
	def newMessage(self, d, m):
		pass
	def serverConnected(self):
		logger.info("Connected to INDI server " + self.getHost() + ":" + str(self.getPort()))
	def serverDisconnected(self, code):
		logger.info("Disconnected from INDI server " + str(self.getHost()) + ":" + str(self.getPort()))

def getJSON(devices):
	observatory_json = json.loads("{}")

	for device in devices:
		device_type = strDeviceType(device.getDriverInterface())
		device_name = "_".join( device.getDeviceName().split() ).upper()
		device_name_json = json.loads("{}")
		properties = device.getProperties()
		device_properties_json = json.loads("{}")

		for property in properties:
			device_property_json = json.loads("{}")
			property_name = property.getName()
			if property.getType() == PyIndi.INDI_TEXT:
				tpy = property.getText()
				for t in tpy:
					device_property_json.update({t.name:t.text})
					device_properties_json.update({property_name:device_property_json})
			elif property.getType()==PyIndi.INDI_NUMBER:
				tpy = property.getNumber()
				for t in tpy:
					device_property_json.update({t.name:t.value})
					device_properties_json.update({property_name:device_property_json})
			elif property.getType()==PyIndi.INDI_SWITCH:
				tpy = property.getSwitch()
				for t in tpy:
					device_property_json.update({t.name:strISState(t.s)})
					device_properties_json.update({property_name:device_property_json})
			elif property.getType()==PyIndi.INDI_LIGHT:
				tpy = property.getLight()
				for t in tpy:
					device_property_json.update({t.name:strIPState(t.s)})
					device_properties_json.update({property_name:device_property_json})
			elif property.getType()==PyIndi.INDI_BLOB:
				tpy = property.getBLOB()
				for t in tpy:
					device_property_json.update({t.name:'<blob ' + str(t.size) + ' bytes>'})
					device_properties_json.update({property_name:device_property_json})

		device_name_json.update({device_name:device_properties_json})

		# Handle multiple devices of a type
		if device_type in observatory_json.keys():
			existing_device_type_json = observatory_json[device_type]
			existing_device_type_json.update(device_name_json)
		else:
			observatory_json.update({device_type:device_name_json})

	logger.debug(json.dumps(observatory_json, indent=4, sort_keys=False))

	return observatory_json

def sendMQTT(observatory_json):
	try:
		# Publish entire json in json topic e.g. observatory/json
		msg = mqttclient.publish(MQTT_ROOT.upper() + "/json", json.dumps(observatory_json))
		if msg.rc == 0:
			logger.debug("Message published to MQTT server " + MQTT_HOST + ":" + str(MQTT_PORT))
		else:
			logger.error("Error publishing to MQTT server " + MQTT_HOST + ":" + str(MQTT_PORT) + " (" + msg.rc + ")")
	except Exception as err:
		logger.error("Unexpected error publishing to MQTT server " + MQTT_HOST + ":" + str(MQTT_PORT), err)

def ClientIdMQTT(stringLength=18):
    lettersAndDigits = string.ascii_letters + string.digits
    return ''.join((random.choice(lettersAndDigits) for i in range(stringLength)))

def onConnectMQTT(client, userdata, flags, rc):
	if rc == 0:
		logger.info("Connected to MQTT server " + MQTT_HOST + ":" + str(MQTT_PORT))

		# Subscribe to external control
		mqttclient.subscribe(MQTT_ROOT.upper() + "/command")
		mqttclient.message_callback_add(MQTT_ROOT.upper() + "/command", onPollMQTT)
		logger.info("Subscribed to " + MQTT_ROOT.upper() + "/command topic at MQTT server " + MQTT_HOST + ":" + str(MQTT_PORT))

def onDisconnectMQTT(client, userdata, rc):
	if rc == 0:
		logger.info("Disconnected from MQTT server  " + MQTT_HOST + ":" + str(MQTT_PORT))
	if rc == 5:
		logger.warning("Access denied to MQTT server " + MQTT_HOST + ":" + str(MQTT_PORT) + ". Check username and password") 
	else:
		logger.warning("Connection lost to MQTT server " + MQTT_HOST + ":" + str(MQTT_PORT))

def onPollMQTT(client, userdata, message):
	global MQTT_POLLING
	msg = message.payload.decode("utf-8")
	logger.debug("message received: " + msg)
	msg = msg.split("=")
	if msg[0] == "rt_abort":
		switch = indiclient.getDevice("Pi Radiotelescope").getSwitch("TELESCOPE_ABORT_MOTION")
		switch[0].s = PyIndi.ISS_ON
		indiclient.sendNewSwitch(switch)
		switch = indiclient.getDevice("Pi Radiotelescope").getSwitch("TELESCOPE_PARK")
		switch[0].s = PyIndi.ISS_OFF
		switch[1].s = PyIndi.ISS_ON
		indiclient.sendNewSwitch(switch)
		time.sleep(1)
		switch = indiclient.getDevice("Pi Radiotelescope").getSwitch("TELESCOPE_TRACK_STATE")
		switch[0].s = PyIndi.ISS_OFF
		switch[1].s = PyIndi.ISS_ON
		indiclient.sendNewSwitch(switch)
	elif msg[0] == "rt_connection":
		weather_switch = indiclient.getDevice("Weather Watcher").getSwitch("CONNECTION")
		gpsd_switch = indiclient.getDevice("GPSD").getSwitch("CONNECTION")
		telescope_switch = indiclient.getDevice("Pi Radiotelescope").getSwitch("CONNECTION")
		if msg[1] == "0":
			weather_switch[0].s = PyIndi.ISS_OFF
			weather_switch[1].s = PyIndi.ISS_ON
			gpsd_switch[0].s = PyIndi.ISS_OFF
			gpsd_switch[1].s = PyIndi.ISS_ON
			telescope_switch[0].s = PyIndi.ISS_OFF
			telescope_switch[1].s = PyIndi.ISS_ON
		elif msg[1] == "1":
			weather_switch[0].s = PyIndi.ISS_ON
			weather_switch[1].s = PyIndi.ISS_OFF
			gpsd_switch[0].s = PyIndi.ISS_ON
			gpsd_switch[1].s = PyIndi.ISS_OFF
			telescope_switch[0].s = PyIndi.ISS_ON
			telescope_switch[1].s = PyIndi.ISS_OFF
		indiclient.sendNewSwitch(weather_switch)
		indiclient.sendNewSwitch(gpsd_switch)
		indiclient.sendNewSwitch(telescope_switch)
	elif msg[0] == "rt_park":
		switch = indiclient.getDevice("Pi Radiotelescope").getSwitch("TELESCOPE_PARK")
		if msg[1] == "0":
			switch[0].s = PyIndi.ISS_OFF
			switch[1].s = PyIndi.ISS_ON
		elif msg[1] == "1":
			switch[0].s = PyIndi.ISS_ON
			switch[1].s = PyIndi.ISS_OFF
		indiclient.sendNewSwitch(switch)
	elif msg[0] == "rt_trackstate":
		switch = indiclient.getDevice("Pi Radiotelescope").getSwitch("TELESCOPE_TRACK_STATE")
		if msg[1] == "0":
			switch[0].s = PyIndi.ISS_OFF
			switch[1].s = PyIndi.ISS_ON
		elif msg[1] == "1":
			switch[0].s = PyIndi.ISS_ON
			switch[1].s = PyIndi.ISS_OFF
		indiclient.sendNewSwitch(switch)
	elif msg[0] == "rt_gpio0":
		switch = indiclient.getDevice("Pi Radiotelescope").getSwitch("GPIO_OUTPUTS")
		if msg[1] == "0":
			switch[0].s = PyIndi.ISS_OFF
		elif msg[1] == "1":
			switch[0].s = PyIndi.ISS_ON
		indiclient.sendNewSwitch(switch)
	elif msg[0] == "rt_gpio1":
		switch = indiclient.getDevice("Pi Radiotelescope").getSwitch("GPIO_OUTPUTS")
		if msg[1] == "0":
			switch[1].s = PyIndi.ISS_OFF
		elif msg[1] == "1":
			switch[1].s = PyIndi.ISS_ON
		indiclient.sendNewSwitch(switch)
	elif msg[0] == "weather_override":
		switch = indiclient.getDevice("Weather Watcher").getSwitch("WEATHER_OVERRIDE")
		if msg[1] == "0":
			switch[0].s = PyIndi.ISS_OFF
		elif msg[1] == "1":
			switch[0].s = PyIndi.ISS_ON
		indiclient.sendNewSwitch(switch)
	elif msg[0] == "rt_slewrate":
		switch = indiclient.getDevice("Pi Radiotelescope").getSwitch("TELESCOPE_SLEW_RATE")
		if msg[1] == "1":
			switch[0].s = PyIndi.ISS_ON
			switch[1].s = PyIndi.ISS_OFF
			switch[2].s = PyIndi.ISS_OFF
			switch[3].s = PyIndi.ISS_OFF
			switch[4].s = PyIndi.ISS_OFF
		elif msg[1] == "2":
			switch[0].s = PyIndi.ISS_OFF
			switch[1].s = PyIndi.ISS_ON
			switch[2].s = PyIndi.ISS_OFF
			switch[3].s = PyIndi.ISS_OFF
			switch[4].s = PyIndi.ISS_OFF
		elif msg[1] == "3":
			switch[0].s = PyIndi.ISS_OFF
			switch[1].s = PyIndi.ISS_OFF
			switch[2].s = PyIndi.ISS_ON
			switch[3].s = PyIndi.ISS_OFF
			switch[4].s = PyIndi.ISS_OFF
		elif msg[1] == "4":
			switch[0].s = PyIndi.ISS_OFF
			switch[1].s = PyIndi.ISS_OFF
			switch[2].s = PyIndi.ISS_OFF
			switch[3].s = PyIndi.ISS_ON
			switch[4].s = PyIndi.ISS_OFF
		elif msg[1] == "5":
			switch[0].s = PyIndi.ISS_OFF
			switch[1].s = PyIndi.ISS_OFF
			switch[2].s = PyIndi.ISS_OFF
			switch[3].s = PyIndi.ISS_OFF
			switch[4].s = PyIndi.ISS_ON
		indiclient.sendNewSwitch(switch)
	elif msg[0] == "rt_inttime":
		inttime = indiclient.getDevice("Pi Radiotelescope").getNumber("INT_TIME")
		inttime[0].value = int(msg[1])
		indiclient.sendNewNumber(inttime)
	elif msg[0] == "rt_move":
		if msg[1] == "E":
			switch = indiclient.getDevice("Pi Radiotelescope").getSwitch("TELESCOPE_MOTION_WE")
			switch[0].s = PyIndi.ISS_OFF
			switch[1].s = PyIndi.ISS_ON
		elif msg[1] == "W":
			switch = indiclient.getDevice("Pi Radiotelescope").getSwitch("TELESCOPE_MOTION_WE")
			switch[0].s = PyIndi.ISS_ON
			switch[1].s = PyIndi.ISS_OFF
		elif msg[1] == "N":
			switch = indiclient.getDevice("Pi Radiotelescope").getSwitch("TELESCOPE_MOTION_NS")
			switch[0].s = PyIndi.ISS_ON
			switch[1].s = PyIndi.ISS_OFF
		elif msg[1] == "S":
			switch = indiclient.getDevice("Pi Radiotelescope").getSwitch("TELESCOPE_MOTION_NS")
			switch[0].s = PyIndi.ISS_OFF
			switch[1].s = PyIndi.ISS_ON
		indiclient.sendNewSwitch(switch)
	elif msg[0] == "rt_stop":
		if msg[1] == "E" or msg[1] == "W":
			switch = indiclient.getDevice("Pi Radiotelescope").getSwitch("TELESCOPE_MOTION_WE")
		elif msg[1] == "N" or msg[1] == "S":
			switch = indiclient.getDevice("Pi Radiotelescope").getSwitch("TELESCOPE_MOTION_NS")
		switch[0].s = PyIndi.ISS_OFF
		switch[1].s = PyIndi.ISS_OFF
		indiclient.sendNewSwitch(switch)

# register term handler
signal.signal(signal.SIGTERM, term_handler)

if __name__ == '__main__':

	# Create MQTT client instance
	mqttclientid = "STW-RT300" + ClientIdMQTT()
	mqttclient = mqtt.Client(client_id = mqttclientid, clean_session = True)
	logger.debug("Creating an instance of MQTT client")

	mqttclient.reconnect_delay_set(min_delay=1, max_delay=60)
	mqttclient.username_pw_set(MQTT_USER, MQTT_PASS)
	mqttclient.on_connect = onConnectMQTT
	mqttclient.on_disconnect = onDisconnectMQTT

	# Try connecting to MQTT server or loop until MQTT server is available
	while True:
		try:
			mqttclient.connect(MQTT_HOST, MQTT_PORT, 60)
			break
		except KeyboardInterrupt:
			shutdown()
		except:
			logger.debug("MQTT server at " + MQTT_HOST + ":" + str(MQTT_PORT) + " not available. Retrying in 10 seconds")
			time.sleep(10)
			continue

	# Start mqtt loop
	mqttclient.loop_start()

	# Create an instance of the IndiClient class and initialize its host/port members
	indiclient = IndiClient()
	indiclient.setServer(INDI_HOST,INDI_PORT)

	while True:
		# Connect to INDI server or loop forever until INDI server available
		while not indiclient.isServerConnected():

			try:
				if not indiclient.connectServer():
					logger.debug("INDI server at " + indiclient.getHost() + ":" + str(indiclient.getPort()) + " not available. Retrying in 10 seconds")
					time.sleep(10)
			except KeyboardInterrupt:
				shutdown()
			except Exception as err:
				logger.error("Unexpected error connecting to INDI server", err)

		if indiclient.isServerConnected():

				try:
					# Get all devices
					devices = indiclient.getDevices()

					# Get properties and their associated values for all devices
					observatory_json = getJSON(devices)

					# Send MQTT message
					sendMQTT(observatory_json)

					# Loop every n seconds
					time.sleep(MQTT_POLLING)
				except KeyboardInterrupt:
					shutdown()
				except Exception as err:
					logger.error("Unexpected error timed polling INDI server", err)
