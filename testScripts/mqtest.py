#!/bin/python3.6
import paho.mqtt.publish as publish

# This script is used to test the mqtt connection between the jetbot and the rpi
publish.single("testTopic", "boo - love jetbot", hostname="10.0.0.1")

