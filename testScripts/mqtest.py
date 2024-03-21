import paho.mqtt.publish as publish

publish.single("testTopic", "boo - love jetbot", hostname="10.0.0.1")

