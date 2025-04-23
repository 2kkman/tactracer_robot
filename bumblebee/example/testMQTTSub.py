#!/usr/bin/env python3
import paho.mqtt.client as mqtt
topicName = 'node_MQTT'
def on_connect(client, userdata, flags, rc):
    # This will be called once the client connects
    print(f"Connected with result code {rc}")
    # Subscribe here!
    client.subscribe(topicName)

def on_message(client, userdata, msg):
    print(f"Message received [{msg.topic}]: {msg.payload}")

client = mqtt.Client("mqtt-test") # client ID "mqtt-test"
client.on_connect = on_connect
client.on_message = on_message
client.username_pw_set("ttracer", "tt2015")
client.connect('www.i9man.com', 1883)
client.loop_forever()  # Start networking daemon