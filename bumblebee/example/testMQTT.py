#!/usr/bin/env python3
import paho.mqtt.client as mqtt  #import the client1
import time
mqtt.Client.connected_flag=False#create flag in class

def on_connect(client, userdata, flags, rc):
    if rc==0:
        client.connected_flag=True #set flag
        print("connected OK")
    else:
        print("Bad connection Returned code=",rc)

broker="www.i9man.com"
client = mqtt.Client("python1")             #create new instance 
client.username_pw_set(username="ttracer", password="tt2015")
client.on_connect=on_connect  #bind call back function
client.loop_start()
print("Connecting to broker ",broker)
client.connect(broker, 1883)      #connect to broker
while not client.connected_flag: #wait in loop
    print("In wait loop")
    time.sleep(1)
print("in Main Loop")
client.loop_stop()    #Stop loop 
client.disconnect() # disconnect