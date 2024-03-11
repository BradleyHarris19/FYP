import matplotlib.pyplot as plt
from drawnow import drawnow
import numpy as np
import paho.mqtt.subscribe as subscribe

def makeFig():
    plt.scatter(xList,LList)
    plt.scatter(xList,RList)

plt.ion() # enable interactivity
fig=plt.figure() # make a figure

xList=list()
LList=list()
RList=list()
i = 0

while True:
    xList.append(i)

    leftmsg = subscribe.simple("test/testTopic/L", hostname="localhost")
    LList.append(leftmsg.payload)
    rightmsg = subscribe.simple("test/testTopic/R", hostname="localhost")
    RList.append(rightmsg.payload)
    
    if len(xList) > 100:
        xList.pop(0)
        LList.pop(0)
        RList.pop(0)

    drawnow(makeFig)
    plt.pause(0.00001)
    i +=  1
