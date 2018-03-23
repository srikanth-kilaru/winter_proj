#!/usr/bin/env python

import rospy
from finger_sensor_msgs.msg import FingerFAI, FingerSAI, FingerTouch

import matplotlib.pyplot as plt
import matplotlib.animation as animation

from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg
from pyqtgraph.ptime import time
import sys

sai_data1 = []
sai_data2 = []
sai_data3 = []
sai_data4 = []
sai_data5 = []
sai_data6 = []
sai_data7 = []
sai_data8 = []

l_app = QtGui.QApplication(["Left finger sensors"])

def update():
    global l_app, sai_curve1, sai_data1, sai_curve2, sai_data2,sai_curve3, sai_data3,sai_curve4, sai_data4,sai_curve5, sai_data5,sai_curve6, sai_data6,sai_curve7, sai_data7,sai_curve8, sai_data8,fai_curve

    sai_curve1.setData(sai_data1)
    sai_curve2.setData(sai_data2)
    sai_curve3.setData(sai_data3)
    sai_curve4.setData(sai_data4)
    sai_curve5.setData(sai_data5)
    sai_curve6.setData(sai_data6)
    sai_curve7.setData(sai_data7)
    sai_curve8.setData(sai_data8)
    l_app.processEvents()

def main():
    global l_app, sai_curve1,sai_curve2,sai_curve3,sai_curve4,sai_curve5,sai_curve6,sai_curve7,sai_curve8
    
    rospy.init_node('l_sensor_graphics')
    rospy.Subscriber('/left_finger/sai', FingerSAI, sai_l_callback)
    p=pg.plot()
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'w')
    p.setWindowTitle('SAI-FAI-Left')
    p.setInteractive(True)

    sai_curve1 = p.plot(pen='r', name="SAI L curve")
    sai_curve2 = p.plot(pen='g', name="SAI L curve")
    sai_curve3 = p.plot(pen='b', name="SAI L curve")
    sai_curve4 = p.plot(pen='c', name="SAI L curve")
    sai_curve5 = p.plot(pen='m', name="SAI L curve")
    sai_curve6 = p.plot(pen='y', name="SAI L curve")
    sai_curve7 = p.plot(pen=(200, 200, 255), name="SAI L curve")
    sai_curve8 = p.plot(pen='w', name="SAI L curve")

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(0)
    l_app.exec_()

    rospy.spin()

def sai_l_callback(msg):
    #print ("sai_l_callback:")
    #print("sensor1 = {}".format(msg.sensor1))
    #print("sensor2 = {}".format(msg.sensor2))
    #print("sensor3 = {}".format(msg.sensor3))    
    #print("sensor4 = {}".format(msg.sensor4))
    #print("sensor5 = {}".format(msg.sensor5))
    #print("sensor6 = {}".format(msg.sensor6))
    #print("sensor7 = {}".format(msg.sensor7))
    #print("sensor8 = {}".format(msg.sensor8))
    sai_data1.append(msg.sensor1)
    sai_data2.append(msg.sensor2)
    sai_data3.append(msg.sensor3)
    sai_data4.append(msg.sensor4)
    sai_data5.append(msg.sensor5)
    sai_data6.append(msg.sensor6)
    sai_data7.append(msg.sensor7)
    sai_data8.append(msg.sensor8)

    
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print ("Shutting Down")
