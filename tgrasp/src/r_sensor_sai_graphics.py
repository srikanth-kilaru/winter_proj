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
r_app = QtGui.QApplication(["Right finger sensors"])

def update():
    global r_app, sai_curve1, sai_data1, sai_curve2, sai_data2,sai_curve3, sai_data3,sai_curve4, sai_data4,sai_curve5, sai_data5,sai_curve6, sai_data6,sai_curve7, sai_data7,sai_curve8, sai_data8

    sai_curve1.setData(sai_data1)
    sai_curve2.setData(sai_data2)
    sai_curve3.setData(sai_data3)
    sai_curve4.setData(sai_data4)
    sai_curve5.setData(sai_data5)
    sai_curve6.setData(sai_data6)
    sai_curve7.setData(sai_data7)
    sai_curve8.setData(sai_data8)
    r_app.processEvents()

def main():
    global r_app, sai_curve1,sai_curve2,sai_curve3,sai_curve4,sai_curve5,sai_curve6,sai_curve7,sai_curve8
    
    rospy.init_node('r_sensor_graphics')
    rospy.Subscriber('/right_finger/sai', FingerSAI, sai_r_callback)

    p=pg.plot()
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'w')
    p.setWindowTitle('SAI-Right')
    p.setInteractive(True)

    sai_curve1 = p.plot(pen='r', name="SAI R curve")
    sai_curve2 = p.plot(pen='g', name="SAI R curve")
    sai_curve3 = p.plot(pen='b', name="SAI R curve")
    sai_curve4 = p.plot(pen='c', name="SAI R curve")
    sai_curve5 = p.plot(pen='m', name="SAI R curve")
    sai_curve6 = p.plot(pen='y', name="SAI R curve")
    sai_curve7 = p.plot(pen=(200, 200, 255), name="SAI R curve")
    sai_curve8 = p.plot(pen='w', name="SAI R curve")

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(0)
    r_app.exec_()

    rospy.spin()

def sai_r_callback(msg):
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
