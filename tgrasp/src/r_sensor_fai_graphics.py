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

fai_data1 = []
fai_data2 = []
fai_data3 = []
fai_data4 = []
fai_data5 = []
fai_data6 = []
fai_data7 = []
fai_data8 = []
r_app = QtGui.QApplication(["Right finger sensors"])

def update():
    global r_app, fai_curve1, fai_data1, fai_curve2, fai_data2,fai_curve3, fai_data3,fai_curve4, fai_data4,fai_curve5, fai_data5,fai_curve6, fai_data6,fai_curve7, fai_data7,fai_curve8, fai_data8,fai_curve

    fai_curve1.setData(fai_data1)
    fai_curve2.setData(fai_data2)
    fai_curve3.setData(fai_data3)
    fai_curve4.setData(fai_data4)
    fai_curve5.setData(fai_data5)
    fai_curve6.setData(fai_data6)
    fai_curve7.setData(fai_data7)
    fai_curve8.setData(fai_data8)
    r_app.processEvents()

def main():
    global r_app, fai_curve1,fai_curve2,fai_curve3,fai_curve4,fai_curve5,fai_curve6,fai_curve7,fai_curve8
    
    rospy.init_node('r_sensor_graphics')
    rospy.Subscriber('/right_finger/fai', FingerFAI, fai_r_callback)

    p=pg.plot()
    pg.setConfigOption('background', 'w')
    pg.setConfigOption('foreground', 'w')
    p.setWindowTitle('FAI-Right')
    p.setInteractive(True)

    fai_curve1 = p.plot(pen='r', name="FAI R curve")
    fai_curve2 = p.plot(pen='g', name="FAI R curve")
    fai_curve3 = p.plot(pen='b', name="FAI R curve")
    fai_curve4 = p.plot(pen='c', name="FAI R curve")
    fai_curve5 = p.plot(pen='m', name="FAI R curve")
    fai_curve6 = p.plot(pen='y', name="FAI R curve")
    fai_curve7 = p.plot(pen=(200, 200, 255), name="FAI R curve")
    fai_curve8 = p.plot(pen='w', name="FAI R curve")

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(0)
    r_app.exec_()

    rospy.spin()

def fai_r_callback(msg):
    fai_data1.append(msg.sensor1)
    fai_data2.append(msg.sensor2)
    fai_data3.append(msg.sensor3)
    fai_data4.append(msg.sensor4)
    fai_data5.append(msg.sensor5)
    fai_data6.append(msg.sensor6)
    fai_data7.append(msg.sensor7)
    fai_data8.append(msg.sensor8)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print ("Shutting Down")
