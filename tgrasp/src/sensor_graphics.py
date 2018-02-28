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
fai_data = []

app = QtGui.QApplication([])

def update():
    global app, sai_curve1, sai_data1, sai_curve2, sai_data2,sai_curve3, sai_data3,sai_curve4, sai_data4,sai_curve5, sai_data5,sai_curve6, sai_data6,sai_curve7, sai_data7,sai_curve8, sai_data8,fai_curve, fai_data

    sai_curve1.setData(sai_data1)
    sai_curve2.setData(sai_data2)
    sai_curve3.setData(sai_data3)
    #sai_curve4.setData(sai_data4)
    #sai_curve5.setData(sai_data5)
    sai_curve6.setData(sai_data6)
    sai_curve7.setData(sai_data7)
    sai_curve8.setData(sai_data8)
    #fai_curve.setData(fai_data)
    app.processEvents()

def main():
    global app, sai_curve1,sai_curve2,sai_curve3,sai_curve4,sai_curve5,sai_curve6,sai_curve7,sai_curve8,fai_curve
    print("main initializing......")
    
    rospy.init_node('sensor_graphics')
    rospy.Subscriber('/left_finger/touch', FingerTouch, touch_l_callback)
    rospy.Subscriber('/right_finger/touch', FingerTouch, touch_r_callback)
    rospy.Subscriber('/left_finger/sai', FingerSAI, sai_l_callback)
    rospy.Subscriber('/right_finger/sai', FingerSAI, sai_r_callback)
    rospy.Subscriber('/left_finger/fai', FingerFAI, fai_l_callback)
    rospy.Subscriber('/right_finger/fai', FingerFAI, fai_r_callback)

    p=pg.plot()
    p.setWindowTitle('SAI-FAI-Left')
    p.setInteractive(True)
    sai_curve1 = p.plot(pen=(255,0,0), name="SAI L curve")
    sai_curve2 = p.plot(pen=(0,255,0), name="SAI L curve")
    sai_curve3 = p.plot(pen=(0,0,255), name="SAI L curve")
    sai_curve4 = p.plot(pen=(125,0,0), name="SAI L curve")
    sai_curve5 = p.plot(pen=(0,125,0), name="SAI L curve")
    sai_curve6 = p.plot(pen=(125,0,0), name="SAI L curve")
    sai_curve7 = p.plot(pen=(0,125,0), name="SAI L curve")
    sai_curve8 = p.plot(pen=(0,0,125), name="SAI L curve")

    #fai_curve = p.plot(pen=(0,255,0), name="FAI L curve")
    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(0)
    app.exec_()

    rospy.spin()
    print("main initialized.")

def touch_l_callback(msg):
    '''
    print ("touch_l_callback:")
    print("sensor1 = {}".format(msg.sensor1))
    print("sensor2 = {}".format(msg.sensor2))
    print("sensor3 = {}".format(msg.sensor3))    
    print("sensor4 = {}".format(msg.sensor4))
    print("sensor5 = {}".format(msg.sensor5))
    print("sensor6 = {}".format(msg.sensor6))
    print("sensor7 = {}".format(msg.sensor7))
    print("sensor8 = {}".format(msg.sensor8))
    '''
    
def touch_r_callback(msg):
    '''
    print ("touch_r_callback:")
    print("sensor1 = {}".format(msg.sensor1))
    print("sensor2 = {}".format(msg.sensor2))
    print("sensor3 = {}".format(msg.sensor3))    
    print("sensor4 = {}".format(msg.sensor4))
    print("sensor5 = {}".format(msg.sensor5))
    print("sensor6 = {}".format(msg.sensor6))
    print("sensor7 = {}".format(msg.sensor7))
    print("sensor8 = {}".format(msg.sensor8))
    '''
    
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

def sai_r_callback(msg):
    '''
    print ("sai_r_callback:")
    print("sensor1 = {}".format(msg.sensor1))
    print("sensor2 = {}".format(msg.sensor2))
    print("sensor3 = {}".format(msg.sensor3))    
    #print("sensor4 = {}".format(msg.sensor4))
    #print("sensor5 = {}".format(msg.sensor5))
    print("sensor6 = {}".format(msg.sensor6))
    print("sensor7 = {}".format(msg.sensor7))
    print("sensor8 = {}".format(msg.sensor8))
    '''
    
def fai_l_callback(msg):
    '''
    print ("fai_l_callback:")
    print("sensor1 = {}".format(msg.sensor1))
    print("sensor2 = {}".format(msg.sensor2))
    print("sensor3 = {}".format(msg.sensor3))    
    print("sensor4 = {}".format(msg.sensor4))
    print("sensor5 = {}".format(msg.sensor5))
    print("sensor6 = {}".format(msg.sensor6))
    print("sensor7 = {}".format(msg.sensor7))
    print("sensor8 = {}".format(msg.sensor8))
    fai_data.append(msg.sensor8)
    '''
    
def fai_r_callback(msg):
    '''
    print ("fai_r_callback:")
    print("sensor1 = {}".format(msg.sensor1))
    print("sensor2 = {}".format(msg.sensor2))
    print("sensor3 = {}".format(msg.sensor3))    
    print("sensor4 = {}".format(msg.sensor4))
    print("sensor5 = {}".format(msg.sensor5))
    print("sensor6 = {}".format(msg.sensor6))
    print("sensor7 = {}".format(msg.sensor7))
    print("sensor8 = {}".format(msg.sensor8))
    '''
    
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print ("Shutting Down")
