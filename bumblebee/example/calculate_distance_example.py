#!/usr/bin/env python

from flask import Flask, Response, render_template_string, request
import cv2
import logging
from OpenSSL import SSL
from cheroot.wsgi import Server as WSGIServer
from cheroot.ssl.builtin import BuiltinSSLAdapter
import threading
import time
import socket
import datetime
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from UtilBLB import *
def cal_distance():
    distance_list = [1000,2000,100,5000,8000] 
    sign = -1
    cur_pos = 10000
    checkpoints = []
    for distance in distance_list:
        cur_pos += sign * distance
        checkpoints.append(cur_pos)
        
    return checkpoints

print(cal_distance())