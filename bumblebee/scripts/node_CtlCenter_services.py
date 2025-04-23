#!/usr/bin/env python3
from node_CtlCenter_callback import *
        
def Rotate540(req):
    angle = req.name[1:]
    lsSend = []
    try:
      dicTmp = GetDicRotateMotorMain(angle)
      lsSend.append(dicTmp)
      SendCMD_Device(lsSend)
    except Exception as e:
      rospy.loginfo(e)
    resp = KillResponse()
    return resp
        
def Rotate360(req):
    angle = req.name[1:]
    lsSend = []
    try:
      dicTmp = GetDicRotateMotorTray(angle)
      lsSend.append(dicTmp)
      SendCMD_Device(lsSend)
    except Exception as e:
      rospy.loginfo(e)
    resp = KillResponse()
    return resp
