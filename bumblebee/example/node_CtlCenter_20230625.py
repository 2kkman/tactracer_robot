#!/usr/bin/env python3
import rospy
from typing import *
#/opt/ros/noetic/share/std_srvs/srv/Empty.srv , SetBool.srv, Trigger.srv
from std_srvs.srv import *
#/opt/ros/noetic/share/rospy_tutorials/srv/AddTwoInts.srv
# input : a,b (정수) , return sum (정수)
from rospy_tutorials.srv import *

#/opt/ros/noetic/share/turtlesim/srv/Kill.srv
# input : string, return : 없음
from turtlesim.srv import *
from tta_blb.srv import *
#import srvSingleParam, srvSingleParamResponse
import serial
import os
import subprocess
from varname import *
import termios, sys
import time
from Util import *
from UtilGPIO import *
from UtilBLB import *
from SPG_Keys import *
#from ServoNano import *
from varname import *
from std_msgs.msg import Header
from std_msgs.msg import String
from typing import List
from dataclasses import dataclass, field
from collections.abc import Sequence
dirPath = os.path.dirname(__file__)
# @dataclass
# class StateCrossMeta():
#     state0_from:int
#     state0_to:int
#     state1_from:int
#     state1_to:int

#StateInfo : Dict[str,StateCrossMeta] = {}
StateInfo : Dict[str,list] = {}
nodeStateOpen = [0,1] #인덱스가 0,1 일때는 Open 상태, 2,3 일땐 Close

strFilePath = f'{dirPath}/CROSS.txt'
file_list = getLines_FromFile(strFilePath)
for i in file_list:
  if i.find('#') >= 0:
    continue
  splitI = i.split(' ')
  if len(splitI) > 4:
    nodeID = (int)(splitI[0])
    #stateNode = StateCrossMeta( (int)(splitI[1]), (int)(splitI[2]),(int)(splitI[3]),(int)(splitI[4]) )
    #stateNode = [ [(int)(splitI[1]), (int)(splitI[2])],[(int)(splitI[3]),(int)(splitI[4])] ]
    stateNode = [ (int)(splitI[1]), (int)(splitI[2]),(int)(splitI[3]),(int)(splitI[4]) ]
    StateInfo[nodeID]=stateNode
print(StateInfo)

#전체 지도 맵을 읽어들인다.
strFilePathShortCut = f'{dirPath}/SHORTCUT.txt'
graph = LoadGraph(strFilePathShortCut)

publish_topic_name = 'KEEP_ARD'
param_ARD_show = False

portArd_M = '/dev/ttCARD_M' # 아두이노 시리얼 포트
#portArd_M = '/dev/ttyACM0' # 아두이노 메가 정품 시리얼 포트
serArd_M = None
listPortArd = [portArd_M]
listArdInstance = [serArd_M]
baudArd = 115200 # 아두이노 통신속도
lineArdData = []
pub_ka = rospy.Publisher(publish_topic_name, String, queue_size=1)

def prtMsg(sendbuf):
    rospy.loginfo(sendbuf)

def SendKeepAlive(sendbuf):
    global pub_ka
    if pub_ka is not None:
        pub_ka.publish(sendbuf)
        prtMsg(sendbuf)

def moveBLB(nodeFrom:int, nodeTo:int, isForward : bool):
  """1개의 노드를 이동한다. 노드 이동 전 분기점에 문제 없는지 검사한다.
  연결되어있는지 여부는 검사하지 않는다. (호출전 별도 검사)
  1. nodeFrom 을 기반으로 

  Args:
      nodeFrom (int): 현재 노드
      nodeTo (int): 이동할 노드
      isForward (bool): 진행방향이 Forward 인지 여부, 
  """
  '''
  0. 노드정보 확인. StateInfo - dict 개체 활용
  1. 만일 둘다 테이블이고 서로 연결되어있다면 문제가 없다.
  2. 둘중 하나 이상이 분기기라면 연결상태로 만듬.
  
  현재출발지가 테이블이면 그냥 가면 됨.
  분기기나 엘베인 경우
  1. 출발지 노드의 진입점 배열에 출발지 ID가 있는지 확인
  2. 당연히 있을거고, (없다면 익셉션) - 상태값이 0인지 1인지 확인 후 SetState 로 출발 노드 세팅
  '''
  scmFrom = StateInfo.get(nodeFrom, None)
  scmTo = StateInfo.get(nodeTo, None)
  
  scmList = [nodeFrom,nodeTo] #시작노드와 목적지 노드
  nodeReadyList = [0, 0]
  nodeSetResult = [False, False]
  iCnt = 0
  directionCheckCnt = 0
  for scmCur in scmList:
    scmCurValue = StateInfo.get(scmCur, None)
    if scmCurValue == None: #테이블이나 분기기, 엑세스 도어처럼 특수 노드가 아닌 경우는 그냥 출발.
      nodeSetResult[iCnt] = True
      nodeReadyList[iCnt] = 0
    else:
      setStateKey = scmCur
      if iCnt == 1:
        setStateKey = scmList[0]
      else:
        setStateKey = scmList[1]
        
      if setStateKey in scmCurValue:
        #nodeReadyList[iCnt] = (int)(scmCurValue.index(setStateKey)/2)
        nodeReadyList[iCnt] = scmCurValue.index(setStateKey)
      if setStateKey < 0:
        nodeSetResult[iCnt] = False
        #익셉션 발생시켜야 함.
      else:
        bIsOpen = nodeReadyList[iCnt] in nodeStateOpen
        nodeSetResult[iCnt]=setStateBranch(scmCur,bIsOpen)
    #directionCheckCnt += nodeReadyList[iCnt]
    iCnt += 1
  result = False in nodeSetResult
  
  #진입방향과 출구방향이 같으면 역방향, 그 외는 정방향으로 나가야 됨
  if nodeReadyList[0] % 2 == nodeReadyList[1] % 2:
    isForward = not isForward
  
  if result:
    print("ERROR")
  else:
    print(f"경로설정 성공! 이동합니다.{scmList},{isForward} : 진입인덱스 {nodeReadyList[1] }")
  return not result,nodeReadyList[1] 
  
  '''
  iPos = 0 일때 Open, iPos = 1일때 Close 상태
  bStraightPath = False 라는건 백워드로 가야한다는 뜻.
  '''

curStart = 5
curEnd = 3
lastNode = 1000

lsPath, pathCost = getPathBLB(graph,curStart,curEnd)

print(lsPath, pathCost)
moveForward = True
for curTarget in lsPath:
  if lastNode == curTarget:
    moveForward = not moveForward
  resultMove, inputIDX = moveBLB(curStart, curTarget, moveForward)
  print(f'{curStart} -> {curTarget} : {resultMove}, 진입인덱스 : {inputIDX}' )
  if resultMove:
    lastNode = curStart #이전 노드를 기억해둔다
    curStart = curTarget #이동 완료한 노드가 새 출발점이 된다
  else:
    print("ERROR")
    break
  

print('END')



class CtlCenter():
    def __init__(self):
        """_
        """
        self.CMD_service = rospy.Service('SetCross', Kill, self.SendCMD) #nodeFrom:str, nodeTo:str, crossID:str
        self.Set_service = rospy.Service('SetServingTables', Kill, self.SendCMD) #robotID:str, nodeID:str, orderID:str
        self.Route_service = rospy.Service('GetRoute', Kill, self.SendCMD) #robotID:str, tableNo:str
        
        # self.CHARGE_ENABLE_service = rospy.Service(ServiceList.NEST_CHARGE_ENABLE.value, SetBool, self.CHARGE_SET)
        # self.USB_ENABLE_service = rospy.Service(ServiceList.NEST_USB_ENABLE.value, SetBool, self.USB_SET)
        # self.BREAK_ENABLE_service = rospy.Service(ServiceList.NEST_BREAK_ENABLE.value, SetBool, self.BV_SET)
        # self.GPIO_service = rospy.Service(ServiceList.NEST_GPO_SET.value, utilboxKeyVal, self.GPO_SET)

        #rospy.loginfo(f'{CtlCenter.__class__} Started')

    def SendCMD(self, req):
      msg = req.name
      resp = KillResponse()
      return resp

lastUpdateTimeStamp = getDateTime() 
cntLoop = 0
if __name__ == "__main__":
  rospy.init_node(f'node_{CtlCenter.__name__}', anonymous = False)
  rate = rospy.Rate(100)
  CtlCenter()
  rospy.loginfo(f'{CtlCenter.__name__} Started')

  while not rospy.is_shutdown():
    try:
      td = getDateTime() - lastUpdateTimeStamp
      if td.total_seconds() >= 1:
        lastUpdateTimeStamp = getDateTime()
        rospy.loginfo(f'Loop : {cntLoop}')  
        cntLoop = 0
      else:
        cntLoop += 1
    except Exception as e:
      bReturn = False
      rospy.loginfo(traceback.format_exc())
      rospy.signal_shutdown(e)
    #rate.sleep()
  rospy.spin()
