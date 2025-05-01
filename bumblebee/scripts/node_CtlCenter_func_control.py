#!/usr/bin/env python3
from node_CtlCenter_func_logic import *

def getLoadWeight():
    """현재까지 수신된 가장 마지막 로드셀 값

    Returns:
        _type_: _description_
    """
    try:
      strWeightValue = node_CtlCenter_globals.dicARD_CARRIER.get(
          CARRIER_STATUS.LOAD1.name, "0"
      )
      strWeightValue2 = node_CtlCenter_globals.dicARD_CARRIER.get(
          CARRIER_STATUS.LOAD2.name, "0"
      )
      r1 = int(strWeightValue)
      r2 = int(strWeightValue2)
    #   r1 = round(float(strWeightValue) * 43)
    #   r2 = round(float(strWeightValue2) * 43)
      r_total = r1+r2
      return r1,r2,r_total
    except Exception as e:
      rospy.loginfo(e)
      sMsg = traceback.format_exc()
      SendAlarmHTTP(sMsg,True,node_CtlCenter_globals.BLB_ANDROID_IP)
      return 0,0,0
  

def StartCaliTray():
  listCtl = []
  runningMotors = len(getRunningMotorsBLB())
  statusCurrent = node_CtlCenter_globals.robot.get_current_state()
  curDistanceSrvTele, curAngle_540,cur_angle_360  = GetCurrentPosDistanceAngle()
  if runningMotors == 0 and statusCurrent == Robot_Status.idle:
    if curDistanceSrvTele <= 110:
      sMsg = f'서빙암이격거리가 짧습니다. 현재이격:{curDistanceSrvTele}mm'
      TTSAndroid(sMsg)
      rospy.loginfo(sMsg)
      #listCtl.extend(GetStrArmExtendMain(110,0,True))
      return False
    pot_cur,not_cur,cmdpos,cur_pos =GetPotNotCurPosServo(ModbusID.ROTATE_SERVE_360)
    cmdCaliStart = getMotorMoveDic(ModbusID.ROTATE_SERVE_360.value, False, pot_cur-not_cur, 200, ACC_DECC_LONG, ACC_DECC_LONG)
    node_CtlCenter_globals.robot.trigger_start_calibration_tray()
    listCtl.append(cmdCaliStart)
    SendCMD_Device(listCtl)
    return True
  else:
    rospy.loginfo(f'활성화된 모터수:{runningMotors},로봇상태:{statusCurrent.name},서빙암이격거리:{curDistanceSrvTele}')
    return False
  
def StartCaliMainRotate():
    listCtl = []
    runningMotors = len(getRunningMotorsBLB())
    statusCurrent = node_CtlCenter_globals.robot.get_current_state()
    bIsAllMotorFolded = isReadyToMoveH_and_540()
    if not bIsAllMotorFolded:
        sMsg = f'암과 트레이를 완전히 접어주세요'
        TTSAndroid(sMsg)
        #listCtl.extend(GetLiftControl(True))
        return False
  
    if runningMotors == 0 and statusCurrent == Robot_Status.idle:
        pot_cur,not_cur,cmdpos,cur_pos =GetPotNotCurPosServo(ModbusID.ROTATE_MAIN_540)
        cmdCaliStart = getMotorMoveDic(ModbusID.ROTATE_MAIN_540.value, False, -(pot_cur-not_cur), MAINROTATE_RPM_CALI, ACC_DECC_LONG, ACC_DECC_LONG)
        SendCMD_Device([cmdCaliStart])
        node_CtlCenter_globals.robot.trigger_start_calibration_mainRotate()
        return True
    else:
        rospy.loginfo(f'활성화된 모터수:{runningMotors},로봇상태:{statusCurrent.name}')
    return False
  
def CheckMotorOrderValid(dictMotor:dict):
    mbid = dictMotor.get(MotorWMOVEParams.MBID.name, None)
    if mbid == None:
        return False
    timeMove = dictMotor.get(MotorWMOVEParams.TIME.name, None)
    mbid_class = ModbusID.from_value(mbid)
    cmd_pos, cur_pos = GetPosServo(mbid_class)
    potpos,notpos = GetPotNotServo(mbid_class)
    pos = int(dictMotor.get(MotorWMOVEParams.POS.name, MIN_INT))

    if mbid == str(ModbusID.ROTATE_MAIN_540.value) or mbid == str(ModbusID.ROTATE_SERVE_360.value):
        target_pulse = pos
        targetCW = target_pulse +potpos
        targetCCW = target_pulse -potpos
        arr =[target_pulse,targetCW,targetCCW]
        iPOSBack = min(arr, key=lambda x: abs(x - cur_pos))
        pos = iPOSBack
    
    if cur_pos == MIN_INT:
      return True
    # if pos >= potpos or pos <= notpos:
    #     return False
    distance_pulse = abs(int(pos) - cur_pos)
    if distance_pulse < (PULSES_PER_ROUND/5):
        if mbid_class == ModbusID.TELE_SERV_MAIN:
            SendInfoHTTP(f'현재서빙암펄스:{cur_pos},생략된지시정보:{dictMotor}')
        return False
    if timeMove is not None and float(timeMove) >= 3:
        return True    
    return True
    
def IsDoorMoving():
    if node_CtlCenter_globals.enableDummyArduino:
        return False

    doorStatusClose = node_CtlCenter_globals.dicARD_CARRIER.get(
        CARRIER_STATUS.I_DOOR_1_BOTTOM.name, None
    )
    doorStatusOpen = node_CtlCenter_globals.dicARD_CARRIER.get(
        CARRIER_STATUS.I_DOOR_1_TOP.name, None
    )
    doorStatusHome = node_CtlCenter_globals.dicARD_CARRIER.get(
        CARRIER_STATUS.I_DOOR_1_HOME.name, None
    )
    if isTrue(doorStatusClose) or isTrue(doorStatusOpen) or isTrue(doorStatusHome):
        return False
    else:
        return True

def GetLedStatus():    
    # led0 = '1,100'
    # led1 = '2,1000'
    led0 = node_CtlCenter_globals.dicARD_CARRIER.get(CARRIER_STATUS.L0.name,"-1,-1")
    led1 = node_CtlCenter_globals.dicARD_CARRIER.get(CARRIER_STATUS.L1.name,"-1,-1")
    lsData = [led0,led1]
    lsReturn = []
    for strTmp in lsData:
      # LED컬러코드0,블링크타임0,LED컬러코드1,블링크타임1
      lsLED = strTmp.split(sep=sDivItemComma)
      dictLED = {}
      dictLED[LED_STATUS.COLOR_CODE.name] = lsLED[0]
      dictLED[LED_STATUS.BLINK_TIME.name] = lsLED[1]
      lsReturn.append(dictLED)
    
    return lsReturn

def GetDoorStatus():
    doorStatusClose1 = node_CtlCenter_globals.dicARD_CARRIER.get(
        CARRIER_STATUS.I_DOOR_1_BOTTOM.name, -1
    )
    doorStatusOpen1 = node_CtlCenter_globals.dicARD_CARRIER.get(
        CARRIER_STATUS.I_DOOR_1_TOP.name, -2
    )
    doorStatusClose2 = node_CtlCenter_globals.dicARD_CARRIER.get(
        CARRIER_STATUS.I_DOOR_2_BOTTOM.name, -1
    )
    doorStatusOpen2 = node_CtlCenter_globals.dicARD_CARRIER.get(
        CARRIER_STATUS.I_DOOR_2_TOP.name, -2
    )
    isOpen1 = isTrue(doorStatusOpen1)
    isClose1 = isTrue(doorStatusClose1)
    isOpen2 = isTrue(doorStatusOpen2)
    isClose2 = isTrue(doorStatusClose2)
    resultReturn = TRAYDOOR_STATUS.MOVING
    resultArray = [isClose1,isClose2]
    if (isOpen1 and isClose1) or (isOpen2 and isClose2):
        resultReturn = TRAYDOOR_STATUS.DOORALARM
    elif isOpen1 or isOpen2:
        resultReturn= TRAYDOOR_STATUS.OPENED
    elif isClose1 and isClose2:
        resultReturn= TRAYDOOR_STATUS.CLOSED
    # else:
    #     return TRAYDOOR_STATUS.MOVING
    return resultReturn,resultArray

def GetRotateMainAngleFromPulse(target_pulse):
  potRotate540, notRotate540, posRotate540,posRotate540= GetPotNotCurPosServo(ModbusID.ROTATE_MAIN_540)
  angle = mapRange(target_pulse,notRotate540,potRotate540,0,MAX_ANGLE_TRAY)
  return round(angle)

def GetRotateMainPulseFromAngle(angleStr, modifyAngle=0):
  #angle = (180 + strToRoundedInt(angleStr)+modifyAngle) % 360
  angle = (360 + strToRoundedInt(angleStr)+modifyAngle) % 360
  potRotate540, notRotate540, poscmdRotate540,poscurRotate540= GetPotNotCurPosServo(ModbusID.ROTATE_MAIN_540)
  target_pulse = mapRange(angle,0,MAX_ANGLE_BLBBODY,notRotate540,potRotate540) 
  return round(target_pulse)

# 메인바디 회전 모터 제어 문자열
def GetDicRotateMotorMain(angleStr, rotateRPM=SPD_540, isTargetPulse=False):
    pot_cur540, not_cur540, cmdpos540, cur_pos540 = GetPotNotCurPosServo(ModbusID.ROTATE_MAIN_540)
    roundFullPulse = pot_cur540 - not_cur540  # 1회전 펄스 값 (649800)

    #target_pulse_raw = angleStr if isTargetPulse else GetRotateMainPulseFromAngle(angleStr)
    target_pulse = angleStr if isTargetPulse else GetRotateMainPulseFromAngle(angleStr)

    # # CW (시계 방향) 이동 거리
    # diff_CW = (target_pulse_raw - cur_pos540) % roundFullPulse
    # # CCW (반시계 방향) 이동 거리
    # diff_CCW = -((cur_pos540 - target_pulse_raw) % roundFullPulse)
    # target_pulseCW = cur_pos540 + diff_CW
    # target_pulseCCW = cur_pos540 + diff_CCW  # CCW 이동 (음수 값)

    # diff_total = abs(diff_CW) - abs(diff_CCW)
    # # 최단 거리 방향 선택
    # if abs(diff_total) < roundPulse:
    #     target_pulse = target_pulseCW if abs(target_pulseCW) < target_pulseCCW else target_pulseCCW
    # elif abs(diff_CW) < abs(diff_CCW):
    #     target_pulse = target_pulseCW  # CW 이동
    # else:
    #     target_pulse = target_pulseCCW  # CCW 이동 (음수 값)

    motorDic = getMotorMoveDic(ModbusID.ROTATE_MAIN_540.value, True, target_pulse,rotateRPM,ACC_540,DECC_540)
    motorDic[MotorWMOVEParams.TIME.name] = GetTimeFromRPM(ModbusID.ROTATE_MAIN_540, target_pulse, rotateRPM)
    return motorDic

def getMainRotateDicByDirection(dicTagretTableInfoTmp : dict):
    dicNewRotateTmp = {}
    cmd_posH, cur_posH = GetPosServo(ModbusID.MOTOR_H)
    pot_cur540,not_cur540,cmdpos540,cur_pos540 =GetPotNotCurPosServo(ModbusID.ROTATE_MAIN_540)
    sPOS_H = dicTagretTableInfoTmp[MotorWMOVEParams.POS.name]
    infoMOVE_DISTANCE = int(sPOS_H)
    #라이다를 앞으로 향하게 하려면 부등호가 오른쪽, 디스플레이가 앞으로 향하게 하려면 부등호가 왼쪽
    if infoMOVE_DISTANCE > cur_posH:  
        dicNewRotateTmp.update(GetDicRotateMotorMain(0))
        # moveRangeNot = abs(cur_pos540-not_cur540)
        # moveRangePot = abs(cur_pos540-pot_cur540)
        # newRotatePos = not_cur540 if moveRangeNot < moveRangePot else pot_cur540
        # dicNewRotateTmp.update(GetDicRotateMotorMain(newRotatePos,isTargetPulse=True))
    else:
        dicNewRotateTmp.update(GetDicRotateMotorMain(180))
    target_pulse = int(dicNewRotateTmp[MotorWMOVEParams.POS.name])
    dicNewRotateTmp[MotorWMOVEParams.TIME.name] = GetTimeFromRPM(ModbusID.ROTATE_MAIN_540, target_pulse, SPD_540)
    return dicNewRotateTmp

def GetRotateTrayAngleFromPulse(target_pulse):
  potRotate360, notRotate360, posRotate360,posRotate360= GetPotNotCurPosServo(ModbusID.ROTATE_SERVE_360)
  angle = mapRange(target_pulse,notRotate360,potRotate360,0,MAX_ANGLE_TRAY)
  return round(angle)

def GetRotateTrayPulseFromAngle(angleStr):
  angle = (strToRoundedInt(angleStr)) % 360
  potRotate360, notRotate360, posRotate360,posRotate360= GetPotNotCurPosServo(ModbusID.ROTATE_SERVE_360)
  target_pulse = mapRange(angle,0,MAX_ANGLE_TRAY,notRotate360,potRotate360) 
  return round(target_pulse)

#트레이 회전 모터 제어 dict
def GetDicRotateMotorTray(angleStr, rotateRPM = SPD_360,acc_rate =ACC_DECC_SMOOTH, decc_rate = DECC_360_DOWN):
    potRotate360, notRotate360, poscmd_Rotate360,posRotate360= GetPotNotCurPosServo(ModbusID.ROTATE_SERVE_360)
    roundFullPulse = potRotate360-notRotate360
    #target_pulse = GetRotateTrayPulseFromAngle(angleStr)
    target_pulse_raw = GetRotateTrayPulseFromAngle(angleStr)
    
    # CW (시계 방향) 이동 거리
    diff_CW = (target_pulse_raw - posRotate360) % roundFullPulse
    # CCW (반시계 방향) 이동 거리
    diff_CCW = -((posRotate360 - target_pulse_raw) % roundFullPulse)

    # 최단 거리 방향 선택
    diff_total = abs(diff_CW) - abs(diff_CCW)
    if abs(diff_CW) < abs(diff_CCW) or abs(diff_total) < roundPulse:
        target_pulse = posRotate360 + diff_CW  # CW 이동
    else:
        target_pulse = posRotate360 + diff_CCW  # CCW 이동 (음수 값)
    # target_CW = target_pulse_raw + roundFullPulse
    # target_CCW = target_pulse_raw - roundFullPulse
    # numbers= [target_pulse_raw,target_CW,target_CCW]
    # target_pulse = min(numbers, key=lambda x: abs(x - posRotate360))  
    motorStr = getMotorMoveDic(ModbusID.ROTATE_SERVE_360.value, True, target_pulse,rotateRPM,acc_rate,decc_rate)
    return motorStr

#주위 테이블 마커를 스캔하기 위한 모터 제어
def GetCameraRotateCmds():
    lsReturn = []
    # motorInnerExtend = getMotorMoveDic(ModbusID.TELE_SERV_INNER.value, True, INNERSTEP_PULSE_TRAYMOVE,SPD_INNER,ACC_INNER,DECC_INNER)
    # lsReturn.append([motorInnerExtend])
    lsReturn.append([GetDicRotateMotorTray(20,SPD_TRAY_MARKER_SCAN,DEFAULT_ACC,DEFAULT_DECC)])
    lsReturn.append([GetDicRotateMotorTray(0,SPD_TRAY_MARKER_SCAN,DEFAULT_ACC,DEFAULT_DECC)])
    # potInner,notInner = GetPotNotServo(ModbusID.TELE_SERV_INNER)
    # motorFold = getMotorMoveDic(ModbusID.TELE_SERV_INNER.value, True,notInner,SPD_INNER,ACC_INNER,DECC_INNER)
    # lsReturn.append([motorFold])
    return lsReturn

def CalculateTarPosFromLength(motorInstance : ModbusID, distancePos_millimeter):
    """
    실제 length 를 pulse 로 변환하는 함수. POT_NOT 값에 따라 자동계산한다.
    :return: (목표POS, 이동해야할 pulse)
    """    
    potpos,notpos = GetPotNotServo(motorInstance)
    if motorInstance == ModbusID.TELE_SERV_MAIN:
        notpos = 0
    cmdpos,curpos = GetPosServo(motorInstance)
    stroke = node_CtlCenter_globals.dicSTROKE[motorInstance]
    targetpos = mapRange(distancePos_millimeter,0,stroke,notpos,potpos)
    pulse_ToMove = abs(targetpos-curpos)
    return round(targetpos),round(pulse_ToMove)
    
def CalculateLengthFromPulse(motorInstance : ModbusID, pulse_pos):
    """
    실제 length 를 pulse 로 변환하는 함수. POT_NOT 값에 따라 자동계산한다.
    :return: (목표POS, 이동해야할 pulse)
    """    
    potpos,notpos = GetPotNotServo(motorInstance)
    cmdpos,curpos = GetPosServo(motorInstance)
    stroke = node_CtlCenter_globals.dicSTROKE[motorInstance]
    lenMillimeter = mapRange(pulse_pos,notpos,potpos,0,stroke)
    return round(lenMillimeter)
    
def GetSerArmDistance():
    posSrvInnerCmd, posSrvMainCur= GetPosServo(ModbusID.TELE_SERV_MAIN)
    potpos,notpos = GetPotNotServo(ModbusID.TELE_SERV_MAIN)
    # posSrvInnerCmd, posSrvInnerCur= GetPosServo(ModbusID.TELE_SERV_INNER)
    # lenSrvInner = CalculateLengthFromPulse(ModbusID.TELE_SERV_INNER, posSrvInnerCur)
    lenSrvMain=GetTargetLengthMMServingArm(posSrvInnerCmd, notpos)
    #lenSrvMain = CalculateLengthFromPulse(ModbusID.TELE_SERV_MAIN, posSrvMainCur)
    return round(lenSrvMain)
    #return round(lenSrvInner+lenSrvMain)

def GetCurrentPosDistanceAngle():
  potRotate540_cmd, notRotate540_cur, posRotate540_cmd,posRotate540_cur= GetPotNotCurPosServo(ModbusID.ROTATE_MAIN_540)
  potRotate360_cmd, notRotate360_cur, posRotate360_cmd,posRotate360_cur= GetPotNotCurPosServo(ModbusID.ROTATE_SERVE_360)
  cur_angle_540 = pulse_to_angle(posRotate540_cur, potRotate540_cmd, MAX_ANGLE_BLBBODY)%360
  #cur_angle_540 = (round(mapRange(posRotate540_cur,notRotate540_cur,potRotate540_cmd,0,MAX_ANGLE_BLBBODY)))%360
  cur_angle_360 = pulse_to_angle(posRotate360_cur, potRotate360_cmd, MAX_ANGLE_TRAY)%360
  return GetSerArmDistance(),cur_angle_540,cur_angle_360 

def GetArmStatus():
    pot_cur_arm1,not_cur_arm1 ,cmdpos_arm1,cur_pos_arm1 =GetPotNotCurPosServo(ModbusID.BAL_ARM1)
    pot_cur_telebal,not_cur_telebal ,cmd_pos_telebal,cur_pos_telebal =GetPotNotCurPosServo(ModbusID.TELE_BALANCE)  
    curDistanceSrvMilimeter =GetSerArmDistance()
    curPercentageSrvMilimeter = (100 * curDistanceSrvMilimeter) / STROKE_SERVE_TOTAL
    curBalancePulse = cur_pos_arm1+cur_pos_telebal
    curBalancePercentage = 100 * (curBalancePulse) / (pot_cur_arm1+pot_cur_telebal)
    return round(curDistanceSrvMilimeter),round(curPercentageSrvMilimeter),round(curBalancePulse),round(curBalancePercentage)

def GetCurrentPosXY():
  lenSrvInner,cur_angle_540 ,cur_angle_360 = GetCurrentPosDistanceAngle()
  return calculate_coordinates(lenSrvInner,cur_angle_540)

def PrintCurrentPos():
    curDistance, curAngle,cur_angle_360  = GetCurrentPosDistanceAngle()
    curX,curY = calculate_coordinates(curDistance,curAngle)
    print(f'현재위치:{curX},{curY}/{curDistance},메인회전:{curAngle}도,트레이회전:{cur_angle_360}')    

def GetDicServingArmRealTime(distanceServingTeleTotal,timeEstInput,bUseCurrentPosition=True):
    listReturn = []
    dicReturnInner = {}
    dicReturnSrvTele = {}
    #estTimeInner = -1
    estTimeSrvTele = -1
    distanceServingTele = 0
    distanceInnerTele = 0
    timeEst = 5 if timeEstInput <= 0 else timeEstInput
    # if timeEst <= 0:
    #     timeEst = 5
        #return listReturn,timeEst
    mbid_instance = ModbusID.TELE_SERV_MAIN
    pot_cur,not_cur,cmdpos,cur_pos =GetPotNotCurPosServo(mbid_instance)
    cur_distance = GetSerArmDistance()
    distanceServingTele = distanceServingTeleTotal
    # if distanceServingTeleTotal > STROKE_INNER:
    #     distanceServingTele = distanceServingTeleTotal - STROKE_INNER
    #     distanceInnerTele = STROKE_INNER
    # else:
    #     distanceInnerTele = distanceServingTeleTotal
    #목적지에 해당하는 PulsePos 값과 이동해야할
    #pulse_ServeInnerTarget,pulse_InnerPulseMove=CalculateTarPosFromLength(ModbusID.TELE_SERV_INNER,distanceInnerTele)    
    #pulse_ServeTeleTarget,pulse_SrvTelePulseMove=CalculateTarPosFromLength(ModbusID.TELE_SERV_MAIN,distanceServingTele)
    pulseTarget = GetTargetPulseServingArm(distanceServingTele)        
    pulse_SrvTelePulseMove = 0 if pulseTarget <= 0 else pulseTarget
    # if bUseCurrentPosition:
    #     pulse_SrvTelePulseMove = pulseTarget-cur_pos
    #pulse_MovePulseAbs 변수로 모터 동작 시간 및 속도 계산함.
    if bUseCurrentPosition:
        pulse_MovePulseAbs = abs(pulse_SrvTelePulseMove-cur_pos)
    else:
        pulse_MovePulseAbs = pulse_SrvTelePulseMove
    # if not bUseCurrentPosition:
    #     #pulse_InnerPulseMove = pulse_ServeInnerTarget
    #     pulse_SrvTelePulseMove = pulse_ServeTeleTarget
    #if not bUseCurrentPosition:
    log_all_frames()
    
    #수축시 서빙암 속도 조절
    if pulse_MovePulseAbs == 0:
        #수축시 타임 조절7
        timeEst += node_CtlCenter_globals.SERVING_ARM_FOLD_CONSTANT
    else:
        #전개시 타임 조절
        if timeEst > 3:
            timeEst -= node_CtlCenter_globals.SERVING_ARM_EXPAND_CONSTANT
    
    rpm_ServeTeleTarget = min( DEFAULT_RPM_SLOW,calculate_targetRPM_fromtime(pulse_MovePulseAbs/PULSES_PER_ROUND, timeEst))
    estTimeSrvTele = calculate_rpm_time(pulse_MovePulseAbs/PULSES_PER_ROUND, rpm_ServeTeleTarget)    
    timeSec_ExtendServe = estTimeSrvTele
    dicReturnSrvTele = getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value, True, pulseTarget,rpm_ServeTeleTarget,ACC_ST,DECC_ST)
    #dicReturnSrvTele = getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value, True, pulse_SrvTelePulseMove,rpm_ServeTeleTarget,ACC_ST,DECC_ST)
    # if pulse_SrvTelePulseMove <= 0:
    #     return [dicReturnSrvTele], timeSec_ExtendServe
    # if pulse_InnerPulseMove > 0:
    #     dicReturnInner = getMotorMoveDic(ModbusID.TELE_SERV_INNER.value, True, pulse_ServeInnerTarget,rpm_ServeInnerTarget,DEFAULT_ACC,DEFAULT_DECC)
    
    rospy.loginfo(f'{dicReturnSrvTele}')
    # if CheckMotorOrderValid(dicReturnInner):
    #   listReturn.append(dicReturnInner)
    #if CheckMotorOrderValid(dicReturnSrvTele):        
    listReturn.append(dicReturnSrvTele)
    #rospy.loginfo(f'서빙텔레스코프:스트로크{pulse_MovePulseAbs}%,시간:{timeSec_ExtendServe}')
    return listReturn, timeSec_ExtendServe

def GetArm2PosFromArm1Pos(pos_arm1):
    potpos_arm1,notpos_arm1, cmdpos_arm1,curpos_arm1 = GetPotNotCurPosServo(ModbusID.BAL_ARM1)
    potpos_arm2,notpos_arm2 ,cmdpos_arm2,curpos_arm2 =GetPotNotCurPosServo(ModbusID.BAL_ARM2)
    newPosArm2 = mapRange(pos_arm1,notpos_arm1,potpos_arm1,notpos_arm2,potpos_arm2)
    return newPosArm2

def GetTuningArms(relative_pulse_arm1):
    potpos_arm1,notpos_arm1, cmdpos_arm1,curpos_arm1 = GetPotNotCurPosServo(ModbusID.BAL_ARM1)
    #potpos_arm2,notpos_arm2 ,cmdpos_arm2,curpos_arm2 =GetPotNotCurPosServo(ModbusID.BAL_ARM2)
    newPosArm1 = curpos_arm1 + relative_pulse_arm1
    newPosArm2 = GetArm2PosFromArm1Pos(newPosArm1)
    
    lsTuningCmd = []
    dic1 = getMotorMoveDic(ModbusID.BAL_ARM1.value, True, newPosArm1, SPD_ARM1, DEFAULT_ACC, DEFAULT_DECC)
    dic2 = getMotorMoveDic(ModbusID.BAL_ARM2.value, True, newPosArm2, SPD_EXTEND_ARM2, DEFAULT_ACC, DEFAULT_DECC)
    lsTuningCmd.append(dic2)
    lsTuningCmd.append(dic1)
    SendCMD_Device(lsTuningCmd)

# def GetDicBalArmFixedRPM(distanceBalanceTotal,bUseCurrentPosition = True):
#     """
#     밸런스 암이 움직여야 하는 거리를 받아 소요시간과 명령어 세트를 리턴
#     밸런스암의 속도에 따라 
#     """
#     listReturn = []
#     dicReturnArm1 = {}
#     dicReturnArm2 = {}
#     dicReturnTeleBal = {}
#     rpm_Arm1 = -1
#     rpm_Arm2 = -1
#     rpm_TeleBal = -1
#     distanceArm1 = 0
#     distanceArm2 = 0
#     distanceBalanceTele = 0
    
#     potpos_arm1,notpos_arm1, cmdpos_arm1,curpos_arm1 = GetPotNotCurPosServo(ModbusID.BAL_ARM1)
#     potpos_arm2,notpos_arm2 ,cmdpos_arm2,curpos_arm2 =GetPotNotCurPosServo(ModbusID.BAL_ARM2)
#     #cmd_pos_teleBal,cur_pos_teleBal = GetPosServo(ModbusID.TELE_BALANCE)
#     if not bUseCurrentPosition:
#       curpos_arm1 = notpos_arm1
#       curpos_arm2 = notpos_arm2
      
#     pulse_arm1Target = potpos_arm1
#     pulse_arm2Target = potpos_arm2
#     angleA_Real = angleDummyB = angleDummyC=0
#     if distanceBalanceTotal < STROKE_BAL_EXTEND:
#       #밸런스 관절 이동거리 산출
#       if distanceBalanceTotal > 0:
#         angleA_Real,angleDummyB,angleDummyC= calculate_angles(LENGTH_ARM1, LENGTH_ARM2, distanceBalanceTotal)
#       if angleDummyC < 0:
#           angleDummyC = 0
#       #2관절 이동거리 산출하면 1관절은 자동계산됨.
#       angleHalf = int(angleDummyC/2)
#       pulse_arm1Target = calculate_rotations(angleHalf,potpos_arm1/3,potpos_arm1)
#       pulse_arm2Target = round(mapRange(angleDummyC,0,180,notpos_arm2,potpos_arm2))
#     else:
#       #1,2관절을 다 펴고 밸런스 텔레스코픽까지 뻗어야 하는 경우
#       distanceBalanceTele = distanceBalanceTotal - STROKE_BAL_EXTEND

#     #목적지에 해당하는 PulsePos 값과 이동해야할 pulse 계산
#     pulse_BalTeleTarget,pulse_TeleMoveAbs =  CalculateTarPosFromLength(ModbusID.TELE_BALANCE,distanceBalanceTele)
#     pulse_Arm1Move =  abs(pulse_arm1Target-curpos_arm1)
#     pulse_Arm2Move =  abs(pulse_arm2Target-curpos_arm2)
#     rpm_TeleBal = SPD_BALTELE
#     estTime_TeleBal = max(0,calculate_rpm_time(pulse_TeleMoveAbs/PULSES_PER_ROUND, rpm_TeleBal))
#     #estArm2Time = max(0,calculate_rpm_time(pulse_Arm2Move/PULSES_PER_ROUND, DEFAULT_SPD_ARM2))
#     estArm1Time = max(0,calculate_rpm_time(pulse_Arm1Move/PULSES_PER_ROUND, SPD_ARM1))
#     rpm_Arm1 = rpm_Arm2 = DEFAULT_RPM_NORMAL
#     if estArm1Time > 0:
#         rpm_Arm1 = rpm_Arm2 = calculate_targetRPM_fromtime(pulse_Arm2Move/PULSES_PER_ROUND, estArm1Time)
    
#     #밸런싱암이 전개하는 경우 - 텔레스코픽이 나중에 움직인다. 즉 rpm_TeleBal = 1
#     pulse_togo_arm2 = pulse_arm2Target-curpos_arm2
#     pulse_togo_arm2_abs = abs(pulse_togo_arm2) 
#     #if pulse_TeleMoveAbs > PULSES_PER_ROUND and pulse_togo_arm2 > PULSES_PER_ROUND:
#     if pulse_togo_arm2 > 0:
#         rpm_TeleBal = DEFAULT_RPM_MIN
#     acc_arm2 = ACC_ARM2_EXTEND
#     #수축인경우
#     if pulse_TeleMoveAbs > PULSES_PER_ROUND and pulse_BalTeleTarget == 0:
#         rpm_Arm1 = rpm_Arm2 = DEFAULT_RPM_MIN
#     else:   #전개인 경우
#         rpm_Arm1 = SPD_ARM1
#         if pulse_arm1Target-curpos_arm1 > 0:
#           rpm_Arm2 = SPD_EXTEND_ARM2
#         else:
#           rpm_Arm2 = SPD_EXTEND_ARM2
#           acc_arm2 = ACC_ARM2_FOLD
          
#     if pulse_Arm1Move > 0:
#         # if estArm2Time < 1:
#         #     rpm_Arm1 = DEFAULT_SPD_ARM2
#         # else:
#         #     rpm_Arm1 = DEFAULT_SPD_MIN
#         dicReturnArm1 = getMotorMoveDic(ModbusID.BAL_ARM1.value, True, pulse_arm1Target,rpm_Arm1,ACC_ARM1,DECC_ARM1)
#         #if CheckMotorOrderValid(dicReturnArm1):
#         listReturn.append(dicReturnArm1)
#     if pulse_Arm2Move > 0:
#         dicReturnArm2 = getMotorMoveDic(ModbusID.BAL_ARM2.value, True, pulse_arm2Target,rpm_Arm2,acc_arm2,DECC_ARM2)
#         #if CheckMotorOrderValid(dicReturnArm2):
#         listReturn.append(dicReturnArm2)
#     if pulse_TeleMoveAbs > 0:
#         if len(listReturn) == 0:
#             rpm_TeleBal = SPD_BALTELE
#         dicReturnTeleBal = getMotorMoveDic(ModbusID.TELE_BALANCE.value, True, pulse_BalTeleTarget,rpm_TeleBal,DEFAULT_ACC,DEFAULT_DECC)            
#         #if CheckMotorOrderValid(dicReturnTeleBal):
#         listReturn.append(dicReturnTeleBal)

#     #rospy.loginfo(f'밸런싱 텔레스코프:스트로크{100*distanceBalanceTotal/STROKE_SERVE_TOTAL:.1f}%,시간:{timeSec_ExtendServe:.1f}')
#     estTotal = estTime_TeleBal + estArm1Time
#     #print(f'밸런싱:{listReturn},밸런싱 운행시간 총합:{estTotal:.1f},2관절운행시간:{estArm2Time:.1f},밸런싱텔레소요시간:{estTime_TeleBal:.1f}')
#     return listReturn, max(estTotal,1)

def GetDicBalArmFixedPulse(pulseBalanceTotal,bUseCurrentPosition = True):
    """
    밸런스 암이 움직여야 하는 거리를 받아 소요시간과 명령어 세트를 리턴
    밸런스암의 속도에 따라 
    """
    listReturn = []
    dicReturnArm1 = {}
    dicReturnArm2 = {}
    dicReturnTeleBal = {}
    rpm_Arm1 = -1
    rpm_Arm2 = -1
    rpm_TeleBal = -1
    distanceArm1 = 0
    distanceArm2 = 0
    distanceBalanceTele = 0
    
    potpos_tele,notpos_tele, cmdpos_tele,curpos_tele = GetPotNotCurPosServo(ModbusID.TELE_BALANCE)
    potpos_arm1,notpos_arm1, cmdpos_arm1,curpos_arm1 = GetPotNotCurPosServo(ModbusID.BAL_ARM1)
    potpos_arm2,notpos_arm2 ,cmdpos_arm2,curpos_arm2 =GetPotNotCurPosServo(ModbusID.BAL_ARM2)
    if not bUseCurrentPosition:
      curpos_tele = notpos_tele
      curpos_arm1 = notpos_arm1
      curpos_arm2 = notpos_arm2
      
    pulse_arm1Target = 0
    pulse_arm2Target = 0
    pulse_BalTeleTarget = 0
    
    if pulseBalanceTotal > potpos_arm1:
      pulse_arm1Target = potpos_arm1
      pulse_BalTeleTarget = pulseBalanceTotal - potpos_arm1
      pot_tb,not_tb=GetPotNotServo(ModbusID.TELE_BALANCE)
      if pot_tb < pulse_BalTeleTarget:
        pulse_BalTeleTarget = pot_tb
    else:
      pulse_arm1Target = pulseBalanceTotal
    
    pulse_arm2Target = GetArm2PosFromArm1Pos(pulse_arm1Target)    
    pulse_TeleMoveAbs = abs(pulse_BalTeleTarget-curpos_tele)
    pulse_Arm1Move =  abs(pulse_arm1Target-curpos_arm1)
    pulse_Arm2Move =  abs(pulse_arm2Target-curpos_arm2)
    
    rpm_TeleBal = SPD_BALTELE
    estTime_TeleBal = max(0,calculate_rpm_time(pulse_TeleMoveAbs/PULSES_PER_ROUND, rpm_TeleBal))
    estArm1Time = max(0,calculate_rpm_time(pulse_Arm1Move/PULSES_PER_ROUND, SPD_ARM1))
    rpm_Arm1 = rpm_Arm2 = DEFAULT_RPM_NORMAL
    if estArm1Time > 0:
        rpm_Arm1 = rpm_Arm2 = calculate_targetRPM_fromtime(pulse_Arm2Move/PULSES_PER_ROUND, estArm1Time)
    
    #밸런싱암이 전개하는 경우 - 텔레스코픽이 나중에 움직인다. 즉 rpm_TeleBal = 1
    pulse_togo_arm2 = pulse_arm2Target-curpos_arm2
    pulse_togo_arm2_abs = abs(pulse_togo_arm2) 
    if pulse_togo_arm2 > 0:
        rpm_TeleBal = DEFAULT_RPM_MIN
    
    if pulse_togo_arm2 > 0:
        rpm_TeleBal = DEFAULT_RPM_MIN
    acc_arm2 = ACC_ARM2_EXTEND
    #수축인경우
    if pulse_TeleMoveAbs > PULSES_PER_ROUND and pulse_BalTeleTarget == 0:
        rpm_Arm1 = rpm_Arm2 = DEFAULT_RPM_MIN
    else:   #전개인 경우
        rpm_Arm1 = SPD_ARM1
        if pulse_arm1Target-curpos_arm1 > 0:
          rpm_Arm2 = SPD_EXTEND_ARM2
        else:
          rpm_Arm2 = SPD_EXTEND_ARM2
          acc_arm2 = ACC_ARM2_FOLD
             
    if pulse_Arm1Move > 0:
        dicReturnArm1 = getMotorMoveDic(ModbusID.BAL_ARM1.value, True, pulse_arm1Target,rpm_Arm1,ACC_ARM1,DECC_ARM1)
        listReturn.append(dicReturnArm1)
    if pulse_Arm2Move > 0:
        dicReturnArm2 = getMotorMoveDic(ModbusID.BAL_ARM2.value, True, pulse_arm2Target,rpm_Arm2,acc_arm2,DECC_ARM2)
        listReturn.append(dicReturnArm2)
    if pulse_TeleMoveAbs > 0:
        if len(listReturn) == 0:
            rpm_TeleBal = SPD_BALTELE
        dicReturnTeleBal = getMotorMoveDic(ModbusID.TELE_BALANCE.value, True, pulse_BalTeleTarget,rpm_TeleBal,ACC_BT,DECC_BT)            
        listReturn.append(dicReturnTeleBal)
    estTotal = estTime_TeleBal + estArm1Time
    return listReturn, max(estTotal,1)

def GetStrArmExtendMain(distanceServingTotal, angle_degrees, bUseCurrentPosition = False):
    lsArm = []
    #서빙텔레스코픽이 뻗어야 할 총 길이 mm 단위 / 메인회전모터 제어
    #distanceBalanceTotal = -1
    # distanceServingTeleTotal, angle_degrees = calculate_distance_and_angle(posX, posY)
    # if posX == 1 and posY == 0:
    #     distanceServingTeleTotal = 1065
    currentWeight1,currentWeight2,currentWeightTotal = getLoadWeight()
    #pot_telesrv, not_telesrv = GetPotNotServo(ModbusID.TELE_SERV_MAIN)
    pot_telesrv,not_telesrv,cmdpos_srv,cur_pos_srv =GetPotNotCurPosServo(ModbusID.TELE_SERV_MAIN)
    
    distanceServingTeleTotal = distanceServingTotal -STROKE_SERV_DEFAULT
    currentWeightTotal = 0
    #currentWeightTotal = getLoadWeight()
    
    #if distanceServingTeleTotal <= stroke_srv_default:
    if distanceServingTeleTotal < 0 and (abs(cur_pos_srv-not_telesrv) < roundPulse or not bUseCurrentPosition):
        #서빙암이 수축하는 경우
        targetPulse = GetTargetPulseServingArm(distanceServingTotal, not_telesrv)
        #targetPulse = round(mapRange(distanceServingTeleTotal,-stroke_srv_default, 0, not_telesrv, 0))
        lsRet = [getMotorMoveDic(ModbusID.TELE_SERV_MAIN.value, True, targetPulse, DEFAULT_RPM_SLOW, ACC_ST, DECC_ST)]
        diffRPM = round(targetPulse/roundPulse)
        timeRet = calculate_rpm_time(diffRPM, DEFAULT_RPM_SLOW)
        return lsRet
    
    if distanceServingTeleTotal < 0:
        distanceServingTeleTotal = 0
    
    
    lenWeightData = len(node_CtlCenter_globals.dicWeightBal)
    if distanceServingTeleTotal > STROKE_SERVE_TOTAL:
        distanceServingTeleTotal = STROKE_SERVE_TOTAL
    
    #pulseBalanceTotal = 1관절 + 밸런스텔레스코픽펄스의 합계
    pulseBalanceTotal = get_balanceArmPulse(currentWeightTotal, node_CtlCenter_globals.dicWeightBal) if lenWeightData >=2 else -1
    #pulseBalanceToGo=GetTargetPulseBalanceArm(distanceServingTotal)
    pulseBalanceToGo = round(mapRange(distanceServingTeleTotal, 0,STROKE_SERVE_TOTAL, 0,pulseBalanceTotal))
    #lsBalArm,timeEst1 = GetDicBalArmFixedRPM(distanceBalanceTotal,bUseCurrentPosition)
    #lsBalArm,timeEst1 = [], 0
    #if pulseBalanceToGo > 0:
    #stroke_balance_length = CalculateLengthFromPulse(ModbusID.TELE_SERV_MAIN, node_CtlCenter_globals.SERVING_ARM_EXPAND_PULSE)
    lsBalArm,timeEst1 = GetDicBalArmFixedPulse(pulseBalanceToGo,bUseCurrentPosition)
    # if distanceServingTeleTotal > stroke_balance_length:
    #     lsArm, timeEst2 = GetDicServingArmRealTime(distanceServingTeleTotal-stroke_balance_length,timeEst1,bUseCurrentPosition)
    # else:
    lsArm, timeEst2 = GetDicServingArmRealTime(distanceServingTotal,timeEst1,bUseCurrentPosition)
    timeEst = max(timeEst2,timeEst1)
    rospy.loginfo(f'시간:{timeEst:.1f},각도:{angle_degrees:.1f},밸런싱:{WEIGHT_BALARM_GRAM}g:{pulseBalanceTotal},서빙:{WEIGHT_SERVARM_GRAM}g:{distanceServingTeleTotal:.1f}mm')
    if len(lsBalArm) > 0:
        lsArm.extend(lsBalArm)
    return lsArm
# lsa =GetStrArmExtendMain(1250,0,False)
# print(lsa)
    
def GetListLiftDown(pot_lift=None):
    if pot_lift is None:
        pot_lift, not_lift = GetPotNotServo(ModbusID.MOTOR_V)
    lsLiftDown = getListedDic(getMotorMoveDic(ModbusID.MOTOR_V.value, True, pot_lift,SPD_LIFT,ACC_LIFT_DOWN,DECC_LIFT_DOWN))
    return lsLiftDown

def GetListLiftUp():
    pot_lift, not_lift = GetPotNotServo(ModbusID.MOTOR_V)
    lsLiftUp = getListedDic(getMotorMoveDic(ModbusID.MOTOR_V.value, True, not_lift,SPD_LIFT,ACC_LIFT_UP,DECC_LIFT_UP))
    return lsLiftUp

def GetLiftControl(isUp: bool, serve_distance_mm = 1800, serve_angle = 100, marker_angle = 90, height_pulse = 100000):
    pot_lift, not_lift = GetPotNotServo(ModbusID.MOTOR_V)
    listBLBTmp = []
    if isUp:    #수축-리프트업
        dicRotate360 = GetDicRotateMotorTray(0,SPD_360,ACC_360_UP, DECC_360_UP)
        lsArmControl = GetStrArmExtendMain(0,0,True)
        lsLiftUp = GetListLiftUp()
        #lsLiftUp.append(dicRotate360)
        listBLBTmp.append(lsLiftUp) #리프트를 먼저 올리고
        listBLBTmp.append([dicRotate360]) #트레이를 돌리고
        listBLBTmp.append(lsArmControl) #밸런스암 수축
    else:   #전개-리프트다운
        lsRotate540_down = getListedDic(GetDicRotateMotorMain(serve_angle))
        #dicRotate360_down = GetDicRotateMotorTray(marker_angle,ACC_360_DOWN, DECC_360_DOWN)
        dicRotate360_down = GetDicRotateMotorTray(marker_angle,SPD_360,ACC_360_DOWN, DECC_360_DOWN)
        #x,y = calculate_coordinates(serve_distance_mm,serve_angle)
        lsArmControl = GetStrArmExtendMain(serve_distance_mm,serve_angle,False)
        lsRotate360_down = [dicRotate360_down]
        lsLiftDown = GetListLiftDown(height_pulse)
        lsRotate360_down.extend(lsLiftDown)
        listBLBTmp.append(lsRotate540_down) #몸통회전해서 각 잡고
        listBLBTmp.append(lsArmControl) #서빙-밸런스 전개하고
        listBLBTmp.append([dicRotate360_down])   
        listBLBTmp.append(lsLiftDown)   

        # if IsEnableSvrPath():
        #     listBLBTmp.append(lsRotate360_down)   #지정된 각도에 따라 돌리고
        # else:
        #     listBLBTmp.append([dicRotate360_down])   
        #     listBLBTmp.append(lsLiftDown)   
        # #listBLBTmp.append(lsLiftDown)   #리프트를 내리면서 아르코 마커의 각도에 따라 내린다.
    return listBLBTmp

# def LiftDown():
#   rospy.loginfo(
#       f"Called : {sys._getframe(0).f_code.co_name}-{sys._getframe(1).f_code.co_name}-{sys._getframe(2).f_code.co_name}"
#   )
#   #node_CtlCenter_globals.listBLB.extend(GetLiftControlUp())
#   node_CtlCenter_globals.listBLB = GetLiftControlDown()
#   # node_CtlCenter_globals.curBLB_Status = (
#   #     BLB_STATUS_FIELD.LIFTING_UP
#   # )
#   node_CtlCenter_globals.cmdIdx = 0
#   node_CtlCenter_globals.is_docked = True
#   node_CtlCenter_globals.is_lifted = False
#   node_CtlCenter_globals.flag_liftup = (
#       False  # 상승 명령어가 들어갔으니 플래그 OFF
#   )  

# def LiftUp():
#   node_CtlCenter_globals.listBLB = GetLiftControlUp()
#   #node_CtlCenter_globals.listBLB.extend(GetLiftControlDown())
#   node_CtlCenter_globals.cmdIdx = 0
#   node_CtlCenter_globals.flag_liftup = True
#   node_CtlCenter_globals.is_docked = False
#   node_CtlCenter_globals.is_lifted = True  

def GetCustomFileControl(strProfileName: str):
    listBLBTmp = []
    strLiftFilePath = ""
    strLiftFilePath = f"{dirPath}/PROFILE_{strProfileName.upper()}.txt"

    try:
        with open(strLiftFilePath, "r") as f:
            list_ex_load = json.load(f)
            print(list_ex_load)
            for lsTmp in list_ex_load:
                listBLBTmp.append(lsTmp)
    except Exception as e:
        message = traceback.format_exc()
        logmsg = f"{message} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
        rospy.loginfo(logmsg)
    return listBLBTmp


def TTSItx(ttsMsg,isQueueing = True, ttsInterval = 1):
    return TTSCommon(ttsMsg,isQueueing,ttsInterval,UbuntuEnv.ITX.name)
        
def TTSCommon(ttsMsg,isQueueing = True, ttsInterval = 1, machineName=UbuntuEnv.QBI.name):
    global lastTTS
    if isTimeExceeded(lastTTS,ttsInterval):
        lastTTS = getDateTime()
        serviceTTS = ServiceBLB.TTS_ITX.name
        if machineName == UbuntuEnv.QBI.name:
            serviceTTS = ServiceBLB.TTS_QBI.name
        
        msg = f'{ttsMsg} from {sys._getframe(1).f_code.co_name}-{sys._getframe(0).f_code.co_name}'
        log_all_frames(msg)
        try:
            # 서비스 프록시 생성
            tts_service = rospy.ServiceProxy(serviceTTS, TTS)            
            # 서비스 요청 생성
            req = TTSRequest()
            req.text = ttsMsg
            req.queuing = isQueueing
            # 서비스 호출
            resp = tts_service(req)
            # 결과 출력
            print(f"Response: {resp.message}")
            return True
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False

    # if ":" in input_str:
    #     key, value = input_str.split(":", 1)  # ":" 기준으로 key와 value 분리
    #     if key in mapping:  # ✅ Key가 mapping 딕셔너리에 있는지 확인
    #         return f"{mapping[key]}:{value}"  # 변환된 key + 원래 value 유지
    #     else:
    #         return input_str  # 변환할 key가 없으면 원래 값 반환
    
    # return mapping.get(input_str, input_str)  # 단순 매핑 변환
  
def SendCMDArd(cmdStr):
    #isRealMachine = get_hostname().find(UbuntuEnv.ITX.name) >= 0
    bReturn,strResult=SendInfoHTTP(cmdStr.replace(sDivFieldColon,sDivSemiCol))
    logger_ard.info(cmdStr)
    print(bReturn,strResult)
    if isRealMachine:
        resultArd = service_setbool_client(ServiceBLB.CMDARD_QBI.value, cmdStr, Kill)
    else:
        cmdStr = replace_string(cmdStr)
        resultArd = service_setbool_client(ServiceBLB.CMDARD_ITX.value, cmdStr, Kill)        
    time.sleep(MODBUS_WRITE_DELAY)        
    return resultArd

def SendCMD_Device(sendbuf):
    cmdTmp = sendbuf
    if isinstance(sendbuf, list):
        if len(sendbuf) > 0:
            dictTmp = sendbuf[0]
            if isinstance(dictTmp, dict):
                if dictTmp.get('MBID') == '11':
                    print(dictTmp)

            cmdTmp = json.dumps(sendbuf)
        else:
            return False
    sMsg = log_all_frames('모터명령어',4)
    SendInfoHTTP(sMsg)
    return service_setbool_client(ServiceBLB.CMD_DEVICE.value, cmdTmp, Kill)

def LightWelcome(isOn):
    OnOff = 1 if isOn else 0
    sCmd = f"V12:{OnOff}"
    SendCMDArd(sCmd)

def SaveCurrentPos():
  dicPos = {}
  for mbid, dicTmp in node_CtlCenter_globals.dic_485ex.items():
      mbid_instance = ModbusID.from_value(mbid)
      cmdpos,cur_pos =GetPosServo(mbid_instance)
      dicPos[mbid] = cur_pos
  
  with open(strFileLastPos, "w") as file:
    json.dump(dicPos, file, indent=4, sort_keys=True)

def GetCurrentPos():
  dicPos = {}
  for mbid, dicTmp in node_CtlCenter_globals.dic_485ex.items():
      mbid_instance = ModbusID.from_value(mbid)
      cmdpos,cur_pos =GetPosServo(mbid_instance)
      dicPos[mbid_instance.name] = cur_pos
  return dicPos
        
def Tilting(tiltStatus : TRAY_TILT_STATUS):
    # minGYRO = -90
    # maxGYRO = 45
    # targetServo = mapRange(tiltStatus.value, minGYRO,maxGYRO,0,180)
    # SendCMDArd(f"S:10,{round(targetServo)}")
    print(TiltingARD(tiltStatus))
    ClearDistanceV()
    node_CtlCenter_globals.tiltStatus = tiltStatus
    UpdateLidarDistanceBoxTimeStamp()
    
def Tilting_old(tiltStatus : TRAY_TILT_STATUS):
    angle_myServo = int(node_CtlCenter_globals.dicARD_CARRIER.get(CARRIER_STATUS.TILT_ANGLE.name,MIN_INT))
    angle_Y = int(node_CtlCenter_globals.dicARD_CARRIER.get(DataKey.Angle_Y.name,MIN_INT))
    while angle_myServo == MIN_INT or angle_Y == MIN_INT:
        rospy.loginfo('Waiting for receive angle info...')
        angle_myServo = int(node_CtlCenter_globals.dicARD_CARRIER.get(CARRIER_STATUS.TILT_ANGLE.name,MIN_INT))
        angle_Y = int(node_CtlCenter_globals.dicARD_CARRIER.get(DataKey.Angle_Y.name,MIN_INT))        
        time.sleep(1)
    
    targetSvr = 0
    target_angleY = tiltStatus.value
    if abs(target_angleY-angle_Y) > 2:
        if target_angleY > angle_Y:
            targetSvr = 180
        
        SetIMUInterval(100)
        dtStart = getDateTime()
        SendCMDArd(f"S:15,{targetSvr}")
        while(abs(angle_Y - target_angleY) > 1):
            if isTimeExceeded(dtStart, 3000):
                break
            if targetSvr == 180 and target_angleY <= angle_Y:
                break
            if targetSvr == 0 and target_angleY >= angle_Y:
                break
            angle_myServo = int(node_CtlCenter_globals.dicARD_CARRIER.get(CARRIER_STATUS.TILT_ANGLE.name,0))
            angle_Y = int(node_CtlCenter_globals.dicARD_CARRIER.get(DataKey.Angle_Y.name,0))
            #rospy.loginfo(f'Target GYRO:{target_angleY},Current GYRO:{angle_Y},CurrentServo:{angle_myServo}')
            #time.sleep(MODBUS_WRITE_DELAY)
        SendCMDArd(f"S:STOP,{targetSvr}")
        SetIMUInterval(1000)
    angleNewmyServo = max(0, min(180, target_angleY - angle_Y + angle_myServo))    
    ClearDistanceV()
    node_CtlCenter_globals.tiltStatus = tiltStatus
    UpdateLidarDistanceBoxTimeStamp()
    # cm = CameraMode.FRONT_LOW
    # if tiltStatus == TRAY_TILT_STATUS.TiltDiagonal or tiltStatus == TRAY_TILT_STATUS.TiltMaxUp:
    #     cm = CameraMode.WIDE_LOW
    # SetCameraMode(cm)
    

#라
def TiltTableObstacleScan():
    Tilting(TRAY_TILT_STATUS.TiltTableObstacleScan)

#아르코마커 수평탐색
def TiltArucoScan(cropProfile=LidarCropProfile.CHECK_GROUND):
    SetCameraMode(CameraMode.WIDE_LOW)
    SetLidarCrop(cropProfile)
    Tilting(TRAY_TILT_STATUS.TiltDown)
    #Tilting(TRAY_TILT_STATUS.TiltFace)

#주행모드.
def TiltFace(cropProfile = LidarCropProfile.MOTOR_H):
    SetCameraMode(CameraMode.WIDE_LOW)
    SetLidarCrop(cropProfile)
    Tilting(TRAY_TILT_STATUS.TiltFace)
    ClearDistanceBox()

#서빙완료시
def TiltServFinish(cropProfile = LidarCropProfile.MOTOR_V):
  log_all_frames()
  lsResult = calculate_triangle_sides(0.55, 90, 30)
  SetLidarCrop(cropProfile)
  dicSet = { LIDAR_CROP_PARAMS.range_max_x.name : round(max(lsResult),3)} 
  Tilting(TRAY_TILT_STATUS.TiltMax)
  SetDynamicConfigROS(dicSet)

#20250427 - 1호기에서는 쓰이지 않음
def TiltDetectingMonitor(cropProfile = LidarCropProfile.MOTOR_V):
    SetLidarCrop(cropProfile)
    Tilting(TRAY_TILT_STATUS.TiltDetectingMonitor)
    ClearDistanceV()
    lsResult = calculate_triangle_sides(0.55, 90, 30)
    dicSet = { LIDAR_CROP_PARAMS.range_max_x.name : round(max(lsResult),3)}
    SetDynamicConfigROS(dicSet)
#사용하지 않음
def TiltDiagonal():
    Tilting(TRAY_TILT_STATUS.TiltDiagonal)    


def DoorStop():
    SendCMDArd(f"O:0,10")
    
def TrayStop():
    SendCMDArd(f"Y:0,10")

def DoorOpen(doorIdx=4):
    #SendInfoHTTP(log_all_frames(doorIdx))
    if isLiftTrayDownFinished() or not isRealMachine:
        #LightWelcome(False) 
        
        #SendCMDArd(f"O:2{sDivItemComma}{doorIdx}")
        
        #V캘리 하는 동안만 주석처리
        #TiltServFinish()

        return True
    else:
        SendMsgToMQTT(pub_topic2mqtt,MQTT_TOPIC_VALUE.BLB_ALARM.value,ALM_User.TRAYDOOR_SAFETY.value)
        return False

def DoorClose(doorIdx=4):
    LightWelcome(False)
    SendCMDArd(f"O:1{sDivItemComma}{doorIdx}")
    #TiltDown()

def TrayClose(spd=10):
    SendCMDArd(f"Y:1{sDivItemComma}{spd}")


def TrayOpen(spd=10):
    SendCMDArd(f"Y:2{sDivItemComma}{spd}")

#@rate_limited(3)  # 실행 제한 시간 설정
def LightTrayCell(traySector=0,blinkTimeMs=1000,colorCode = 0):
    #ledColor0,blink0,ledColor1,blink1  = GetLedStatus()
    dicLED = GetLedStatus()[traySector]
    curLedTime = int(dicLED.get(LED_STATUS.BLINK_TIME.name))
    curLedColor = int(dicLED.get(LED_STATUS.COLOR_CODE.name))
    if curLedTime == blinkTimeMs and curLedColor == colorCode:
        return
    sCmd = f"L:{colorCode}{sDivItemComma}{traySector}{sDivItemComma}{blinkTimeMs}"
    log_all_frames(sCmd)
    SendCMDArd(sCmd)


def GetpulseBalanceCalculated():
    curDistanceSrvTele2, curSrvPer, curBalPulse, curBalPer = GetArmStatus()
    #currentWeight1,currentWeight2,currentWeightTotal = getLoadWeight()
    currentWeightTotal = 0
    lenWeightData = len(node_CtlCenter_globals.dicWeightBal)
    pulseBalanceTotal = get_balanceArmPulse(currentWeightTotal, node_CtlCenter_globals.dicWeightBal) if lenWeightData >=2 else -1
    pulseBalanceCalculated = round(mapRange(curDistanceSrvTele2, 0,STROKE_SERVE_TOTAL, 0,pulseBalanceTotal))
    return pulseBalanceCalculated

def StopMotor(mbid, decc = EMERGENCY_DECC):
    sendInit = getMotorStopDic(mbid, decc)
    SendCMD_Device([sendInit])
    log_all_frames(f"Trying to stop Motor {mbid},DECC:{decc}")

def StopAllMotors(decc = EMERGENCY_DECC):
    lsModbusRequests = []
    for id in ModbusID:
      sendInit = getMotorStopDic(id.value, decc)
      #sendInit = getMotorSimpleCmdDic(id.value, MotorCmdField.WSTOP)
      lsModbusRequests.append(sendInit)
      #rospy.loginfo(f"Trying to stop Motor {id.name}:{id.value}")
    SendCMD_Device(lsModbusRequests)

def getBLBStatus() -> BLB_STATUS_FIELD:
    #1- "Ideal"
    dic_SpdTable = {}
    dic_PosTable = {}
    doorStatus,doorArray = GetDoorStatus()
    dictGlobal = node_CtlCenter_globals.dic_485ex.get(TopicName.BMS.name, {})
    curcadc = float(dictGlobal.get(MonitoringField_BMS.CurCadc.name, 1))
    charging = True if curcadc <= 0 else False
    #curTable,curNode = GetCurrentTargetTable()
    curNode = GetCurrentNode()
    for mbid, dictModbus in node_CtlCenter_globals.dic_485ex.items():
        if mbid in node_CtlCenter_globals.lsSlowDevices:
            continue
        cur_spd = int(dictModbus.get(MonitoringField.CUR_SPD.name, MIN_INT))
        cur_pos = int(dictModbus.get(MonitoringField.CUR_POS.name, MIN_INT))
        mbidInt = int(mbid)
        dic_SpdTable[mbidInt] = int(cur_spd)
        dic_PosTable[mbidInt] = int(cur_pos)

    spd_MOTOR_H = dic_SpdTable.get(ModbusID.MOTOR_H.value)
    spd_LIFT_V = dic_SpdTable.get(ModbusID.MOTOR_V.value)
    spd_TELE_SERV_MAIN_H = dic_SpdTable.get(ModbusID.TELE_SERV_MAIN.value)
    spd_MainRotate = dic_SpdTable.get(ModbusID.ROTATE_MAIN_540.value)
    spd_TrayRotate = dic_SpdTable.get(ModbusID.ROTATE_SERVE_360.value)
    
    minSpd = 10
    isDoorOpened = True if TRAYDOOR_STATUS.OPENED == doorStatus else False
    #if isDoorOpened and curNode == node_KITCHEN_STATION:
    if curNode == node_KITCHEN_STATION:        
        return BLB_STATUS_FIELD.READY
    elif isDoorOpened:
        return BLB_STATUS_FIELD.CONFIRM    
    elif IsSuspendedJob() and IsEnableSvrPath():
      return BLB_STATUS_FIELD.PAUSED
    elif isActivatedMotor(ModbusID.MOTOR_H.value):
        if abs(spd_MOTOR_H) < DEFAULT_RPM_SLOW * 1.1:
          return BLB_STATUS_FIELD.OBSTACLE_DETECTED
        return BLB_STATUS_FIELD.MOVING  #RUNNING
    else:
        if abs(spd_MainRotate) > minSpd:
            return BLB_STATUS_FIELD.ROTATING_MAIN
        elif abs(spd_LIFT_V) > minSpd or spd_TrayRotate > minSpd:
            if abs(spd_LIFT_V) <= minSpd:
                return BLB_STATUS_FIELD.LIFTING_DOWN
            if spd_LIFT_V > minSpd:
                return BLB_STATUS_FIELD.LIFTING_DOWN
            if spd_LIFT_V < minSpd:
                return BLB_STATUS_FIELD.LIFTING_UP
        #리프트 모터가 멈춰있고 30000 펄스 이상 하강한 경우는 도어 열어야 함
        # elif abs(spd_LIFT_V) < minSpd and isFinishedMotor(ModbusID.MOTOR_V) and abs(pos_LIFT_V) > 30000:
        #     return BLB_STATUS_FIELD.DOOR_MOVING
        elif spd_TELE_SERV_MAIN_H > minSpd: #spd_TELE_SERV_INNER_H > minSpd or 
            return BLB_STATUS_FIELD.EXPANDING
        elif spd_TELE_SERV_MAIN_H < -minSpd: #spd_TELE_SERV_INNER_H < -minSpd or 
            return BLB_STATUS_FIELD.FOLDING
        elif curNode == node_CHARGING_STATION and charging:
            return BLB_STATUS_FIELD.CHARGING
        elif GetWaitCrossFlag():
            return BLB_STATUS_FIELD.WAITING_CROSS
        else:
            return BLB_STATUS_FIELD.N_A

def SendStatus(blb_status: BLB_STATUS_FIELD):
    cur_pos_cross = try_parse_int(GetCrossInfo(MonitoringField.CUR_POS.name), MIN_INT)
    DI_POT_15,DI_NOT_15,DI_ESTOP_15,SI_POT = GetPotNotHomeStatus(ModbusID.MOTOR_H)
    table_target = GetTableTarget()
    dicCurrentJob = GetCurrentJob(table_target)
    dicFirstJob = GetCurrentJob(table_target, True)
    dicLastJob = GetCurrentJob(table_target, False)
    vx = 0
    vy = 0
    cur_rpm = getMotorSpdDirection(ModbusID.MOTOR_H.value)
    cur_spd= abs(calculate_speed_fromRPM(cur_rpm))
    curDirection = dicCurrentJob.get(APIBLB_FIELDS_INFO.direction.name, None)
    if curDirection is not None:
      if curDirection == 'N': #y증가
        vy = cur_spd
      if curDirection == 'S': #y감소
        vy = -cur_spd
      if curDirection == 'E': #x증가
        vx = cur_spd
      if curDirection == 'W': #x감소
        vx = -cur_spd
    
    dicStatusData = {}
    dicStatusData[APIBLB_FIELDS_INFO.vy.name] = vy
    dicStatusData[APIBLB_FIELDS_INFO.vx.name] = vx
    dicStatusData[APIBLB_FIELDS_INFO.w.name] = 0
    dicStatusData[BLB_STATUS.ID.name] = device_ID
    dicStatusData[BLB_STATUS.STATUS.name] = blb_status.name
    lsCurTable,curNode = GetCurrentTableNode()
    dicStatusData[BLB_STATUS.NODE_CURRENT.name] = str(curNode)
    #dicStatusData[BLB_STATUS.TABLE_CURRENT.name] = str(lsCurTable)
    dicStatusData[BLB_STATUS.NODE_TARGET.name] = str(table_target)
    # dict 에서 key와 value를 변수에 할당
    key, value = next(iter(node_CtlCenter_globals.dict_WaitReason.items()))
    dicStatusData[APIBLB_FIELDS_STATUS.error_code.name] = key
    dicStatusData[APIBLB_FIELDS_STATUS.alarm_msg.name] = value
    curX,curY = GetLocXY()
    dicStatusData[APIBLB_FIELDS_INFO.x.name] = curX
    dicStatusData[APIBLB_FIELDS_INFO.y.name] = curY
    dicStatusData[APIBLB_FIELDS_NAVI.current_station.name] = GetCurrentNode()
    curDistanceSrvTele, curAngle540,cur_angle_360  = GetCurrentPosDistanceAngle()
    dicStatusData[APIBLB_FIELDS_INFO.angle.name] = curAngle540
    curTargetTable,curTarNode = GetCurrentTargetTable()
    dfReceived = GetDF(curTargetTable)
    if dfReceived is None:
      dfReceived = pd.DataFrame()
    dicStatusData[APIBLB_FIELDS_STATUS.df.name] = dfReceived.to_json(orient='records')    
    dicStatusData[MonitoringField.LASTSEEN.name] = getDateTime().timestamp()
    sendbuf = json.dumps(dicStatusData)
    if pub_BLB_STATUS is not None:
        pub_BLB_STATUS.publish(sendbuf)
        prtMsg(sendbuf)
        
    if len(node_CtlCenter_globals.StateSet) > 0:
      if len(node_CtlCenter_globals.stateDic) < 0:
          return
      dicReceivedJC = {}
      bSendJCCmd = False
      for jc_id, isCrossStatus in node_CtlCenter_globals.stateDic.items():
          #dicReceivedJC[jc_id] = dicStatus[BLB_CMD.STATE.name]
          dicReceivedJC[int(jc_id)] = isCrossStatus
      for jc_id, isCrossStatus in node_CtlCenter_globals.StateSet.items():
          currentState = dicReceivedJC.get(jc_id)
          if node_CtlCenter_globals.StateSet[jc_id] != currentState:
              bSendJCCmd = True
          
      #if bSendJCCmd and isRealMachine and isTrue(motor_pot) :
      # TODO : 충전소 가는 방향 0 and 레일 주행 1
      #목적지가 충전소로 가는 경우에는 반드시 현재 RFID태그가 읽히고 있고 POT_15 ON 이어야 함.
      #분기기로 가는 경우에는 현재 정지상태인지만 체크하면 됨.
      #제정신일때 다시 정의하자.
      if bSendJCCmd and isRealMachine:
        rospy.loginfo(f'범블비가 원하는 분기기 세팅:{node_CtlCenter_globals.StateSet},현재 수신된 분기기 세팅:{node_CtlCenter_globals.stateDic}')
          
        for jc_id, isCrossStatus in node_CtlCenter_globals.StateSet.items():
          if cur_pos_cross < roundPulse or cur_pos_cross > 490000:
            if isCrossStatus == 0:
                if curNode in NODES_SPECIAL and isTrue(DI_POT_15): #세로 로 만들어야 할때 - 분기기에서 충전소로 나갈때
                    API_CROSS_set(jc_id,isCrossStatus)
                #     if isTrue(DI_POT_15):
                #         API_CROSS_set(jc_id,isCrossStatus)
                # else:   #세로 로 만들어야 할때 - 충전소에서 분기기로 진입시
                #     if SI_POT == 'GPI':
                #         API_CROSS_set(jc_id,isCrossStatus)
            elif isCrossStatus == 1:    #가로로 만들어야 할때
                if curNode == 10:   #분기기에 올라타 있는 상태 - 
                    if isTrue(DI_POT_15):
                        API_CROSS_set(jc_id,isCrossStatus)
                else:
                    if SI_POT == 'GPI':
                        API_CROSS_set(jc_id,isCrossStatus)
        
        # dicSendMqttTopic = {}
        # # mqttTopic = f'BLB/mcu_relay_CMD/set'
        # mqttRequestTopic = MQTT_TOPIC_VALUE.MSG_CROSS_REQUEST.name
        # dicSendMqttTopic[MQTT_FIELD.TOPIC.name] = mqttRequestTopic
        # dicSendMqttTopic[MQTT_FIELD.PAYLOAD.name] = json.dumps(node_CtlCenter_globals.StateSet)
        # # dicSendMqttTopic[MQTT_FIELD.PAYLOAD.name] = f'E:{statusVal},ID:{nodeID}'
        # data_out = json.dumps(dicSendMqttTopic)
        # pub_topic2mqtt.publish(data_out)
        

def HomeCall(homerightnow=False):
  node_current = GetCurrentNode()
  if node_current is None:
    return APIBLB_ACTION_REPLY.E101  
  
  if int(node_current) == node_KITCHEN_STATION:
    return APIBLB_ACTION_REPLY.R101
  
  log_all_frames()

  if homerightnow:
    node_CtlCenter_globals.listBLB.clear()
    node_CtlCenter_globals.listTable.clear()
    TTSAndroid("리턴 명령어가 수신되었습니다.")
    StopAllMotors(DECC_MOVE_H)
  
  if IsEnableSvrPath():
    API_SetOrderHome()
    time.sleep(0.1)
  InsertTableList(HOME_TABLE)
  #node_CtlCenter_globals.robot.trigger_complete_serving()
  SetWaitConfirmFlag(False,AlarmCodeList.OK)
  SendInfoHTTP(ALM_User.JOB_HOMECALL.value)
  
  
  return APIBLB_ACTION_REPLY.R101

#@log_arguments
def ResumeState(curTargetTable=None):  
    try:
        #StopAllMotors(ACC_DECC_SMOOTH)
        if node_CtlCenter_globals.robot.get_current_state() != Robot_Status.paused:
            TTSAndroid('서빙중이 아닙니다')
            return pd.DataFrame(), APIBLB_ACTION_REPLY.R103
        
        key_MOTOR_H=str(ModbusID.MOTOR_H.value)
        
        if curTargetTable is None:
            curTargetTable,curTarNode = GetCurrentTargetTable()
        dfReceived = GetDF(curTargetTable)
        PrintDF(dfReceived)
        lsPaused = []
        if IsEnableSvrPath():
            lsPaused = dfReceived[dfReceived[APIBLB_FIELDS_NAVI.workstatus.name] == APIBLB_STATUS_TASK.Paused.value].tail(1).to_dict(orient='records')
        
        #1. 기존 주행완료 후 완료노드 위에서 Pause한 경우, Pause 해제 하면 다음 오더로 넘어간다.
        if len(lsPaused) == 0:
            SetWaitConfirmFlag(False,AlarmCodeList.OK)
            if isActivatedMotor(key_MOTOR_H):
                cmdSpdChangeSrv = getMotorSpeedDic(ModbusID.MOTOR_H.value,True,SPD_MOVE_H,ACC_MOVE_H,DECC_MOVE_H)
                rospy.loginfo(cmdSpdChangeSrv)
                SendCMD_Device([cmdSpdChangeSrv])
            return
        dicPause = lsPaused[0]
        #2.EndNode 및 현재 위치 확인.
        
        endNode = str(dicPause[APIBLB_FIELDS_TASK.endnode.name])
        iDetailcode_running = dicPause[key_detailcode]
        #pos_abs = dicPause[APIBLB_FIELDS_TASK.POS_ABS.name]
        #3.EndNode 가 숫자가 아니면 마지막 노드 다 온거라서 그대로 내리면 됨.
        tableNo = try_parse_int(endNode, MIN_INT)
        dfReceived.loc[dfReceived[key_detailcode] == iDetailcode_running, APIBLB_FIELDS_NAVI.workstatus.name] = APIBLB_STATUS_TASK.Running.value
        resultAPI, nodeReturn = API_robot_navigation_info(dfReceived,APIBLB_STATUS_TASK.Paused)
        SendInfoHTTP(ALM_User.JOB_RESUMED.value)
        TTSAndroid("운행을 재개합니다.")
        if nodeReturn is None:
            rospy.loginfo(resultAPI)
            SendAlarmHTTP('API_robot_navigation_info 응답값 이상', True, node_CtlCenter_globals.BLB_ANDROID_IP)
        else:
            SetCurrentNode(nodeReturn)
        # DataFrame을 리스트로 변환
        data_list = dfReceived.to_dict(orient="records")
        # 리스트를 JSON 문자열로 변환
        json_string = json.dumps(data_list, ensure_ascii=False)
        pub_DF.publish(json_string)
        
        #1.Endnode (마지막 노드인경우)
        if tableNo == MIN_INT:
            # if isLiftTrayDownFinished():
            #     DoorClose()
            #기존 위치제어 정보 클리어
            node_CtlCenter_globals.listBLB.clear()

            dicTagretTableInfo = getTableServingInfo(endNode)
            rospy.loginfo(dicTagretTableInfo)
            #현재 도달한 목적지 테이블의 위치를 가져온다.
            infoSERVING_ANGLE = dicTagretTableInfo.get(TableInfo.SERVING_ANGLE.name, None)
            infoSERVING_DISTANCE = dicTagretTableInfo.get(TableInfo.SERVING_DISTANCE.name, None)
            infoMARKER_ANGLE = dicTagretTableInfo.get(TableInfo.MARKER_ANGLE.name, None)
            infoLIFT_Height = dicTagretTableInfo.get(TableInfo.HEIGHT_LIFT.name, None)
            lsLiftDown = GetLiftControl(False, infoSERVING_DISTANCE, infoSERVING_ANGLE, infoMARKER_ANGLE,infoLIFT_Height)
            node_CtlCenter_globals.listBLB.extend(lsLiftDown)
        #EndNode 가 숫자라면 마저 가야됨. 이때는 dicTargetPos 에 남아있는 곳으로 이동하는 명령어 추가.
        else:
            if isActivatedMotor(key_MOTOR_H):
                cmdSpdChangeSrv = getMotorSpeedDic(ModbusID.MOTOR_H.value,True,SPD_MOVE_H,ACC_MOVE_H,DECC_MOVE_H)
                rospy.loginfo(cmdSpdChangeSrv)
                SendCMD_Device([cmdSpdChangeSrv])
            elif node_CtlCenter_globals.dicTargetPos.get(key_MOTOR_H) is not None:
                iTargetPulse = node_CtlCenter_globals.dicTargetPos[key_MOTOR_H]
                dicInfo_local = getMotorMoveDic(key_MOTOR_H, True, iTargetPulse, SPD_MOVE_H* SPEED_RATE_H, ACC_MOVE_H,DECC_MOVE_H)
                if CheckMotorOrderValid(dicInfo_local):
                    node_CtlCenter_globals.listBLB.insert(0,dicInfo_local)
        SetWaitConfirmFlag(False,AlarmCodeList.JOB_RESUME)
        AppendSendStatus(BLB_STATUS_FIELD.Started)
        # PrintDF(dfReceived)
        # API_robot_navigation_info(dfReceived)
        node_CtlCenter_globals.robot.trigger_resume_serving()
        return APIBLB_ACTION_REPLY.R101
    except Exception as e:
      rospy.loginfo(e)
      sMsg = traceback.format_exc()
      SendAlarmHTTP(sMsg,True,node_CtlCenter_globals.BLB_ANDROID_IP)
      SetWaitConfirmFlag(True,AlarmCodeList.OK)
      return APIBLB_ACTION_REPLY.R103

def SetPauseState():
    try:
        if IsSuspendedJob():
            TTSAndroid('이미 일시정지 된 작업입니다')
            return pd.DataFrame(), APIBLB_ACTION_REPLY.R102
        if node_CtlCenter_globals.robot.get_current_state() != Robot_Status.onServing:
            TTSAndroid('서빙중이 아닙니다')
            return pd.DataFrame(), APIBLB_ACTION_REPLY.R103
            
        SetWaitConfirmFlag(True,AlarmCodeList.JOB_PAUSE)
        key_motorH = str(ModbusID.MOTOR_H.value)
        key_motorMainRotate = str(ModbusID.ROTATE_MAIN_540.value)
        #curTargetTable=GetTableTarget()
        dfReceived = GetDF()
        lsCurrentRow = []
        if IsEnableSvrPath():
            if dfReceived is None:
                TTSAndroid('현재 운영상태가 아닙니다')
                return pd.DataFrame(), APIBLB_ACTION_REPLY.R103
            try:
                lsCurrentRow = dfReceived.loc[dfReceived[APIBLB_FIELDS_TASK.workstatus.name] == APIBLB_STATUS_TASK.Running.value].tail(1).to_dict(orient='records')
            except Exception as e:
                rospy.loginfo(e)
                sMsg = traceback.format_exc()
                SendAlarmHTTP(sMsg,True,node_CtlCenter_globals.BLB_ANDROID_IP)
        
        iDetailcode_running = MIN_INT
        #동작중인 모터 리스트 저장
        lsMotorsActivate = copy.deepcopy(getRunningMotorsBLB())
        #주행중이었던 경우 인접한 노드의 위치를 계산하여 저장한다.
        node_CtlCenter_globals.dicTTS.clear()
        #SetWaitConfirmFlag(True,AlarmCodeList.JOB_PAUSE)
        
        #while 코드 삽입.
        #모든 모터가 멈출때까지 잠시 대기.
            
        if len(lsCurrentRow) > 0:
            dicRunning = lsCurrentRow[0]
            strDetailcode_running=dicRunning[key_detailcode]
            iDetailcode_running = try_parse_int(strDetailcode_running)
            PrintDF(dfReceived)
            strDetailcode_passed = dfReceived[dfReceived[key_detailcode].astype(int) < iDetailcode_running][key_detailcode].max()
            lsPassed = dfReceived.loc[dfReceived[key_detailcode].astype(str) == str(strDetailcode_passed)].tail(1).to_dict(orient='records')
        
        if key_motorH in lsMotorsActivate:
            spd_h = int(GetItemsFromModbusTable(ModbusID.MOTOR_H,MonitoringField.CUR_SPD))
            decc_current_h = min(abs(spd_h*2), DECC_MOVE_H)
            #StopMotor(ModbusID.MOTOR_H.value,decc_current_h )
            cmdSpdChangeSrv = getMotorSpeedDic(ModbusID.MOTOR_H.value,True,ALMOST_ZEROINT,decc_current_h,decc_current_h)
            SendCMD_Device([cmdSpdChangeSrv])
        elif key_motorMainRotate in lsMotorsActivate:
            StopAllMotors(decc=DECC_MOVE_H)
        else:
            #전개나 하강중이거나 회전중이면 정지하고 폴드한다
            StopAllMotors(decc=ACC_DECC_SMOOTH)
            #spd_v = int(GetItemsFromModbusTable(ModbusID.MOTOR_V,MonitoringField.CUR_SPD))
            #spd_srvArm = int(GetItemsFromModbusTable(ModbusID.TELE_SERV_MAIN,MonitoringField.CUR_SPD))
            # spd_540 = int(GetItemsFromModbusTable(ModbusID.ROTATE_MAIN_540,MonitoringField.CUR_SPD))
            # MotorV= isActivatedMotor(ModbusID.MOTOR_V.value)
            # TELE_SERV_MAIN= isActivatedMotor(ModbusID.TELE_SERV_MAIN.value)
            # ROTATE_MAIN_540= isActivatedMotor(ModbusID.ROTATE_MAIN_540.value)
            #if MotorV or TELE_SERV_MAIN or ROTATE_MAIN_540:
            # if spd_v > 0 or spd_srvArm > 0:
            #     if isLiftTrayDownFinished():
            #         DoorClose()
            #     else:
            lsLiftUp = GetLiftControl(True)
            lsBLBTmp = copy.deepcopy(node_CtlCenter_globals.listBLB)
            node_CtlCenter_globals.listBLB[:] = lsLiftUp + lsBLBTmp
        
        if IsEnableSvrPath() and dfReceived is not None:
            #지시 상태 필드 업데이트, Resume 시에는 저 부분부터 다시 하면 됨.
            dfReceived.loc[dfReceived[key_detailcode] == iDetailcode_running, APIBLB_FIELDS_NAVI.workstatus.name] = APIBLB_STATUS_TASK.Paused.value
            SetWaitConfirmFlag(True,AlarmCodeList.WAITING_USER)
            PrintDF(dfReceived)
            resultAPI, nodeReturn = API_robot_navigation_info(dfReceived,APIBLB_STATUS_TASK.Paused)
            #SetTableTarget(HOME_CHARGE)
            if nodeReturn is None:
                rospy.loginfo(resultAPI)
            else:
                SetCurrentNode(nodeReturn)
            # DataFrame을 리스트로 변환
            data_list = dfReceived.to_dict(orient="records")
            # 리스트를 JSON 문자열로 변환
            json_string = json.dumps(data_list, ensure_ascii=False)
            pub_DF.publish(json_string)
        SendInfoHTTP(ALM_User.JOB_PAUSED.value)
        TTSAndroid("일시중지 명령어가 수신되었습니다.")
        if node_CtlCenter_globals.robot.get_current_state() == Robot_Status.onServing:
            node_CtlCenter_globals.robot.trigger_pause_serving()
        else:
            TTSAndroid('현재 상태는 중지가 아닙니다')
            return dfReceived, APIBLB_ACTION_REPLY.R103                
        return dfReceived,APIBLB_ACTION_REPLY.R101
    except Exception as e:
      rospy.loginfo(e)
      sMsg = traceback.format_exc()
      SendAlarmHTTP(sMsg,True,node_CtlCenter_globals.BLB_ANDROID_IP)
      return pd.DataFrame(), APIBLB_ACTION_REPLY.R102

# def CancelJob():
#     try:
#         StopMotor(ModbusID.MOTOR_H.value, DECC_MOVE_H)
#         dfReceived, bApiReturn = SetPauseState()
#         curTargetTable,curTarNode = GetTargetTableNode()
#         if dfReceived is None or dfReceived.empty:
#             dfReceived = GetDF(curTargetTable)
#         # if not IsSuspendedJob():
#         #     dfReceived, bApiReturn = SetPauseState()            
#         if dfReceived is not None and not dfReceived.empty:
#             lsPaused = dfReceived[dfReceived[APIBLB_FIELDS_NAVI.workstatus.name] == APIBLB_STATUS_TASK.Paused.value].tail(1).to_dict(orient='records')
#             lnTables = 0 if GetTableList() == None else len(GetTableList())
            
#             if len(lsPaused) > 0:
#                 dicPaused = lsPaused[0]
#                 #direction = dicPaused[APIBLB_FIELDS_TASK.direction.name]
#                 endNode = dicPaused[APIBLB_FIELDS_TASK.endnode.name]
#                 #iDetailcode = dicPaused[key_detailcode]
#                 taskid = dicPaused[APIBLB_FIELDS_TASK.taskid.name]
            
#                 if str(endNode).startswith('T'):
#                     # min_notStarted_detailcode = dfReceived[dfReceived[key_detailcode] < iDetailcode][key_detailcode].max()
#                     # direction = dfReceived[dfReceived[key_detailcode] == min_notStarted_detailcode][APIBLB_FIELDS_TASK.direction.name].iloc[0]
#                     endNode = dicPaused[APIBLB_FIELDS_TASK.startnode.name]
                    
#                 PrintDF(dfReceived)
#                 resultAPI, node_CtlCenter_globals.node_current = API_robot_navigation_info(dfReceived,APIBLB_STATUS_TASK.Canceled)
#                 SetTaskCompleted(taskid,2)
#                 rospy.loginfo(f'endNode:{endNode}, currNode:{node_CtlCenter_globals.node_current}')
#                 if len(node_CtlCenter_globals.lsHistory_motorH) > 1:
#                     node_CtlCenter_globals.lsHistory_motorH[-1][SeqMapField.END_NODE.name]=node_CtlCenter_globals.node_current
#                 API_SetCurrentNode(nodeID=endNode,taskid=taskid)
#                 node_CtlCenter_globals.listBLB.clear()
#                 node_CtlCenter_globals.dicTargetPosFeedBack
#             else:
#                 print('Pause 없이 캔슬되면 에러')      
#             SetWaitConfirmFlag(False,AlarmCodeList.OK)
#             RemoveDF()
#             AppendSendStatus(BLB_STATUS_FIELD.CANCELLED)
#             ReloadSvrTaskList()
#             return APIBLB_ACTION_REPLY.R101
#         else:
#             return APIBLB_ACTION_REPLY.R102      
#     except Exception as e:
#       rospy.loginfo(e)
#       return APIBLB_ACTION_REPLY.R102

def CancelJob():
    SendInfoHTTP(log_all_frames('작업취소'))
    blbState = node_CtlCenter_globals.robot.get_current_state()
    if blbState != Robot_Status.paused:
        TTSAndroid("작업중지를 먼저 해주세요.")
        return    
    
    SetWaitConfirmFlag(True,AlarmCodeList.JOB_CANCEL)
    StopMotor(ModbusID.MOTOR_H.value,EMERGENCY_DECC)
    #StopAllMotors(decc=ACC_DECC_SMOOTH)
    curTargetTable,curTarNode = GetCurrentTargetTable()
    dfReceived = GetDF(curTargetTable)
    if dfReceived is None:
      return APIBLB_ACTION_REPLY.R102
    
    curNode = GetCurrentNode()
    #pos_abs = dict(zip(dfReceived[APIBLB_FIELDS_TASK.POS_ABS.name], dfReceived[APIBLB_FIELDS_TASK.endnode.name]))  
    dicLast = dfReceived.iloc[-1]
    taskID = int(dicLast[APIBLB_FIELDS_TASK.taskid.name])
    tableTarget = (dicLast[APIBLB_FIELDS_TASK.endnode.name])
    resultAPI, nodeReturn = API_robot_navigation_info(dfReceived,APIBLB_STATUS_TASK.Canceled)
    
    # if nodeReturn is None:
    #     print(resultAPI)
    # else:
    #     SetCurrentNode(curNode)
    # DataFrame을 리스트로 변환
    data_list = dfReceived.to_dict(orient="records")
    # 리스트를 JSON 문자열로 변환
    json_string = json.dumps(data_list, ensure_ascii=False)
    pub_DF.publish(json_string)
    SetTaskCompleted(taskID)
    #RemoveDF(tableTarget)
    node_CtlCenter_globals.dicTargetPosFeedBack.clear()
    node_CtlCenter_globals.listBLB.clear()
    node_CtlCenter_globals.listTable.clear()
    logmsg = ALM_User.JOB_CANCELED.value
    # blbState = node_CtlCenter_globals.robot.get_current_state()
    # if blbState == Robot_Status.paused:
    #     node_CtlCenter_globals.robot.trigger_resume_serving()
    # else:
    #     SendAlarmHTTP(f"서빙상태 정보가 맞지 않습니다1.{blbState}",True,node_CtlCenter_globals.BLB_ANDROID_IP)

    # blbState = node_CtlCenter_globals.robot.get_current_state()
    # if blbState == Robot_Status.onServing:
    #     node_CtlCenter_globals.robot.trigger_complete_serving()
    # else:
    #     SendAlarmHTTP(f"서빙상태 정보가 맞지 않습니다2.{blbState}",True,node_CtlCenter_globals.BLB_ANDROID_IP)

    recvData = f'Stopped by Home call from server at node {curNode},{resultAPI}'
    logmsg = f"State:{node_CtlCenter_globals.robot.get_current_state()},{recvData} from {sys._getframe(0).f_code.co_name} - {sys._getframe(1).f_code.co_name}"
    #SendAlarmHTTP(logmsg,True,node_CtlCenter_globals.BLB_ANDROID_IP)
    StopAllMotors(DECC_MOVE_H)
    #rospy.loginfo(f'Job Canceled at node {GetCurrentNode()},{resultAPI}')
    SendInfoHTTP(logmsg)
    TTSAndroid("취소 명령어가 수신되었습니다.")
    SetTableTarget(HOME_TABLE)
    SetWaitConfirmFlag(False,AlarmCodeList.OK)    
    return APIBLB_ACTION_REPLY.R101
  
def StopEmergency(strAlarmMsg = '-1:알람발생!', isStopMotor = True):
    node_CtlCenter_globals.listBLB.clear()
    node_CtlCenter_globals.dicTargetPos.clear()
    node_CtlCenter_globals.dicTargetPosFeedBack.clear()
    
    key, value = strAlarmMsg.split(':', 1)
    # key와 value의 앞뒤 공백 제거
    key = key.strip()
    value = value.strip()
    key=get_timestampNow()
    #alarmMsg = f'{key}={value}'
    SetWaitConfirmFlag(True, {key:value})
    #node_CtlCenter_globals.listTable.clear()
    if isStopMotor:
        StopAllMotors(ACC_DECC_SMOOTH)
    DoorStop()
    logmsg = f"{strAlarmMsg}:{sys._getframe(1).f_code.co_name}:{getCurrentTime()}"
    SendMsgToMQTT(pub_topic2mqtt,MQTT_TOPIC_VALUE.BLB_ALARM.value,logmsg)
    API_SendAlarm(ALM_User.USER_ESTOP)
    SendAlarmHTTP(value,True,node_CtlCenter_globals.BLB_ANDROID_IP)
    SetPauseState()
    
def GetNewRotateArmList(dicAruco, ignoreArm=False):
    lsMotorOperationNew = []
    isNewMarkerisBetter = compare_better_marker(node_CtlCenter_globals.dicARUCO_last, dicAruco)
    if not isNewMarkerisBetter:
        return []
    if node_CtlCenter_globals.dicARUCO_last:
        return []
        tsOld = datetime.fromtimestamp(node_CtlCenter_globals.dicARUCO_last[ARUCO_RESULT_FIELD.LASTSEEN.name])
        tsNew = datetime.fromtimestamp(dicAruco[ARUCO_RESULT_FIELD.LASTSEEN.name])
        td = tsNew - tsOld 
        if td.total_seconds() < 10:
            return []
    
    node_CtlCenter_globals.dicARUCO_last.update(dicAruco)
    #rospy.loginfo(json.dumps(dicAruco, indent=4))
    curDistanceSrvTele, curAngle_540,cur_angle_360  = GetCurrentPosDistanceAngle()
    dISTANCE_delta_mm,angle_delta_degree = compute_distance_and_rotation_from_dict(ref_dict, dicAruco)
    marker_angle = (180+dicAruco[ARUCO_RESULT_FIELD.ANGLE.name])%360
    print(dISTANCE_delta_mm,angle_delta_degree)
    angle_cal = (curAngle_540 + angle_delta_degree) % 360
    dISTANCE_delta_mm_real = ConvertArucoSizeToReal(dISTANCE_delta_mm)
    curPosX,curPosY = calculate_coordinates(curDistanceSrvTele,curAngle_540)
    resultDiff,diff_X,diff_Y= compare_dicts(dicAruco, ref_dict, CAM_LOCATION_MARGIN_OK)
    newPosX,newPosY = curPosX+diff_Y,curPosY+diff_Y
    distance_new_mm_real, angle_new_degrees = calculate_distance_and_angle(newPosX, newPosY)
    #distance_final_mm_real =dISTANCE_delta_mm_real+curDistanceSrvTele
    #newPosX,newPosY = calculate_coordinates(dISTANCE_delta_mm_real,angle_cal)
    # distance_new_pulse = GetTargetPulseServingArm(distance_final_mm_real)
    # rt540pulse= GetRotateMainPulseFromAngle(angle_cal)
    rospy.loginfo(format_vars(resultDiff,diff_X,diff_Y))
    rospy.loginfo(f'현재좌표:{curPosX,curPosY},좌표마진:{diff_X,diff_Y},신규좌표:{newPosX,newPosY},최종각도:{distance_new_mm_real, angle_new_degrees},트레이각도:{marker_angle}')
    # rospy.loginfo(f'추가길이:{dISTANCE_delta_mm_real},추가각도:{angle_delta_degree},최종길이:{distance_final_mm_real},{distance_new_pulse},최종각도:{angle_cal},{rt540pulse},트레이각도:{marker_angle}')
    #CAMERA_DISTANCE_FROM_CENTER = ConvertRealToArucoSize(0.32)
    rospy.loginfo(f'카메라보정:{get_camera_offset(CAMERA_DISTANCE_FROM_CENTER,marker_angle)}')
    # lsArmBeTunedFromOrigin = GetStrArmExtendMain(distance_new_mm,angle_cal,False)
    # lsArmBeTunedFromCurrent = GetStrArmExtendMain(distance_new_mm,angle_cal,True)
    # print(lsArmBeTunedFromCurrent)
    # lsArmBeFold = GetStrArmExtendMain(0,0,True)
    # dicRotateNewVerySlow = GetDicRotateMotorMain(angle_cal,MAINROTATE_RPM_SLOWEST,False)
    # dicRotateNewNormal = GetDicRotateMotorMain(angle_cal)
    # rotateTime = dicRotateNewVerySlow.get(MotorWMOVEParams.TIME.name, MIN_INT)
    # if (distance_new_mm > curDistanceSrvTele and ROTATE_ANGLE <= 15) or ignoreArm:
    #     lsMotorOperationNew.append([dicRotateNewVerySlow])
    #     lsMotorOperationNew.append(lsArmBeTunedFromCurrent)
    # else:
    #     lsMotorOperationNew.append(lsArmBeFold)
    #     lsMotorOperationNew.append([dicRotateNewNormal])
    #     lsMotorOperationNew.append(lsArmBeTunedFromOrigin)
    
    # # lsNewArmRotate = GetStrArmExtendMain(distance_new_mm,angle_cal,True)
    # # dicNewRotate = GetDicRotateMotorMain(angle_cal, rotateRPM=SPD_540, isTargetPulse=False)                
    # #rospy.loginfo(f'타겟암:{lsNewArmRotate},타겟회전:{dicNewRotate}')
    
    # # marker_angle = node_CtlCenter_globals.dicARUCO_last[ARUCO_RESULT_FIELD.ANGLE.name]
    # #rospy.loginfo(f'현재각도:{curAngle_540},감지각도:{angle_cal},서빙암길이:{distance_new_mm}')
    # dicRotate360_down = GetDicRotateMotorTray(marker_angle,SPD_360,ACC_360_DOWN, DECC_360_DOWN)
    # lsRotate360_down = [dicRotate360_down]
    # lsLiftDown = GetListLiftDown(200000)
    # lsRotate360_down.extend(lsLiftDown)                     
    # lsMotorOperationNew.append(lsRotate360_down)
    #TTSAndroid('위치를 수정합니다.')
    return lsMotorOperationNew

def calculate_robot_translation_aruco(dicAruco):
    GetNewRotateArmList(dicAruco)
    lsMotorOperationNew=[]
    current_x = dicAruco['X']
    current_y = dicAruco['Y']
    current_marker_angle = dicAruco['ANGLE']
    target_x = ref_dict['X']
    target_y = ref_dict['Y']
    target_marker_angle = ref_dict['ANGLE']
    marker_diff = target_marker_angle - current_marker_angle
    curDistanceSrvTele, curAngle_540,cur_angle_360  = GetCurrentPosDistanceAngle()
    diffX,diffY=calculate_robot_translation(current_x,current_y,marker_diff,target_x,target_y)
    diffx_meter = ConvertArucoSizeToReal(diffX)
    diffy_meter = -ConvertArucoSizeToReal(diffY)
    #distanceServingTeleTotal, angle_degrees =calculate_relative_extension_and_rotation(diffx_meter,diffy_meter,curDistanceSrvTele,0)
    distanceServingTeleTotal,angle_degrees= calculate_coordinates(curDistanceSrvTele,cur_angle_360, diffx_meter,diffy_meter)
    
    #distanceServingTeleTotal, angle_degrees =calculate_robot_movement(diffx_meter,diffy_meter,curDistanceSrvTele,0)        
    # distanceServingTeleTotal, angle_degrees = calculate_distance_and_angle(diffx_meter,diffy_meter)
    curX,curY = calculate_coordinates(curDistanceSrvTele,curAngle_540)
    newX = curX + diffx_meter
    newY = curY + diffy_meter
    distanceFinal, angle_degrees_final = calculate_distance_and_angle(newX, newY)
    print(f'현재위치XY:{curX,curY},XY보정마진:{diffx_meter,diffy_meter},타겟XY:{newX,newY}')
    print(f'마커각도:{current_marker_angle:.1f},마커각마진:{marker_diff:.1f},현재위치각:{curDistanceSrvTele,curAngle_540},타겟위치각:{distanceFinal,angle_degrees_final}')
    lsMotorOperationNew.append([GetDicRotateMotorMain(angle_degrees_final)])
    lsMotorOperationNew.append(GetStrArmExtendMain(distanceFinal,angle_degrees_final, True))
    lsMotorOperationNew.append([GetDicRotateMotorTray(current_marker_angle)])
    return []
    return 

def calculate_robot_translation2(dicAruco):
    curDistanceSrvTele, curAngle_540,cur_angle_360  = GetCurrentPosDistanceAngle()
    angle = dicAruco['ANGLE']
    curX,curY = calculate_coordinates(curDistanceSrvTele,curAngle_540)
    diffX, diffY = calculate_position_shift(angle, marker_coords_goldsample)
    diffx_meter = ConvertArucoSizeToReal(diffX)
    diffy_meter = ConvertArucoSizeToReal(diffY)    
    newX = curX + diffx_meter
    newY = curY + diffy_meter
    distanceFinal, angle_degrees_final = calculate_distance_and_angle(newX, newY)
    print(f'현재위치XY:{curX,curY},XY보정마진:{diffx_meter,diffy_meter},타겟XY:{newX,newY}')
    print(f'마커각도:{angle:.1f},마커각마진:{cur_angle_360-angle:.1f},현재위치각:{curDistanceSrvTele,curAngle_540},타겟위치각:{distanceFinal,angle_degrees_final}')
    return []
    return lsMotorOperationNew

print(os.path.splitext(os.path.basename(__file__))[0],getDateTime())