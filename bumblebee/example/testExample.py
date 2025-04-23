#! /usr/bin/env python3
import os
from Util import *
from UtilGPIO import *
import pandas as pd
import numpy as np
import copy

dirPath = os.path.dirname(__file__)
filePath_rosparam = f'{dirPath}/param_ros.yaml'
filePath_ModbusSetupHoming = f'{dirPath}/SERVO_Homing.txt'
filePath_ModbusSetupHomingTest = f'{dirPath}/SERVO_HomingTest.txt'
filePath_ModbusSetupInit = f'{dirPath}/SERVO_Init.txt'
filePath_ModbusAlarm = f'{dirPath}/SERVO_ALARM.txt'
filePath_map_default = f'{dirPath}/MAPPING_RFID.txt'
filePath_mapSPD_default = f'{dirPath}/MAPPING_SPD.txt'
filePath_param_parse = f'{dirPath}/param_parser.txt'
filePath_param_default = f'{dirPath}/param_default.txt'
filePath_CMD_RECONNECT = f'{dirPath}/CMD_RECONNECT.txt'
filePath_CMD_GETHOME = f'{dirPath}/CMD_GETHOME.txt'

filePath_CMD_TESTCLASS = f'{dirPath}/CMD_TESTCLASS.txt'
filePath_CMD_TESTMAIN = f'{dirPath}/CMD_TESTMAIN.txt'

tmpData = getStr_FromFile(filePath_CMD_TESTMAIN)          
curMap =str2frame(tmpData,'\t')

tmpData2 = getStr_FromFile(filePath_CMD_TESTCLASS)          
curMap2 =str2frame(tmpData2,'\t')
curMap3 = pd.merge(left = curMap , right = curMap2, how = "inner", on = "PROFILE")
lenMap1 = len(curMap)
lenMap2 = len(curMap2)
lenMap3 = len(curMap3)
if lenMap1 != lenMap3:
    print(f'프로필이 정의되지 않았습니다 {lenMap1},{lenMap2},{lenMap3}')
    print(curMap)
    print(curMap2)
    print(curMap3)
    
#curMap.append(curMap2, sort=False)
curMap3.dropna(subset=[])
#curMap3.reset_index(inplace=True)
# print(curMap)
# print(curMap2)
df_fromSPGMAP=curMap3.sort_values(by='LASTSEEN', ascending=True)
df_fromSPGMAP.reset_index(inplace=True)
print(df_fromSPGMAP)
currOperationPos = 0

lsDicArray = []
iPrevPos = 0

while currOperationPos != len(df_fromSPGMAP):
    '''
    지시정보를 분석 후 새로 생성해서 리턴.
    생성할 필드 LASTSEEN,START,HEIGHT,HEIGHT,SCAN_3D,MULTI_CAM,SPD_H,MOVE_,TIER,LENGTH,ANGLE,ACC,DECC
    해서 리턴.
    '''
    dicTmp = {} #필드값을 map 형태로 저장할 임시변수

    #1 - 송신된 지시정보 필드값 수집.
    sLastSeen = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.LASTSEEN.name)
    sStart = str(df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.START.name))
    iLENGTH = int(df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.LENGTH.name))
    sProfile = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.PROFILE.name)
    sHEIGHT = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.HEIGHT.name)
    sTier = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.TIER.name)
    sSPD_H = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.SPD_H.name)
    sSPD_D = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.SPD_D.name)
    sSPD_U = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.SPD_U.name)

    sACC_H = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.ACC_H.name)
    sACC_V = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.ACC_V.name)
    sDECC_H = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.DECC_H.name)
    sDECC_V = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.DECC_V.name)

    sACT_H = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.ACT_H.name)
    sACT_D = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.ACT_D.name)
    sACT_U = df_fromSPGMAP.loc[currOperationPos].get(SPG_Profile.ACT_U.name)
    

    #2 - 지시정보 해석하여 새로운 지시정보 생성, 수평운동 map 개체 생성 후 복사하여 수직운동 속성에도 반영.
    # LASTSEEN	START  SCAN_3D	MULTI_CAM	SPD_H	MOVE_	TIER	LENGTH	ANGLE	ACC	DECC
    # 형태로 생성되어야 함.
    
    #2-1 수평운동 지시정보부터 생성
    dicTmp[SPG_Keys.LASTSEEN.name] = f'{sLastSeen}00'
    dicTmp[SPG_Keys.START.name] = sStart
    dicTmp[SPG_Keys.SPD_H.name] = sSPD_H
    dicTmp[SPG_Keys.TIER.name] = sTier    
    dicTmp[SPG_Keys.LENGTH.name] = iLENGTH    
    dicTmp[SPG_Keys.ACC.name] = sACC_H
    dicTmp[SPG_Keys.DECC.name] = sDECC_H
    dicTmp[SPG_Keys.ACT.name] = sACT_H
    dicTmp[SPG_Profile.PROFILE.name] = sProfile
    
    if iLENGTH < iPrevPos:
        dicTmp[SPG_Keys.MOVE_.name] = 'L'
    else:
        dicTmp[SPG_Keys.MOVE_.name] = 'R'
    lsDicArray.append(dicTmp)
    
    currOperationPos +=1    
    if sHEIGHT == '0':
        continue
    
    iPrevPosV = 0
    #2-2 하강운동 지시정보 파싱
    lsHeight = sHEIGHT.split(sep='/')
    iVCnt = 0
    for curHeight in lsHeight:
        iVCnt +=1
        dicTmpD = copy.deepcopy(dicTmp)
        padLeftStr = str(iVCnt).rjust(2,'0')
        dicTmpD[SPG_Keys.LASTSEEN.name] = f'{sLastSeen}{padLeftStr}'
        dicTmpD[SPG_Keys.TIER.name] = iVCnt    
        dicTmpD[SPG_Keys.LENGTH.name] = curHeight    
        dicTmpD[SPG_Keys.ACC.name] = sACC_V
        dicTmpD[SPG_Keys.DECC.name] = sDECC_V
        dicTmpD[SPG_Keys.ACT.name] = sACT_D
        dicTmpD[SPG_Keys.MOVE_.name] = 'D'
        dicTmpD[SPG_Keys.SPD_H.name] = sSPD_D
        lsDicArray.append(dicTmpD)

    #2-3 상승운동 지시정보 파싱
    dicTmpU = copy.deepcopy(dicTmp)
    dicTmpU[SPG_Keys.LASTSEEN.name] = f'{sLastSeen}99'
    dicTmpU[SPG_Keys.SPD_H.name] = sSPD_U
    dicTmpU[SPG_Keys.TIER.name] = 0
    dicTmpU[SPG_Keys.LENGTH.name] = 6000 #맨 위로 올린다
    dicTmpU[SPG_Keys.ACC.name] = sACC_V
    dicTmpU[SPG_Keys.DECC.name] = sDECC_V
    dicTmpU[SPG_Keys.ACT.name] = sACT_U
    dicTmpU[SPG_Keys.MOVE_.name] = 'U'
    dicTmpU[SPG_Keys.SPD_H.name] = sSPD_U
    lsDicArray.append(dicTmpU)
    

df = pd.DataFrame(lsDicArray)
print(df)