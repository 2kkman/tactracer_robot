import http.client as httplib
import time
from time import sleep  #time 라이브러리의 sleep함수 사용
import os
import sys
import rosnode
from get_nic import getnic
import subprocess
import re
import platform
import netifaces

fileno = sys.stdout.fileno()
ttyResult = os.ttyname(fileno)
print(f'파일순번 : {fileno} - 터미널 정보 : {ttyResult}')

strArray = ttyResult.split("/")
sCD = os.path.dirname(__file__)
os.chdir(sCD)
scrPath = f'{sCD}/.rrStart -&'
print(f'현재 실행 경로 : {os.getcwd()}, 실행스크립트 : {scrPath}')
rosResult = False
iCntRep = 0    
# def system_call(command):
#     p = subprocess.Popen([command], stdout=subprocess.PIPE, shell=True)
#     return p.stdout.read()

# def get_gateway_address():
#     ubuntu18 = "route -n get default | grep 'gateway' | awk '{print $2}'"
#     ubuntu20 = "route | grep default | sed -r 's/\s+/ /g' | cut -f 2 -d ' '"
#     return system_call(ubuntu20)

def have_internet():
    #global interfaces
    #urlTarget = "172.30.1.254"
    wifiCheckCnt = 0
    interfaces = ''
    wifiCheckCnt_Limit = 30
    while wifiCheckCnt < wifiCheckCnt_Limit:
        try:
            interfaces = netifaces.gateways()['default'][netifaces.AF_INET][0]
            wifiCheckCnt = wifiCheckCnt_Limit
            time.sleep(1)
        except Exception as e:
            print(e)
            wifiCheckCnt += 1
    print(interfaces)
    urlTarget = interfaces
    msg = f'Check internet : {urlTarget}'
    print(msg)
    conn = httplib.HTTPConnection(urlTarget, timeout=5)
    try:
        conn.request("HEAD", "/")
        conn.close()
        return True
    except:
        conn.close()
        return False

def wait_for_internet_connection():
    global iCntRep
    bBreak = False
    while not bBreak:
        try:
            response = getnic.interfaces()
            print(f'Try {iCntRep} : {response}')
            if 'wlan0' in response:
                bBreak = True
            else:
                time.sleep(1)
                if iCntRep > 20 and 'wlan0' in response:
                    bBreak = True
        except:
            pass
    return bBreak

if len(strArray) == 4:
    sDev = strArray[1]
    sPTS = strArray[2]
    sTTY = strArray[3]
    
    #현재 장치 자체에서 실행되고 있고 가장 첫번째 열린 터미널 창인경우 ( sTTY == 0)
    #즉 데스크톱에서 바로 열린 경우.
    if sDev == 'dev' and sTTY == '0': 
        while rosResult is False:
            try:
                iCntRep+=1
                rosResult = have_internet()
                print(f'Init ROS node - {iCntRep}, Dev = {sDev}, sTTY = {sTTY}, internet Connection = {rosResult}')
                # chk = rosnode.get_node_names()
                # print(chk)
                # rosResult = True
            except Exception as e:
                #if iCntRep == 10:
                    #wait_for_internet_connection()
                #     #os.system('roslaunch rosbridge_server rosbridge_websocket.launch -&')
                #print(traceback.format_exc())
                print(e)
            time.sleep(1)
        #scrPath = f'{sCD}/external_display.sh'
        #os.system(scrPath)
        scrPath = f'{sCD}/.rr -&'
        #scrPath = f'{sCD}/.rrDev -&'
        
        #scrPath = f'{sCD}/.rrStart -&'
        print(f'Run script : {scrPath}')
        os.system(scrPath)
    # else:
    #     filePath_killprocess = f'{sCD}/.kill -&'
    #     os.system(filePath_killprocess)
        


#sCmd = 'python3 /root/catkin_ws/src/actionlib_tutorials/SPG_server_3060.py -&'
#sCmd = 'python3 SPG_server_3060.py -&'
