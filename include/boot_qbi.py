import http.client as httplib
import time

# from time import sleep  #time 라이브러리의 sleep함수 사용
import os
import sys

# import rosnode
from get_nic import getnic
import subprocess

# import re
# import platform
import netifaces
import rospy
from Util import *

fileno = sys.stdout.fileno()
ttyResult = os.ttyname(fileno)
print(f"{fileno} - {ttyResult}")
strArray = ttyResult.split("/")
sCD = os.path.dirname(__file__)
os.chdir(sCD)
print(os.getcwd())
netCnt = 0
interfaces = None
bNetworkEnable = False
while not bNetworkEnable:
    print(f"Checking the NIC : {netCnt}")
    try:
        interfaces = netifaces.gateways()["default"][netifaces.AF_INET][0]
        print(f"{netCnt}-{interfaces}")
        bNetworkEnable = True
    except Exception as e:
        print(f"Exception on checking the NIC : {netCnt}-{e}")
        time.sleep(1)
    netCnt += 1


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
    global interfaces
    # urlTarget = "172.30.1.254"
    urlTarget = interfaces
    msg = f"Check internet : {urlTarget}"
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
            print(f"Try {iCntRep} : {response}")
            if "wlan0" in response:
                bBreak = True
            else:
                time.sleep(1)
                if iCntRep > 20 and "wlan0" in response:
                    bBreak = True
        except:
            pass
    return bBreak


if len(strArray) == 4:
    sDev = strArray[1]
    sPTS = strArray[2]
    sTTY = strArray[3]
    print(f"Dev = {sDev}, sTTY = {sTTY}, sPTS = {sPTS}")
    # if sDev == 'dev' and sTTY == '1':
    #     # scrPath = f'{sCD}/touch2.sh'
    #     # #scrPath = 'xdotool keydown Control keydown Alt key t keyup Control keyup Alt'
    #     # print(f'{os.path.isfile(scrPath)} : {scrPath} Run touch script #2')
    #     # os.system(scrPath + ' -&')
    #     scrPath = f'~/run_gui.sh'
    #     print(f'{os.path.isfile(scrPath)} : {scrPath} Run XTE script after somewhile2')
    #     os.system(scrPath)

    if sDev == "dev" and sTTY == "0":
        # time.sleep(15)
        # scrPath = f"{sCD}/external_display.sh"
        # # result = subprocess.check_output (scrPath , shell=True)
        # os.system(scrPath)
        # print(result)
        while rosResult is False:
            try:
                iCntRep += 1
                # rosResult = have_internet()
                # chk = rospy.get_master().getSystemState()
                # chk = have_internet()
                # chk = rosnode.get_node_names()
                # print(chk)
                # rosResult = True
                time.sleep(1)
                rosResult = check_ros_master()
                print(
                    f"Init ROS node - {iCntRep}, Dev = {sDev}, sTTY = {sTTY}, ROS Connection = {rosResult}"
                )
            except Exception as e:
                # if iCntRep == 10:
                # wait_for_internet_connection()
                #     #os.system('roslaunch rosbridge_server rosbridge_websocket.launch -&')
                # print(traceback.format_exc())
                rosResult = False
                print(f"Try {iCntRep} - {e}")
                time.sleep(1)
        # scrPath = f'{sCD}/touch.sh'
        # print(f'{os.path.isfile(scrPath)} : {scrPath} Run touch script #1.')
        # os.system(scrPath+ ' -&')
        # time.sleep(10)

        # scrPath = f"{sCD}/external_display.sh"
        # print(f"{os.path.isfile(scrPath)} : {scrPath} Run video script after somewhile")
        # os.system(scrPath + " -&")

        #scrPath = f"{sCD}/.rr -&"
        scrPath = f"{sCD}/.rrStartqbi -&"
        # scrPath = '/root/.rrStartReal -&'
        print(f"{os.path.isfile(scrPath)} : {scrPath} Run ros script")
        os.system(scrPath + " -&")

        # scrPath = f"{sCD}/run_gui.sh"
        # print(f"{os.path.isfile(scrPath)} : {scrPath} Run UI after somewhile1")
        # os.system(scrPath + " -&")
        # time.sleep(10)

        # print(f'{os.path.isfile(scrPath)} : {scrPath} Run OK and sleep')

    # else:
    #     filePath_killprocess = f'{sCD}/.kill -&'
    #     os.system(filePath_killprocess)


# sCmd = 'python3 /root/catkin_ws/src/actionlib_tutorials/SPG_server_3060.py -&'
# sCmd = 'python3 SPG_server_3060.py -&'
