import os
import sys
import json

from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtGui import *
from PyQt5.QtCore import *

import rospy
from std_msgs.msg import String, Time
from importlib import reload
from ui.KeypadDialog import KeypadDialog
from ui.mainwindow_ui import Ui_MainWindow
from ui.moving import Ui_Form
from datetime import datetime

import threading
from dataclasses import dataclass
import re

chSplit1 = ':'
chSplit2 = ','
chSplit3 = '`'

prev_msg = None

def resource_path(relative_path):
    base_path = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base_path, relative_path)

# form = resource_path('ui/mainwindow.ui')
# form_class = uic.loadUiType(form)[0]

# form_second = resource_path('ui/moving.ui')
# form_secondwindow = uic.loadUiType(form_second)[0]

msgfont = QFont('Noto Sans Mono CJK KR', 16)
DeviceID = 'Bumblebee1'

class WindowClass(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        self.count = 0
        self.maximumLevel = 3
        self.minimumLevel = -3
        self.floor_level = 0
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.resetCount)
        self.timer.setSingleShot(True)
        
    def resetCount(self):
        self.count = 0

    def mousePressEvent(self, e):
        self.count += 1
        if self.count >= 10:
            qApp.quit()
        self.timer.start(500) # 500 milliseconds

    def evtmousePress(self, e):
        self.count += 1
        if self.count >= 7:
            qApp.quit()
        self.timer.start(500) # 500 milliseconds

    def evt_floor_default(self):
        self.floor_level = 0
        self.floor_label.setText(f"{self.floor_level}")

    def evt_floor_up(self):
        self.floor_level += 1 
        self.floor_level = min(self.floor_level, self.maximumLevel)
        self.floor_label.setText(f"{self.floor_level}")
        
    def evt_floor_down(self):
        self.floor_level -= 1 
        self.floor_level = max(self.floor_level, self.minimumLevel)
        self.floor_label.setText(f"{self.floor_level}")        
        
    def btn_main_to_movewindow(self):
        self.hide()                     # 메인윈도우 숨김
        self.second = secondwindow()    #
        self.second.exec()              # 두번째 창을 닫을 때 까지 기다림
        self.showFullScreen()     
        
    def evt_send_message(self):
        global sResult
        global send_data
        global pub
        
        pub = rospy.Publisher('BLB_CMD', String, queue_size=10)

        # print(f"floor_level_value : {floor_level_value}")
        
        # sResult = 'ID' + chSplit1 + 'BumbleBee1'+ chSplit3 + 'TRAY_A' + chSplit1 + self.spinbox1.text() + chSplit3 + 'TRAY_B' + chSplit1 + self.spinbox2.text() + chSplit3 +'FLOOR_LEVEL' + chSplit1 + str(floor_level_value) + chSplit3 + 'STATE' + chSplit1 + 'move1' + chSplit3 + 'TIME' + chSplit1 + getTimestamp()

        sResult = {'ID':DeviceID}                           #DEVICE ID
        sResult['TRAY_A'] = self.spinbox1.text()            #TABLE NO
        sResult['TRAY_B'] = self.spinbox2.text()            #TABLE NO
        sResult['FLOOR_LEVEL'] = self.floor_label.text()    #FLOOR LEVEL
        sResult['STATE'] = 'READY'                          #STATE
        sResult['TIME'] = getTimestamp()                    #TIMESTAMP
        
        
        # print(f"sResult : {sResult}")
        for key, value in sResult.items():
            print(f'{key} : {value}')
        # print(f'sResult : {type(sResult)}')
        
        ret = show_custom_messagebox()
        if ret:
            self.spinbox1.setValue(0)
            self.spinbox2.setValue(0)
            self.floor_level = 0
            self.floor_label.setText(f"{self.floor_level}")
            self.btn_main_to_movewindow()
        else:
            pass
        
class secondwindow(QDialog,QWidget, Ui_Form):
    def __init__(self, parent=None):
        super(secondwindow,self).__init__(parent)
        global send_data
        global table_Text
        global sResult
        global received_message

        table_Text = ''
        received_message = ''
        
        self.parent = parent
        self.initUi()
        self.showFullScreen()
        # self.table_label.setText(f'이동중 ...')
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.close)
        self.timer.stop()
        
        self.btnTable.setVisible(True)
        self.lblImage.setVisible(True)        
        self.btnConfirm.setVisible(False)     
        # send_data = String(f'DeliveryTable:{sResult}')
        
        # self.btnTable.clicked.connect(self.btnKeypad)
        self.btnConfirm.clicked.connect(self.btn_return)
        # pub.publish(send_data)
        
        # print(f"BLB_STATUS.ID:{type(blb_status.ID)}")
        self.btnConfirm.setVisible(True)    
        
        if blb_status.ID == DeviceID:
            # print(f"BLB_STATUS.STATUS : {blb_status.STATUS}")
            if blb_status.STATUS == 'READY':
                print(f"BubleBee1 is Ready")
                table_Text = f"준비중 ..."
            elif blb_status.STATUS == 'MOVING':
                print(f"BubleBee1 is Moving")
                table_Text = f"이동중 ..."
            elif blb_status.STATUS == 'LIFTING_DOWN':
                print(f"BubleBee1 is Lifting Down")
                table_Text = f"하강중 ..."
            elif blb_status.STATUS == 'LIFTING_UP':
                print(f"BubleBee1 is Lifting Up")
                table_Text = f"상승중 ..."
            elif blb_status.STATUS == 'WAITING':
                print(f"BubleBee1 is Waiting")
                table_Text = f"대기중 ..."
            elif blb_status.STATUS == 'HOMING':
                print(f"BubleBee1 is Homing")
                table_Text = f"복귀중 ..."
            elif blb_status.STATUS == 'CHARGING':
                print(f"BubleBee1 is Charging")
                table_Text = f"충전중 ..."
            else:
                print(f"BubleBee1 is Unknown")
                table_Text = f"지정되어있는 않은 명령어 ..."
                
            self.table_label.setText(f"{table_Text}")
        else:
            self.close()

        # if sResult['TRAY_A'] == sResult['TRAY_B'] and sResult['STATE'] == 'run':
        #     # getTimestamp_str()
        #     setTimestamp()
        #     sResult['state'] = 'move2'
        #     # send_data = String(f'DeliveryTable:{sResult}')
        #     send_data = String(f'{sResult}')
        #     pub.publish(send_data)
        # # print(f"state : {sResult['state']}")    
        # if sResult['STATE'] == 'run':
        #     table_Text = f"{sResult['TRAY_A']}번 테이블 이동중 ..."
        # elif sResult['STATE'] == 'move2':
        #     table_Text = f"{sResult['TRAY_B']}번 테이블 이동중 ..."
        # elif sResult['STATE'] == 'move3':
        #     table_Text = f"복귀중 ..."
        #     self.btnTable.setVisible(False)
        #     self.lblImage.setVisible(True)        
        #     self.btnConfirm.setVisible(True)                
        # # print(f"table_Text : {table_Text}")       
        # if sResult['STATE'] == 'trayA':      
        #     if int(sResult['TRAY_A']) == input_Table:
        #         self.btnConfirm.setVisible(True)    
        #         self.btnTable.setVisible(False)
        #         self.lblImage.setVisible(False)
        #         # table_Text+=(f"Get to the Table\nThe food you ordered is out\n on the Tray 1")
        #         table_Text+=("트레이 A\n주문하신 메뉴가 나왔습니다\n메뉴를 확인해주세요")

        #         # getTimestamp_str()
        #         sResult['STATE'] = 'move2'
        #         # send_data = String(f'DeliveryTable:{sResult}')
        #     else:
        #         pass

        # elif sResult['STATE'] == 'trayB':
            
        #     if int(sResult['TRAY_B']) == input_Table:
        #         self.btnConfirm.setVisible(True)    
        #         self.btnTable.setVisible(False)
        #         self.lblImage.setVisible(False)        
                
        #         # table_Text+=(f"Get to the Table\nThe foo d you ordered is out\n on the Tray 2")
        #         table_Text+=("트레이 B\n주문하신 메뉴가 나왔습니다\n메뉴를 확인해주세요")
        #         # send_data = String(f'DeliveryTable:{sResult}')
        #     else:
        #         pass
            
        # elif sResult['STATE'] == 'home':
        #     table_Text = f"복귀완료"
        #     self.btnTable.setVisible(False)
        #     self.lblImage.setVisible(False)        
        #     self.btnConfirm.setVisible(True)  
        #     sResult['STATE'] = 'complete' 
        #     setTimestamp()
        #     send_data = String(f'{sResult}')
            
        # pub.publish(send_data)
        # self.table_label.setText(f"{table_Text}")
        
    def initUi(self):
        self.setupUi(self)

    def btn_return(self):
        if blb_status.ID == DeviceID:
            if blb_status.STATUS == 'HOMING':
                self.close()                    #클릭시 종료됨.
            elif blb_status.STATUS == 'READY':  # READY 상태에서 확인버튼 클릭시
                setState('COMPLETE')
                send_data = String(f'{sResult}')
                print(f"send_data : {send_data}")
                pub.publish(send_data)
                self.close()
                self.__init__()
            elif blb_status.STATUS == 'MOVING':
                setState('MOVING')
                send_data = String(f'{sResult}')
                print(f"send_data : {send_data}")
                pub.publish(send_data)
                self.close()
                self.__init__()
            else:
                self.close()
                self.__init__()
        else:
            self.close()

    def timer_second_to_main(self):
        self.close()
        self.__init__()

    def btnKeypad(self):
        global input_Table
        input_Table = ''
        
        self.inputTable = KeypadDialog()
        if self.inputTable.exec_() == QDialog.Accepted: # 두번째 창을 닫을 때 까지 기다림
            input_Table = int(self.inputTable.lineEdit.text())
            # print(f"btnKeypad sResult['state'] : {sResult['state']}")
            
            if sResult['state'] == 'move1':
                sResult['state'] = 'trayA'
                # getTimestamp_str()
            elif sResult['state'] == 'move2':
                sResult['state'] = 'trayB'
            else:
                return
            
            setTimestamp()
            send_data = String(f'{sResult}')
            pub.publish(send_data)
            
            self.btnTable.setVisible(False)

            # print(f"입력된 테이블: {input_Table}")
            self.close()
            self.__init__()

def show_custom_messagebox():
    
    msgDeliveryTable = QMessageBox()
    # # msgDeliveryTable.setStyleSheet("QLabel{min-width:200 px; font-size: 13px;} QPushButton{ width:25px; font-size: 13px; }");
    # # DeliveryTable.setFont(msgfont)
    aTable = sResult['TRAY_A']
    bTable = sResult['TRAY_B']
    msgDeliveryTable.setText(f"A :\t{aTable} \nB :\t{bTable}")
    msgDeliveryTable.setWindowTitle("테이블 배달")
    cBtnYes = msgDeliveryTable.addButton("확인", QMessageBox.ActionRole)
    cBtnNo = msgDeliveryTable.addButton("취소", QMessageBox.ActionRole)
    # msgDeliveryTable.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
    # msgDeliveryTable.setFixedSize(500, 500)
    button_list = msgDeliveryTable.findChildren(QPushButton)
    for button in button_list:
        button.setFixedSize(150, 40)
    msgDeliveryTable.setStyleSheet("QMessageBox{background-color: #FDDE45; border-radius: 5px; font-size: 24px; padding: 10px; text-align: center;} QLabel{min-width: 300px; text-align: center; font-size: 36px;} QPushButton {font-size: 20px;}")
    msgDeliveryTable.exec_()
    if msgDeliveryTable.clickedButton() == cBtnYes:
            # getTimestamp_str()
        send_data = json.dumps(sResult)
        print(f"send_data : {send_data}")
        pub.publish(send_data)
        # self.spinbox1.setValue(0)
        # self.spinbox2.setValue(0)
        # self.floor_label.setText(f"{self.floor_default}")
        # print(self.floor_label.text())
        return True
        # self.spinbox3.setValue(0)
    elif msgDeliveryTable.clickedButton() == cBtnNo:
        send_data = String('no')
        print(f"send_data : {send_data}")
        return False
 
def getTimestamp():
    # current_time = rospy.get_rostime()
    
    # 년-월-일 시:분:초 포맷 예시
    # timestamp_str = datetime.fromtimestamp(current_time.to_sec()).strftime('%Y-%m-%d %H:%M:%S')
    
    # ISO 8601 확장형 포맷 예시
    #timestamp_str = datetime.fromtimestamp(current_time.to_sec()).isoformat(timespec='milliseconds')
    
    # 년월일_시분초 포맷 예시
    #timestamp_str = datetime.fromtimestamp(current_time.to_sec()).strftime('%Y%m%d_%H%M%S')
    #timestamp_str = datetime.fromtimestamp(current_time.to_sec()).strftime('%Y-%m-%d %H:%M:%S')
    
    # TIMESTAMP
    timestamp_str = datetime.now().timestamp()
    
    return timestamp_str

def setTimestamp():
    sResult['TIME'] = getTimestamp() 
    
def setState(state):
    sResult['STATE'] = state
    setTimestamp()
@dataclass
class BLB_STATUS:
    ID: str
    STATUS: str
    
@dataclass
class ORDER_INFO:
    ID: str
    TRAY_A: int
    TRAY_B: int
    FLOOR_LEVEL: int
    STATE: str
    TIME: str
                
def callback(msg):
    # parsed_data = data.data.split(chSplit1)
    # print(f"parsed_data : {parsed_data}")
    # key = parsed_data[0]
    # value = parsed_data[1]
    pattern = r'(\w+):(\w+)'
    matches = re.findall(pattern, msg.data)
    # 추출된 키-값 쌍들을 딕셔너리로 저장합니다.
    data_dict = dict(matches)
    global blb_status
    # 데이터 클래스로 변환하여 반환합니다.
    
    global prev_msg
    if prev_msg is not None and data_dict['STATUS'] == prev_msg:
        return  # 이전 메시지와 동일한 경우 로그를 찍지 않습니다
    prev_msg = data_dict['STATUS']
    
    if data_dict['ID'] != DeviceID:
        pass
    else:
        try:
            blb_status = BLB_STATUS(ID=data_dict['ID'], STATUS=data_dict['STATUS'])
            # if blb_status.ID == DeviceID:
            #     print(f"blb_status : {blb_status}")
            #     # print(f"blb_status.ID : {blb_status.ID}")
            #     # print(f"blb_status.STATUS : {blb_status.STATUS}")
            # else:
            #     return None
            # # 수신한 문자열 메시지 출력
            
            rospy.loginfo("Received message: %s", msg.data)
            # return blb_status
        except KeyError:
            return None
    
    
    
def subscribe_to_topic():
    # 구독자(Subscriber) 생성
    rospy.Subscriber('BLB_STATUS', String, callback)
    
    # 루프 유지
    rospy.spin()
        
if __name__ == '__main__':
    try:
        rospy.init_node('my_node',anonymous=True)
        sub_thread = threading.Thread(target=subscribe_to_topic, daemon=True)
        sub_thread.start()
        app = QApplication(sys.argv)
        mainWindow = WindowClass()
        mainWindow.showFullScreen()
        app.exec_()
        
    except rospy.ROSInterruptException:
        pass