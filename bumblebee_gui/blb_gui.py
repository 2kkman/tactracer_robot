#!/usr/bin/env python3

# Standard library imports
import os
import sys
import json
import threading
import re
from datetime import datetime
from importlib import reload
from typing import Dict
# Related third-party imports
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import rospy
from std_msgs.msg import String, Time

from flask import Flask, request, render_template

# Local application/library specific imports
from ui.KeypadDialog import KeypadDialog
# from ui.mainwindow_ui import Ui_MainWindow
from ui.single_tray_ui import Ui_MainWindow
from ui.custom_massagebox_ui import Ui_blbMessagebox
# from blb_setting import SettingClass
from Util import *
from UtilBLB import *
from ui.moving import Ui_Form
from ui.setting_ui import Ui_settingForm
from ui.setting_info_ui import Ui_SettingInfoForm
from ui.setting_language_ui import Ui_SettingLanguageForm
prev_msg = None
blb_status: Dict[str,str] = {}
blb_cmd: Dict[str,str] = {}
flagautorandr = False
global pub
pub = rospy.Publisher('BLB_CMD', String, queue_size=10)

def resource_path(relative_path):
    base_path = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base_path, relative_path)

msgfont = QFont('Noto Sans Mono CJK KR', 40)
msgfont.setBold(True)
msgfont.setWeight(75)

DeviceID = 'Bumblebee1'
table_Text = ''
    
app = Flask(__name__)
@app.route('/')



class WindowClass(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(WindowClass, self).__init__()
        self.setupUi(self)
        self.showFullScreen()
        
        sub_thread = threading.Thread(target=self.subscribe_to_topic, daemon=True)
        sub_thread.start()

        self.count = 0
        self.maximumLevel = 3
        self.minimumLevel = -3
        self.floor_level = 0
        
        # self.timer = QTimer(self)
        # self.timer.timeout.connect(self.resetCount)
        # self.timer.setSingleShot(True)
        self.second = SecondWindow()    #
        self.second.hide()
        # self.setting = SettingClass(self)
        # self.setting.hide()
        # self.floor_label.hide()
        
        # self.label_4.mousePressEvent = self.evt_app_exit
        self.logo = QPixmap(":/res/bumblebee_character.png").scaled(self.label_4.width(), self.label_4.height(), Qt.KeepAspectRatio)
        self.label_4.setPixmap(self.logo)
        # self.label_4.resize(10, 10)
        # self.label_3.mousePressEvent = self.evt_setting
        
        # self.label.mousePressEvent = self.evt_B_pressed
        # self.label.mouseReleaseEvent = self.evt_B_released
        # self.label_2.mousePressEvent = self.evt_A_pressed
        # self.label_2.mouseReleaseEvent = self.evt_A_released
        self.btn_fast.clicked.connect(self.update_btn)
        self.btn_normal.clicked.connect(self.update_btn)
        self.btn_slow.clicked.connect(self.update_btn)
        
        self.btn_normal.setChecked(True)
        
        self.translator = QTranslator()
        # self.setting.setlanguage.languageChanged.connect(self.change_language)
        pub = rospy.Publisher(TopicName.BLB_CMD.name, String, queue_size=10)
        
        blb_cmd[BLB_CMD.ID.name] = DeviceID                             #DEVICE ID
        blb_cmd[BLB_CMD.STATE.name] = BLB_CMD_STATUS.INIT.name          #STATE
        blb_cmd[BLB_CMD.TIME.name] = getTimestamp()
        rospy.loginfo("Send message: %s", blb_cmd)
        pub.publish(json.dumps(blb_cmd))
        self.update_btn()   
        app = Flask(__name__)
        
    def update_btn(self):
        self.btn_fast.setStyleSheet(
            "QRadioButton {background-color: #FDDE45;border-style: outset; border-width: 5px; border-radius: 15px;border-color: beige;min-width: 5em;padding: 5px; }\n"
            "QRadioButton::indicator { width: 0px; height: 0px;}\n")
        self.btn_normal.setStyleSheet(
            "QRadioButton {background-color: #FDDE45;border-style: outset; border-width: 5px; border-radius: 15px;border-color: beige;min-width: 5em;padding: 5px; }\n"
            "QRadioButton::indicator { width: 0px; height: 0px;}\n")
        self.btn_slow.setStyleSheet(
            "QRadioButton {background-color: #FDDE45;border-style: outset; border-width: 5px; border-radius: 15px;border-color: beige;min-width: 5em;padding: 5px; }\n"
            "QRadioButton::indicator { width: 0px; height: 0px;}\n")
        
        # 선택된 버튼의 배경색을 변경
        if self.btn_fast.isChecked():
            self.btn_fast.setStyleSheet(
                "QRadioButton {background-color: #FC6835;border-style: outset; border-width: 5px; border-radius: 15px;border-color: beige;min-width: 5em;padding: 5px; }\n"
                "QRadioButton::indicator { width: 0px; height: 0px;}\n")
        elif self.btn_normal.isChecked():
            self.btn_normal.setStyleSheet(
                "QRadioButton {background-color: #FC6835;border-style: outset; border-width: 5px; border-radius: 15px;border-color: beige;min-width: 5em;padding: 5px; }\n"
                "QRadioButton::indicator { width: 0px; height: 0px;}\n")
        elif self.btn_slow.isChecked():
            self.btn_slow.setStyleSheet(
                "QRadioButton {background-color: #FC6835;border-style: outset; border-width: 5px; border-radius: 15px;border-color: beige;min-width: 5em;padding: 5px; }\n"
                "QRadioButton::indicator { width: 0px; height: 0px;}\n")  
            
    def resetCount(self):
        """Event handler for resetting the count.
        """
        self.count = 0

    def evt_setting(self, e):
        """event handler for setting.

        Args:
            e (_type_): _description_
        """
        self.count += 1

        if self.count >= 10:
            self.spinbox1.setValue(0)
            self.spinbox2.setValue(0)
            self.floor_level = 0
            self.floor_label.setText(f"{self.floor_level}")
            self.floor_label.hide()
            
            self.evt_main_to_settingwindow()
            
        self.timer.start(500) # 500 milliseconds

    def evt_app_exit(self, e):
        """event handler to exit the application.

        Args:
            e (_type_): _description_
        """
        self.count += 1
        if self.count >= 7:
            qApp.quit()
        self.timer.start(500) # 500 milliseconds
        
    def evt_B_pressed(self, e):
        """event handler for floor default."""
        # self.floor_level = 0
        # self.floor_label.setText(f"{self.floor_level}")
        blb_cmd[BLB_CMD.ID.name] = DeviceID                             #DEVICE ID
        blb_cmd[BLB_CMD.STATE.name] = BLB_CMD_STATUS.EVENT.name         #STATE
        blb_cmd[BLB_CMD.LEVEL.name] = BLB_UI_EVENTS.B_pressed.name      #B PRESSED
        blb_cmd[BLB_CMD.TIME.name] = getTimestamp()                     #TIMESTAMP
        
        pub.publish(json.dumps(blb_cmd))
        rospy.loginfo("Send message: %s", blb_cmd)
        
    def evt_B_released(self, e):
        """event handler for floor default."""
        # self.floor_level = 0
        # self.floor_label.setText(f"{self.floor_level}")
        blb_cmd[BLB_CMD.ID.name] = DeviceID                             #DEVICE ID
        blb_cmd[BLB_CMD.STATE.name] = BLB_CMD_STATUS.EVENT.name         #STATE
        blb_cmd[BLB_CMD.LEVEL.name] = BLB_UI_EVENTS.B_released.name     #B RELEASED
        blb_cmd[BLB_CMD.TIME.name] = getTimestamp()                     #TIMESTAMP
        
        pub.publish(json.dumps(blb_cmd))
        rospy.loginfo("Send message: %s", blb_cmd)
        
    def evt_A_pressed(self, e):
        """event handler for floor default."""
        # self.floor_level = 0
        # self.floor_label.setText(f"{self.floor_level}")
        blb_cmd[BLB_CMD.ID.name] = DeviceID                             #DEVICE ID
        blb_cmd[BLB_CMD.STATE.name] = BLB_CMD_STATUS.EVENT.name         #STATE
        blb_cmd[BLB_CMD.LEVEL.name] = BLB_UI_EVENTS.A_pressed.name      #A PRESSED
        blb_cmd[BLB_CMD.TIME.name] = getTimestamp()                     #TIMESTAMP
        
        pub.publish(json.dumps(blb_cmd))
        rospy.loginfo("Send message: %s", blb_cmd)
        
    def evt_A_released(self, e):
        """event handler for floor default."""
        # self.floor_level = 0
        # self.floor_label.setText(f"{self.floor_level}")
        blb_cmd[BLB_CMD.ID.name] = DeviceID                             #DEVICE ID
        blb_cmd[BLB_CMD.STATE.name] = BLB_CMD_STATUS.EVENT.name         #STATE
        blb_cmd[BLB_CMD.LEVEL.name] = BLB_UI_EVENTS.A_released.name     #A RELEASED
        blb_cmd[BLB_CMD.TIME.name] = getTimestamp()                     #TIMESTAMP
        
        pub.publish(json.dumps(blb_cmd))
        rospy.loginfo("Send message: %s", blb_cmd)
        
    def evt_floor_default(self):
        """event handler for floor default."""
        # self.floor_level = 0
        # self.floor_label.setText(f"{self.floor_level}")
        blb_cmd[BLB_CMD.ID.name] = DeviceID                             #DEVICE ID
        blb_cmd[BLB_CMD.STATE.name] = BLB_CMD_STATUS.EVENT.name         #STATE
        blb_cmd[BLB_CMD.LEVEL.name] = BLB_UI_EVENTS.level_reset.name    #FLOOR LEVEL
        blb_cmd[BLB_CMD.TIME.name] = getTimestamp()                     #TIMESTAMP
        
        pub.publish(json.dumps(blb_cmd))
        rospy.loginfo("Send message: %s", blb_cmd)

    def evt_floor_up_pressed(self):
        """event handler for floor up pressed."""
        self.floor_level += 1 
        self.floor_level = min(self.floor_level, self.maximumLevel)
        # self.floor_label.setText(f"{self.floor_level}")

        blb_cmd[BLB_CMD.ID.name] = DeviceID                             #DEVICE ID
        blb_cmd[BLB_CMD.STATE.name] = BLB_CMD_STATUS.EVENT.name         #STATE
        blb_cmd[BLB_CMD.LEVEL.name] = BLB_UI_EVENTS.up_pressed.name     #FLOOR LEVEL
        blb_cmd[BLB_CMD.TIME.name] = getTimestamp()                     #TIMESTAMP

        pub.publish(json.dumps(blb_cmd))
        rospy.loginfo("Send message: %s", blb_cmd)
                        
    def evt_floor_up_released(self):
        """event handler for floor up released."""
        self.floor_level += 1 
        self.floor_level = min(self.floor_level, self.maximumLevel)
        # self.floor_label.setText(f"{self.floor_level}")
        
        blb_cmd[BLB_CMD.ID.name] = DeviceID                             #DEVICE ID
        blb_cmd[BLB_CMD.STATE.name] = BLB_CMD_STATUS.EVENT.name         #STATE
        blb_cmd[BLB_CMD.LEVEL.name] = BLB_UI_EVENTS.up_released.name    #FLOOR LEVEL
        blb_cmd[BLB_CMD.TIME.name] = getTimestamp()                     #TIMESTAMP
        
        pub.publish(json.dumps(blb_cmd))
        rospy.loginfo("Send message: %s", blb_cmd)

    def evt_floor_down_pressed(self):
        """event handler for floor down pressed."""
        self.floor_level -= 1 
        self.floor_level = max(self.floor_level, self.minimumLevel)
        # self.floor_label.setText(f"{self.floor_level}")
        
        blb_cmd[BLB_CMD.ID.name] = DeviceID                             #DEVICE ID
        blb_cmd[BLB_CMD.STATE.name] = BLB_CMD_STATUS.EVENT.name         #STATE
        blb_cmd[BLB_CMD.LEVEL.name] = BLB_UI_EVENTS.down_pressed.name   #FLOOR LEVEL
        blb_cmd[BLB_CMD.TIME.name] = getTimestamp()                     #TIMESTAMP
        
        pub.publish(json.dumps(blb_cmd))
        rospy.loginfo("Send message: %s", blb_cmd)
        
    def evt_floor_down_released(self):
        """event handler for floor down released."""
        self.floor_level -= 1 
        self.floor_level = max(self.floor_level, self.minimumLevel)
        # self.floor_label.setText(f"{self.floor_level}")
        
        blb_cmd[BLB_CMD.ID.name] = DeviceID                             #DEVICE ID
        blb_cmd[BLB_CMD.STATE.name] = BLB_CMD_STATUS.EVENT.name         #STATE
        blb_cmd[BLB_CMD.LEVEL.name] = BLB_UI_EVENTS.down_released.name  #FLOOR LEVEL
        blb_cmd[BLB_CMD.TIME.name] = getTimestamp()                     #TIMESTAMP
        
        pub.publish(json.dumps(blb_cmd))
        rospy.loginfo("Send message: %s", blb_cmd)
        
    def evt_floor_up(self):
        """event handler for floor up."""
        self.floor_level += 1 
        self.floor_level = min(self.floor_level, self.maximumLevel)
        self.floor_label.setText(f"{self.floor_level}")
        
    def evt_floor_down(self):
        """event handler for floor down."""
        self.floor_level -= 1 
        self.floor_level = max(self.floor_level, self.minimumLevel)
        self.floor_label.setText(f"{self.floor_level}")        
        
    def btn_main_to_movewindow(self):
        """event handler for moving window."""
        self.hide()                     # 메인윈도우 숨김
        self.second.showFullScreen()     
        self.second.exec_()              # 두번째 창을 닫을 때 까지 기다림
        self.showFullScreen() 

    def evt_main_to_settingwindow(self):
        """event handler for setting window."""
        self.hide()                     # 메인윈도우 숨김
        self.setting.showFullScreen()     
        # self.setting.exec_()              # 두번째 창을 닫을 때 까지 기다림
        self.showFullScreen() 
                
    def evt_send_message(self):
        """event handler for sending message."""
        global blb_cmd

        blb_cmd.clear()

        blb_cmd[BLB_CMD.ID.name] = DeviceID                             #DEVICE ID
        blb_cmd[BLB_CMD.TRAY_A.name] = self.spinbox1.text()             #TABLE NO
        
        try:
            if self.spinbox2:
                blb_cmd[BLB_CMD.TRAY_B.name] = self.spinbox2.text()             #TABLE NO
        except:
            blb_cmd[BLB_CMD.TRAY_B.name] = "-1"
                
        # blb_cmd[BLB_CMD.TRAY_B.name] = self.spinbox2.text()             #TABLE NO
        # blb_cmd[BLB_CMD.TRAY_B.name] = self.spinbox2.text()             #TABLE NO
        try:
            if self.floor_label:
                blb_cmd[BLB_CMD.LEVEL.name] = self.floor_label.text()           #FLOOR LEVEL
        except:
            blb_cmd[BLB_CMD.LEVEL.name] = "-1"
            
        if self.btn_normal.isChecked():
            blb_cmd[BLB_CMD.MODE.name] = BLB_CMD_MODE.NORMAL.name
        elif self.btn_fast.isChecked():
            blb_cmd[BLB_CMD.MODE.name] = BLB_CMD_MODE.FAST.name
        elif self.btn_slow.isChecked():
            blb_cmd[BLB_CMD.MODE.name] = BLB_CMD_MODE.SLOW.name
            
        # blb_cmd[BLB_CMD.MODE.name] = self.btn_normal.isChecked()           #TABLE NO
        blb_cmd[BLB_CMD.STATE.name] = BLB_CMD_STATUS.MOVE.name          #STATE
        blb_cmd[BLB_CMD.TIME.name] = getTimestamp()                     #TIMESTAMP

        print(f"BLB_CMD : {blb_cmd}")
        self.custommsg = CustomMessageBox("테이블 확인", "확인", "취소")
        self.show()
        self.custommsg.exec_()
        ret = self.custommsg.button_clicked
        
        if ret:
            self.spinbox1.setValue(0)
            # self.spinbox2.setValue(0)
            # self.floor_level = 0
            # self.floor_label.setText(f"{self.floor_level}")
            self.btn_main_to_movewindow()
        else:
            pass

        
    def status_callback(self, msg):
        """callback function for status."""
        
        # pattern = r'(\w+):(\w+)'
        # matches = re.findall(pattern, msg.data)
        # # 추출된 키-값 쌍들을 딕셔너리로 저장합니다.
        # data_dict = dict(matches)
        data_dict = json.loads(msg.data)
        global blb_status
        # 데이터 클래스로 변환하여 반환합니다.
        global table_Text
        global prev_msg
        global flagautorandr
        
        if prev_msg is not None and data_dict[BLB_STATUS.STATUS.name] == prev_msg:
            return  # 이전 메시지와 동일한 경우 로그를 찍지 않습니다
        prev_msg = data_dict[BLB_STATUS.STATUS.name]
        
        if data_dict[BLB_STATUS.ID.name] != DeviceID:
            pass
        else:
            try:
                blb_status[BLB_STATUS.ID.name] = data_dict[BLB_STATUS.ID.name]
                blb_status[BLB_STATUS.STATUS.name] = data_dict[BLB_STATUS.STATUS.name]
                    
                self.second.btnConfirm.setVisible(False)
            
                if blb_status[BLB_STATUS.STATUS.name] == BLB_STATUS_FIELD.READY.name:
                    # print(f"{blb_status[BLB_STATUS.ID.name]} is Ready")
                    self.second.hide()
                    # table_Text = f"준비중 ..."
                elif blb_status[BLB_STATUS.STATUS.name] == BLB_STATUS_FIELD.MOVING.name:
                    # print(f"{blb_status[BLB_STATUS.ID.name]} is Moving")
                    table_Text = f"이동중 ..."
                elif blb_status[BLB_STATUS.STATUS.name] == BLB_STATUS_FIELD.LIFTING_DOWN.name:
                    # print(f"{blb_status[BLB_STATUS.ID.name]} is Lifting Down")
                    table_Text = f"하강중 ..."
                elif blb_status[BLB_STATUS.STATUS.name] == BLB_STATUS_FIELD.LIFTING_UP.name:
                    # print(f"{blb_status[BLB_STATUS.ID.name]} is Lifting Up")
                    table_Text = f"상승중 ..."
                elif blb_status[BLB_STATUS.STATUS.name] == BLB_STATUS_FIELD.WAITING.name:
                    # print(f"{blb_status[BLB_STATUS.ID.name]} is Waiting")
                    table_Text = f"대기중 ..."
                elif blb_status[BLB_STATUS.STATUS.name] == BLB_STATUS_FIELD.HOMING.name:
                    # print(f"{blb_status[BLB_STATUS.ID.name]} is Homing")
                    table_Text = f"복귀중 ..."
                elif blb_status[BLB_STATUS.STATUS.name] == BLB_STATUS_FIELD.CHARGING.name:
                    # print(f"{blb_status[BLB_STATUS.ID.name]} is Charging")
                    table_Text = f"충전중 ..."
                elif blb_status[BLB_STATUS.STATUS.name] == BLB_STATUS_FIELD.FOLDING.name:
                    # print(f"{blb_status[BLB_STATUS.ID.name]} is Lifting Down")
                    table_Text = f"접는중 ..."
                elif blb_status[BLB_STATUS.STATUS.name] == BLB_STATUS_FIELD.EXPANDING.name:
                    # print(f"{blb_status[BLB_STATUS.ID.name]} is Lifting Down")
                    table_Text = f"펴는중 ..."
                elif blb_status[BLB_STATUS.STATUS.name] == BLB_STATUS_FIELD.CONFIRM.name:
                    self.second.btnConfirm.setText('확인')
                    # print(f"{blb_status[BLB_STATUS.ID.name]} is Charging")
                    table_Text = f"주문하신 음식이 나왔습니다."
                    self.second.btnConfirm.setVisible(True)
                elif blb_status[BLB_STATUS.STATUS.name] ==  BLB_STATUS_FIELD.INIT.name:
                    # if not flagautorandr:
                    os.system('xinput set-prop "TeNizo TeNizo_R7Series_TC" --type=float "Coordinate Transformation Matrix" 0 1 0 -1 0 1 0 0 1')
                    os.system('autorandr --change')  
                        # flagautorandr = True   
                else:
                    # print(f"BubleBee1 is Unknown")
                    table_Text = f"{blb_status[BLB_STATUS.STATUS.name]}"

                self.second.update_label(table_Text)
                rospy.loginfo("Received message: %s", blb_status)
                # return blb_status
            except KeyError:
                return None
        
        
    def subscribe_to_topic(self):
        # 구독자(Subscriber) 생성
        rospy.Subscriber('BLB_STATUS', String, self.status_callback)
        # 루프 유지
        rospy.spin()
        
    def change_language(self, locale):
        """Changes the language.

        Args:
            locale (string): target language locale
        """
        languagepath = "/root/catkin_ws/src/bumblebee_gui/language/"
        target_path = languagepath+ "main_" + locale + ".qm"
        # ret = self.translator.load(target_path)
        # target_path = languagepath+ locale + ".qm"
        
        ret = self.translator.load(target_path)
        print(f"change_language : {locale}, ret : {ret}, target_path : {target_path}")

        # ret = self.translator.load(QLocale(), locale, prefix="menu", directory="/root/catkin_ws/src/bumblebee_gui/language", suffix=".qm")
        QApplication.instance().installTranslator(self.translator)
        self.retranslateUi(self)     
              
class SecondWindow(QDialog, QWidget, Ui_Form):
    """SecondWindows Class.

    Args:
        QDialog (object): _description_
        QWidget (object): _description_
        Ui_Form (object): Moving UI
    """
    def __init__(self, parent=None):
        super(SecondWindow,self).__init__()
        global table_Text
        global received_message
       
        received_message = ''
        
        self.parent = parent
        self.initUi()
        # self.showFullScreen()
        # self.table_label.setText(f'이동중 ...')
        # self.timer = QTimer(self)
        # self.timer.timeout.connect(self.close),
        # self.timer.stop()
        
        self.btnTable.setVisible(False)
        self.lblImage.setStyleSheet("QLabel {background-color: #FDDE45;}")
        self.lblImage.setVisible(True)
        self.lblImage.setPixmap(QPixmap(":/res/bumblebee_character_s.png"))        
        self.btnConfirm.setVisible(False)
        self.btnConfirm.resize(180, 400)
        self.btnConfirm.move(1100, 0)
        self.btnConfirm.setFont(msgfont)
        self.btnConfirm.clicked.connect(self.btn_return)
     
    def initUi(self):
        """initializes the UI."""
        self.setupUi(self)

    def btn_return(self):
        """button handler for return."""
        if blb_status[BLB_STATUS.STATUS.name] ==  BLB_STATUS_FIELD.HOMING.name:
            self.close()                    #클릭시 종료됨.
        elif blb_status[BLB_STATUS.STATUS.name] ==  BLB_STATUS_FIELD.CONFIRM.name:  # READY 상태에서 확인버튼 클릭시
            self.set_state('CONFIRM')
            self.table_label.setText(f"문이 닫힙니다.")
            self.btnConfirm.setVisible(False)
            # self.close()
            # self.__init__()
        else:
            self.close()
            self.__init__()

    def timer_second_to_main(self):
        """timer handler for second to main."""
        self.close()
        self.__init__()
        
    def update_label(self, text):
        """Updates the label.

        Args:
            text (string): Text to be displayed
        """
        self.table_label.setText(text)  
           
    def set_state(self, valuse):
        """set state.

        Args:
            valuse (string): State value
        """
        blb_cmd[BLB_CMD.STATE.name] = valuse
        blb_cmd[BLB_CMD.TIME.name] = getTimestamp()
        rospy.loginfo("Send message: %s", blb_cmd)
        pub.publish(json.dumps(blb_cmd))
           
class CustomMessageBox(QDialog, Ui_blbMessagebox):
    def __init__(self, lbl_title, lbl_left, lbl_right, parent=None):
        super(CustomMessageBox, self).__init__(parent)
        self.setupUi(self)
        self.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.FramelessWindowHint)
        self.btn_left.setText(lbl_left)
        self.btn_right.setText(lbl_right)
        aTable = blb_cmd[BLB_CMD.TRAY_A.name]
        bTable = blb_cmd[BLB_CMD.TRAY_B.name]
        self.lbl_title.setText(lbl_title)
        
        try:
            if bTable == '-1':
                self.lbl_message.setText(f"A :\t{aTable}")
        except:
            self.lbl_message.setText(f"A :\t{aTable} \nB :\t{bTable}") 
            
        # self.lbl_message.setText(f"A :\t{aTable} \nB :\t{bTable}")
        self.button_clicked = False

    def evt_clicked_left(self):
        """Event handler for left button."""
        aTable = blb_cmd[BLB_CMD.TRAY_A.name]
        print(f'{aTable}')
        url = f'https://{BLB_SVR_IP_DEFAULT}:3031/api/BeeCoreAPI/robot_task_action?target=T{aTable}&tasktype=1&updateduser=11'
        response = requests.get(url, verify=False)  # SSL 인증 무시
        if response.status_code == 200:
            print(f'Success: ' + response.text)
        else:
            print(f'Failed with status code: {response.status_code}')
 
        rospy.loginfo("Send message: %s", blb_cmd)
        # pub.publish(json.dumps(blb_cmd))
        self.close()
        self.button_clicked = True
        
    def evt_clicked_right(self):
        """Event handler for right button."""
        send_msg = str('cancel')
        rospy.loginfo("Send message: %s", send_msg)
        self.reject()
        self.button_clicked = False 

                
class SettingClass(QDialog, QWidget, Ui_settingForm):
    def __init__(self, parent=WindowClass):
        super(SettingClass, self).__init__()
        self.initUi()
        self.parent = parent
        
    def initUi(self):
        """initializes the UI."""
        self.setupUi(self)    
        # self.timer = QTimer(self)
        # self.timer.timeout.connect(self.close),
        # self.timer.stop()
        
        self.setinfo = SettingInfoClass()    #
        self.setinfo.hide()
        self.setlanguage = SettingLanguageClass()
        self.setlanguage.hide()
        
        self.translator = QTranslator()
        self.setlanguage.languageChanged.connect(self.change_language)
        
    def evt_setting(self):
        """Event handler for setting windows."""
        print('evt_setting')
        # self.hide()
        # self.battery.showFullScreen()
        # self.battery.exec_()
        # self.showFullScreen()
        
    def evt_language(self):
        """Event handler for language windows."""
        self.hide()
        self.setlanguage.showFullScreen()
        self.setlanguage.exec_()
        self.showFullScreen()
        
    def evt_battery(self):
        """Event handler for battery windows."""
        print('evt_battery')
        # self.hide()
        # self.setbattery.showFullScreen()
        # self.setbattery.exec_()
        # self.showFullScreen()
        
    def evt_info(self):
        """Event handler for info windows."""
        self.hide()
        self.setinfo.showFullScreen()
        self.setinfo.exec_()
        self.showFullScreen()
        
    def evt_return(self):
        """Event handler for return windows."""
        # self.close()
        print("evt_return called")
        self.hide()
        self.parent.showFullScreen()
        print("WindowClass should now be in full screen mode")
        
    def change_language(self, locale):
        """Changes the language.

        Args:
            locale (string): target language locale
        """
        languagepath = "/root/catkin_ws/src/testgui/language/"
        target_path = languagepath+ "menu_" + locale + ".qm"
        ret = self.translator.load(target_path)
        # print(f"change_language : {locale}, ret : {ret}")

        # ret = self.translator.load(QLocale(), locale, prefix="menu", directory="/root/catkin_ws/src/bumblebee_gui/language", suffix=".qm")
        QApplication.instance().installTranslator(self.translator)
        self.retranslateUi(self)           
               
class SettingInfoClass(QDialog, QWidget, Ui_SettingInfoForm):
    def __init__(self ):
        super(SettingInfoClass, self).__init__()
        self.initUi()
        # self.showFullScreen()
        # self.table_label.setText(f'이동중 ...')
        self.translator = QTranslator()
        
    def initUi(self):
        self.setupUi(self)

    def evt_return(self):
        # self.close()
        self.hide()
        # self.parent.showFullScreen()
    
    @pyqtSlot()    
    def change_language(self, locale):
        """Changes the language.
        
        Args:
            locale (string): target language locale
        """
        languagepath = "/root/catkin_ws/src/bumblebee_gui/language/"
        target_path = languagepath+ locale + ".qm"
        self.translator.load(QLocale(), target_path)
        QApplication.instance().installTranslator(self.translator)
        self.retranslateUi(self)        
        
class SettingLanguageClass(QDialog, QWidget, Ui_SettingLanguageForm):
    languageChanged = pyqtSignal(str)
    
    def __init__(self):
        super(SettingLanguageClass, self).__init__()
        self.initUi()       
        
    def initUi(self):
        self.setupUi(self)
        self.lbl_language.setText('Language Setting')
        self.translator = QTranslator()
        self.change_language("en_US")

    def evt_return(self):
        # self.close()
        self.hide()
                
    def evt_clicked_korean(self, event):
        self.change_language("ko_KR")
        
    def evt_clicked_english(self, event):
        self.change_language("en_US") 
    
    @pyqtSlot()           
    def change_language(self, locale):
        # print(f"change_language : {locale}")
        languagepath = "/root/catkin_ws/src/bumblebee_gui/language/"
        target_path = languagepath+ locale + ".qm"
        self.translator.load(QLocale(), target_path)
        # ret=self.translator.load(QLocale(),locale,directory="/root/catkin_ws/src/bumblebee_gui/language",suffix=".qm")
        # print(f"ret : {ret}")
        QApplication.instance().installTranslator(self.translator)
        self.retranslateUi(self)  
        self.languageChanged.emit(locale)  # emit signal when language is changed        
        


if __name__ == '__main__':
    try:
        rospy.init_node('BLB',anonymous=False)
        app = QApplication(sys.argv)

        
        mainWindow = WindowClass()
        app.exec_()
        
    except rospy.ROSInterruptException:
        pass