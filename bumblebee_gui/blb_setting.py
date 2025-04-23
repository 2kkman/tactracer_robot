import os
import sys
import json
import threading
import re
from datetime import datetime
from importlib import reload
from typing import Dict

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5 import uic
import rospy
from std_msgs.msg import String, Time

from ui.KeypadDialog import KeypadDialog
from ui.mainwindow_ui import Ui_MainWindow
from ui.custom_massagebox_ui import Ui_blbMessagebox
from ui.setting_ui import Ui_settingForm
from ui.setting_info_ui import Ui_SettingInfoForm
from ui.setting_language_ui import Ui_SettingLanguageForm
from ui.moving import Ui_Form
from Util import *
from UtilBLB import *

prev_msg = None
blb_status: Dict[str,str] = {}
blb_cmd: Dict[str,str] = {}

def resource_path(relative_path):
    base_path = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(base_path, relative_path)

msgfont = QFont('Noto Sans Mono CJK KR', 40)
msgfont.setBold(True)
msgfont.setWeight(75)

DeviceID = 'Bumblebee1'
table_Text = ''

class SettingClass(QWidget, Ui_settingForm):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
                
        self.setinfo = SettingInfoClass()    #
        self.setinfo.hide()
        self.setlanguage = SettingLanguageClass()
        self.setlanguage.hide()
        self.translator = QTranslator()
        self.change_language("en_US")
        self.setlanguage.languageChanged.connect(self.change_language)
    def evt_setting(self):
        print('evt_setting')
    
    def evt_language(self):
        self.hide()
        self.setlanguage.showFullScreen()
        self.setlanguage.exec()
        self.showFullScreen()
        
    def evt_battery(self):
        print('evt_battery')
        
    def evt_info(self):
        self.hide()
        self.setinfo.showFullScreen()
        self.setinfo.exec()
        self.showFullScreen()
        
    def evt_return(self):
        self.close()
        
    def change_language(self, locale):
        languagepath = "/root/catkin_ws/src/bumblebee_gui/language/"
        target_path = languagepath+ "menu_" + locale + ".qm"
        ret = self.translator.load(target_path)
        print(f"change_language : {locale} ret : {ret}")

        # ret = self.translator.load(QLocale(), locale, prefix="menu", directory="/root/catkin_ws/src/testgui/language", suffix=".qm")
        QApplication.instance().installTranslator(self.translator)
        self.retranslateUi(self)           
               
class SettingInfoClass(QDialog, QWidget, Ui_SettingInfoForm):
    def __init__(self):
        super().__init__()
        self.initUi()
        # self.showFullScreen()
        # self.table_label.setText(f'이동중 ...')
        self.translator = QTranslator()
        
    def initUi(self):
        self.setupUi(self)

    def evt_return(self):
        self.close()
    
    @pyqtSlot()    
    def change_language(self, locale):
        languagepath = "/root/catkin_ws/src/bumblebee_gui/language/"
        target_path = languagepath+ locale + ".qm"
        self.translator.load(QLocale(), target_path)
        QApplication.instance().installTranslator(self.translator)
        self.retranslateUi(self)        
        
class SettingLanguageClass(QDialog, QWidget, Ui_SettingLanguageForm):
    languageChanged = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.initUi()       
        
    def initUi(self):
        self.setupUi(self)
        self.lbl_language.setText('Language Setting')
        self.translator = QTranslator()
        self.translator = QTranslator()
        self.change_language("en_US")

    def evt_return(self):
        self.close()
                
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
        # ret=self.translator.load(QLocale(),locale,directory="/root/catkin_ws/src/testgui/language",suffix=".qm")
        # print(f"ret : {ret}")
        QApplication.instance().installTranslator(self.translator)
        self.retranslateUi(self)  
        self.languageChanged.emit(locale)  # emit signal when language is changed

if __name__ == '__main__':
    try:
        rospy.init_node('BLB',anonymous=False)
        app = QApplication(sys.argv)
        mainWindow = SettingClass()
        mainWindow.showFullScreen()
        # app.exec_()
        app.exec()
        
    except rospy.ROSInterruptException:
        pass