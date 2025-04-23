import sys

import rospy
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QLabel, QLineEdit, QHBoxLayout, QComboBox
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from turtlesim.srv import Kill, KillRequest
from tta_blb.srv import TTS, TTSRequest
from ui.tray_ui import Ui_MainWindow

class ROSServiceCaller(QThread):
    result_signal = pyqtSignal(str)

    def __init__(self, service_name, service_argument):
        super().__init__()
        self.service_name = service_name
        self.service_argument = service_argument

    def run(self):
        rospy.wait_for_service(self.service_name)
        try:
            # 서비스 호출 준비
            service_proxy = rospy.ServiceProxy(self.service_name, Kill)
            req = KillRequest()
            req.name = self.service_argument
            response = service_proxy(req)
            self.result_signal.emit(f"Successfully called: {response}")
        except rospy.ServiceException as e:
            self.result_signal.emit(f"Service call failed: {e}")
            
class ROSServiceTTSCaller(QThread):
    result_signal = pyqtSignal(str)

    def __init__(self, service_name, text):
        super().__init__()
        self.service_name = service_name
        self.service_text = text
        # self.service_queuing = queuing
        

    def run(self):
        rospy.wait_for_service(self.service_name)
        try:
            # 서비스 호출 준비
            service_proxy = rospy.ServiceProxy(self.service_name, TTS)
            req = TTSRequest()
            req.text = self.service_text
            response = service_proxy(req)
            self.result_signal.emit(f"Successfully called: {response}")
        except rospy.ServiceException as e:
            self.result_signal.emit(f"Service call failed: {e}")
            
class MyApp(QMainWindow, Ui_MainWindow):

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.initUI()

    def initUI(self):
        # 키와 값을 입력할 수 있는 입력 필드 추가
        self.btn_call.clicked.connect(self.call_service)        
        self.btn_up.clicked.connect(self.call_door_up)       
        self.btn_stop.clicked.connect(self.call_door_stop)
        self.btn_down.clicked.connect(self.call_door_down)
        self.btn_degree23.clicked.connect(self.call_degree_23)
        self.btn_degree30.clicked.connect(self.call_degree_30)
        self.btn_degree45.clicked.connect(self.call_degree_45)
        # self.btn_degree60.clicked.connect(self.call_degree_60)
        self.btn_degree90.clicked.connect(self.call_degree_90)
        self.btn_degree120.clicked.connect(self.call_degree_120)
        self.btn_send.clicked.connect(self.call_tts_send)
        
        self.cb_cmd.currentTextChanged.connect(self.changedItem)
        self.cb_cmd.setEditable(True)
        cb_cmd_line_edit = self.cb_cmd.lineEdit()
        cb_cmd_line_edit.setAlignment(Qt.AlignCenter)
        cb_cmd_line_edit.setReadOnly(True)
        
        self.cb_name.setEditable(True)
        cb_name_line_edit = self.cb_name.lineEdit()
        cb_name_line_edit.setAlignment(Qt.AlignCenter)
        cb_name_line_edit.setReadOnly(True)

        self.cb_value.setEditable(True)
        cb_value_line_edit = self.cb_value.lineEdit()
        cb_value_line_edit.setAlignment(Qt.AlignCenter)
        cb_value_line_edit.setReadOnly(True)
                
        self.result_label = QLabel("", self)
        self.verticalLayout_6.addWidget(self.result_label)

        self.setWindowTitle("ROS Service Caller with PyQt5")
        self.show()
        
    def changedItem(self):
        if self.cb_cmd.currentText().upper() == "S":
            self.cb_name.clear()
            self.cb_name.addItems(["4"])
            self.cb_value.clear()
            self.cb_value.addItems(["23","30", "45", "90", "120"])

        elif self.cb_cmd.currentText().upper() == "O":
            self.cb_name.clear()
            self.cb_name.addItems(["0","1","2"])
            self.cb_value.clear()
            self.cb_value.addItems(["5"])
        
    def call_service(self):
        key = self.cb_cmd.currentText()
        name = self.cb_name.currentText()
        value = self.cb_value.currentText()
        
        self.run_caller(key, name, value)

            
    def call_door_up(self):
        key = "O"
        name = "2"
        value = "5"

        self.run_caller(key, name, value)
        
    def call_door_down(self):
        key = "O"
        name = "1"
        value = "5"
        
        self.run_caller(key, name, value)

    def call_door_stop(self):
        key = "O"
        name = "0"
        value = "5"
        
        self.run_caller(key, name, value)
        
    def display_result(self, result):
        
        self.result_label.setText(result)
        
    def call_degree_23(self):
        key = "S"
        name = "4"
        value = "23"
        
        self.run_caller(key, name, value)
                
    def call_degree_30(self):
        key = "S"
        name = "4"
        value = "30"
        
        self.run_caller(key, name, value)
        
    def call_degree_45(self):
        key = "S"
        name = "4"
        value = "45"
        
        self.run_caller(key, name, value)
        
    def call_degree_60(self):
        key = "S"
        name = "4"
        value = "60"
        
        self.run_caller(key, name, value)
        
    def call_degree_90(self):
        key = "S"
        name = "4"
        value = "90"
        
        self.run_caller(key, name, value)  
        
    def call_degree_120(self):
        key = "S"
        name = "4"
        value = "120"
        
        self.run_caller(key, name, value)
        
    def call_tts_send(self):
        value = self.textEdit.toPlainText()
  

        if value:
            # 문자열을 f-string으로 포맷팅하면서 중괄호를 이스케이프 처리
            service_argument = f'{value}'
            service_name = "/tts"
            self.result_label.setText(f"{service_argument}")
            self.ros_service_caller = ROSServiceTTSCaller(service_name, service_argument)
            self.ros_service_caller.result_signal.connect(self.display_result)
            self.ros_service_caller.start()
        else:
            self.result_label.setText("Please enter massage.")
                
                              
    def run_caller(self, key, name, value):
        if key and value:
            service_argument = f"{key}:{name},{value}"
            service_name = "/QBI/CMDARD"
            self.result_label.setText(f"{service_argument}")
            self.ros_service_caller = ROSServiceCaller(service_name, service_argument)
            self.ros_service_caller.result_signal.connect(self.display_result)
            self.ros_service_caller.start()
        else:
            self.result_label.setText("Please enter both key and value.")  
            
            
if __name__ == "__main__":
    rospy.init_node('pyqt_ros_service_caller', anonymous=True)

    app = QApplication(sys.argv)
    ex = MyApp()
    ex.show()
    sys.exit(app.exec_())
