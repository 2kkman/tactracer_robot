from email.charset import QP
import sys

from sympy import Q
import rospy
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QLabel, QLineEdit, QHBoxLayout, QComboBox
from PyQt5.QtCore import QThread, pyqtSignal
from turtlesim.srv import Kill, KillRequest

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
            self.result_signal.emit(f"Successfully called: {req.name}")
        except rospy.ServiceException as e:
            self.result_signal.emit(f"Service call failed: {e}")

class MyApp(QMainWindow):

    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)

        layout = QVBoxLayout(central_widget)

        self.label = QLabel("Enter key and value to call /CMD service", self)
        layout.addWidget(self.label)

        # 키와 값을 입력할 수 있는 입력 필드 추가
        h_layout = QHBoxLayout()
        # self.key_input = QLineEdit(self)
        self.key_input = QComboBox(self)
        self.key_input.addItems(["S","O"])
        
        self.key_input.currentTextChanged.connect(self.changedItem)
        
        # self.key_input.setPlaceholderText("Key (e.g., S)")
        
    
        self.value_input = QComboBox(self)
        self.value_input.addItems(["4"])
                    
        # self.value_input.setPlaceholderText("Value (e.g., 4)")
        
        # self.value_angle = QLineEdit(self)
        # self.value_angle.setPlaceholderText("Value (e.g., 30)")
        
        self.value_angle = QComboBox(self)
        self.value_angle.addItems(["30", "60", "90", "120"])
        
        h_layout.addWidget(self.key_input)
        h_layout.addWidget(self.value_input)
        h_layout.addWidget(self.value_angle)

        layout.addLayout(h_layout)

        self.button = QPushButton("Call Service", self)
        self.button.clicked.connect(self.call_service)
        
        layout.addWidget(self.button)
        
        layout_door = QVBoxLayout()
        
        self.btn_up = QPushButton(self)
        self.btn_up.setText("UP")
        self.btn_up.clicked.connect(self.call_door_up)
        
        self.btn_stop = QPushButton(self)
        self.btn_stop.setText("STOP")
        self.btn_stop.clicked.connect(self.call_door_stop)
        
        self.btn_down = QPushButton(self)
        self.btn_down.setText("DOWN")
        self.btn_down.clicked.connect(self.call_door_down)
        
        
        layout_door.addWidget(self.btn_up)
        layout_door.addWidget(self.btn_stop)
        layout_door.addWidget(self.btn_down)
        layout.addLayout(layout_door)
        
        self.result_label = QLabel("", self)
        layout.addWidget(self.result_label)

        self.setWindowTitle("ROS Service Caller with PyQt5")
        self.setGeometry(300, 300, 400, 200)
        self.show()
        
    def changedItem(self):
        if self.key_input.currentText().upper() == "S":
            self.value_input.clear()
            self.value_input.addItems(["4"])
            self.value_angle.clear()
            self.value_angle.addItems(["30", "60", "90", "120"])

        elif self.key_input.currentText().upper() == "O":
            self.value_input.clear()
            self.value_input.addItems(["0","1","2"])
            self.value_angle.clear()
            self.value_angle.addItems(["5"])
        
    def call_service(self):
        key = self.key_input.currentText()
        value = self.value_input.currentText()
        angle = self.value_angle.currentText()
        
        if key and value:
            service_argument = f"{key}:{value},{angle}"
            service_name = "/QBI/CMDARD"
            self.result_label.setText(f"Calling {service_name} with argument {service_argument}...")
            self.ros_service_caller = ROSServiceCaller(service_name, service_argument)
            self.ros_service_caller.result_signal.connect(self.display_result)
            self.ros_service_caller.start()
        else:
            self.result_label.setText("Please enter both key and value.")
            
    def call_door_up(self):
        key = "O"
        value = "2"
        angle = "5"
        
        if key and value:
            service_argument = f"{key}:{value},{angle}"
            service_name = "/QBI/CMDARD"
            self.result_label.setText(f"Calling {service_name} with argument {service_argument}...")
            self.ros_service_caller = ROSServiceCaller(service_name, service_argument)
            self.ros_service_caller.result_signal.connect(self.display_result)
            self.ros_service_caller.start()
        else:
            self.result_label.setText("Please enter both key and value.")
            
    def call_door_down(self):
        key = "O"
        value = "1"
        angle = "5"
        
        if key and value:
            service_argument = f"{key}:{value},{angle}"
            service_name = "/QBI/CMDARD"
            self.result_label.setText(f"Calling {service_name} with argument {service_argument}...")
            self.ros_service_caller = ROSServiceCaller(service_name, service_argument)
            self.ros_service_caller.result_signal.connect(self.display_result)
            self.ros_service_caller.start()
        else:
            self.result_label.setText("Please enter both key and value.")       

    def call_door_stop(self):
        key = "O"
        value = "0"
        angle = "5"
        
        if key and value:
            service_argument = f"{key}:{value},{angle}"
            service_name = "/QBI/CMDARD"
            self.result_label.setText(f"Calling {service_name} with argument {service_argument}...")
            self.ros_service_caller = ROSServiceCaller(service_name, service_argument)
            self.ros_service_caller.result_signal.connect(self.display_result)
            self.ros_service_caller.start()
        else:
            self.result_label.setText("Please enter both key and value.")     
                
    def display_result(self, result):
        self.result_label.setText(result)

if __name__ == "__main__":
    rospy.init_node('pyqt_ros_service_caller', anonymous=True)

    app = QApplication(sys.argv)
    ex = MyApp()
    sys.exit(app.exec_())
