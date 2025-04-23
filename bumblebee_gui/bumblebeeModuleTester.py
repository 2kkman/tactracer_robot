import sys

from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout
from PyQt5.QtCore import QTimer, QDate, Qt
from PyQt5.QtGui import QTransform, QPixmap, QPainter, QPen, QPalette, QColor

import rospy
from std_msgs.msg import String, Header
from Util import *
from ui.bumblebeeModuleTester_ui import Ui_MainWindow
from MotorControl import MOTOR_CONTROL_CMD
from Util import *
from UtilBLB import *

import threading


dic_485ex = {}  # 모터 모니터링 데이터
    
global motorctl
motorctl = MOTOR_CONTROL_CMD()


class bumblebeeModuleTester(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(bumblebeeModuleTester, self).__init__()
        self.setupUi(self)
        self.initUI()
        
        # self.set_init()
        self.cmd_setting_load("/root/catkin_ws/src/tactracer_robot/bumblebee_gui/config/setting_value_list.json")
        self.set_logo_image('/root/catkin_ws/src/tactracer_robot/bumblebee_gui/resource/bumblebee_logo.png')
        self.set_wheel_image('/root/catkin_ws/src/tactracer_robot/bumblebee_gui/resource/wheel.png')
        
        topic_list = [
            "MB_13",
            "MB_30",
            "MB_10",
            "MB_15",
            "MB_12",
            "MB_29",
            "MB_28",
            "MB_31",
            "MB_6",
        ]
        
        for topic_name in topic_list:
            sub_thread = threading.Thread(target=self.subscribeTopic, args=(topic_name,), daemon=True)
            
            sub_thread.start()
    
        for identifier in range(1, 9 + 1):
            self.set_timer(identifier)
       
           
    def set_logo_image(self, src: str):
        self.pixmap = QPixmap(src)
        self.logo.setPixmap(self.pixmap)        
        
        
    def set_wheel_image(self, src: str):
        self.pixmap = QPixmap(src)
        self.pixmap = self.pixmap.scaled(128, 128, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.Wheel_4.setPixmap(self.pixmap)
        self.Wheel_5.setPixmap(self.pixmap)
        self.Wheel_8.setPixmap(self.pixmap)
        

    def subscribeTopic(self, topic_name: str):
            self.subscriber = rospy.Subscriber(topic_name, String, self.callbackCmd)
            rospy.spin()
    
    
    def callbackCmd(self, data):
        global recvDataMap

        recvDataMap = getDic_strArr(data.data, sDivFieldColon, sDivItemComma)
        mbid = recvDataMap.get(MotorWMOVEParams.MBID.name, None)
        
        global motorID_list
        motorID_list = ["13","30","10","15","12","29","28","31","6",]
        # 28, 31, 6
        if mbid is not None:
            dic_485ex[mbid] = recvDataMap
            
            for index, motorID in enumerate(motorID_list):
                # recvDataMap[mbid] = recvDataMap
                if mbid == motorID:
                    di_not = dic_485ex[mbid].get(MonitoringField.DI_NOT.name, None)
                    di_pot = dic_485ex[mbid].get(MonitoringField.DI_POT.name, None)
                    di_home = dic_485ex[mbid].get(MonitoringField.DI_HOME.name, None)
                    not_pos = dic_485ex[mbid].get(MonitoringField.NOT_POS.name, None)
                    pot_pos = dic_485ex[mbid].get(MonitoringField.POT_POS.name, None)
                    cur_pos = dic_485ex[mbid].get(MonitoringField.CUR_POS.name, None)
                    
                    palette = QPalette()
                    
                    if di_not is not None:
                        di_not_color = self.get_di_color(di_not)
                        palette.setColor(QPalette.Base, QColor(di_not_color))
                        getattr(self, f"lineEdit_not_{index+1}").setPalette(palette)
                    
                    if di_pot is not None:
                        di_pot_color = self.get_di_color(di_pot)
                        palette.setColor(QPalette.Base, QColor(di_pot_color))
                        getattr(self, f"lineEdit_pot_{index+1}").setPalette(palette)
                    
                    if di_home is not None:
                        di_home_color = self.get_di_color(di_home)
                        palette.setColor(QPalette.Base, QColor(di_home_color))
                        getattr(self, f"lineEdit_home_{index+1}").setPalette(palette)
                        
                    # self.set_palette(di_home, f"lineEdit_home", index)
                    _slider = getattr(self, f"Slider_{index+1}")
                    
                    if _slider.maximum() is not int(pot_pos):
                        if pot_pos == '-1':
                            max_value = int(10e5)
                            _slider.setMaximum(max_value)
                            tick_interval = self.get_tick_interval(max_value, 10)
                            _slider.setTickInterval(tick_interval)

                        else:                    
                            _slider.setMaximum(int(pot_pos))
                            tick_interval = self.get_tick_interval(int(pot_pos), 10)
                            _slider.setTickInterval(tick_interval)

                    getattr(self, f"label_motor_value_{index+1}").setText(cur_pos)
    
    
    def set_palette(self, di_name: str, lineEdit_name: str, index: int):
        palette = QPalette()
        if di_name is not None:
            di_color = self.get_di_color(di_name)
            palette.setColor(QPalette.Base, QColor(di_color))
            getattr(self, f"{lineEdit_name}_{index+1}").setPalette(palette)
            
    
    def update_ui(self):
        mbid = recvDataMap.get(MotorWMOVEParams.MBID.name, None)
        for index, motorID in enumerate(motorID_list):
            if mbid == motorID:
               
                di_not = recvDataMap.get(MonitoringField.DI_NOT, None)
                di_pot = recvDataMap.get(MonitoringField.DI_POT, None)
                di_home = recvDataMap.get(MonitoringField.DI_HOME, None)
                not_pos = recvDataMap.get(MonitoringField.NOT_POS, None)
                pot_pos = recvDataMap.get(MonitoringField.POT_POS, None)
                cur_pos = recvDataMap.get(MonitoringField.CUR_POS, None)
                
                palette = QPalette()
                
                if di_not is not None:
                    di_not_color = self.get_di_color(di_not)
                    palette.setColor(QPalette.Base, QColor(di_not_color))
                    self.lineEdits_not[f'lineEdit_not_{index+1}'].setPalette(palette)
                
                if di_pot is not None:
                    di_pot_color = self.get_di_color(di_pot)
                    palette.setColor(QPalette.Base, QColor(di_pot_color))
                    self.lineEdits_pot[f'lineEdit_pot_{index+1}'].setPalette(palette)
                
                if di_home is not None:
                    di_home_color = self.get_di_color(di_home)
                    palette.setColor(QPalette.Base, QColor(di_home_color))
                    self.lineEdits_home[f'lineEdit_home_{index+1}'].setPalette(palette)
                
                if pot_pos is not None:
                    slider = self.sliders[f'Slider_{index+1}']
                    if slider.maximum() is not int(pot_pos):
                        slider.setMaximum(int(pot_pos))
                        tick_interval = self.get_tick_interval(int(pot_pos), step=10)
                        slider.setTickInterval(tick_interval)

                if cur_pos is not None:
                    self.labels[f'label_motor_value_{index+1}'].setText(cur_pos)


    def get_tick_interval(self, pot_pos: int, step: int) -> int:
        return pot_pos // 100 * step
    
    
    def get_di_color(self, di_value: str) -> str:
        if di_value == '1':
            return 'red'
        else:
            return 'green'
        
        
    def initUI(self):
        self.angle = 0
        
        for identifier in range(1, 9 + 1):
            self.set_init_slot(identifier)
            self.Slider_valueChanged(identifier)
            
            
        self.Slider_1.setMinimum(0)
        self.Slider_1.setMaximum(100)
        
        
        self.Slider_6.setInvertedAppearance(True)
        self.Slider_7.setInvertedAppearance(True)
        self.Slider_9.setInvertedAppearance(True)
        
        
    def set_timer(self, identifier: int):
        timer_name = f'motor_{identifier}_timer'
        
        if hasattr(self, f"motor_{identifier}_timer") is False:
            setattr(self, timer_name, QTimer(self))
            getattr(self, timer_name).timeout.connect(self.update_ui)
            
            getattr(self, timer_name).timeout.connect(lambda: self.change_value(identifier))

             
    def set_init_slot(self, identifier: int):
        button_list = [
            f"pushButton_cw_{identifier}",
            f"pushButton_ccw_{identifier}",
        ]
        command_list = [
            "cmd_cw",
            "cmd_ccw",
        ]
        button_command = dict(zip(button_list, command_list))
        
        for button_name, command in button_command.items():
            button = getattr(self, button_name)
            button.pressed.connect(lambda cmd=command, ident=identifier: self.on_button_pressed(cmd, ident))
            button.released.connect(lambda cmd="cmd_stop", ident=identifier: self.on_button_released(cmd, ident))

        
    def on_button_pressed(self, command, identifier: int):
        getattr(self, command)(identifier)
        
        
    def on_button_released(self, command, identifier: int):
        getattr(self, command)(identifier)
        
        
    def draw_rect(self, qp):
        qp.setPen(QPen(Qt.blue, 1, Qt.DashLine))
        qp.drawRect(0, 0, 127, 127)
        qp.setPen(QPen(Qt.red, 10))
        qp.drawPoint(64, 64)


    def cmd_cw(self, identifier: int):
        self.change_direction = 1
        
        motor_timer = getattr(self, f"motor_{identifier}_timer")
        motor_timer.start(10)
        
        motorctl.MBID = f"{motorID_list[(identifier-1)]}"
        motorctl.CMD = MotorCmdField.WMOVE.name
        motorctl.MODE = ServoParam.MOVE_TYPE_ABS.value
        motorctl.POS = "100"
        motorctl.SPD = "100"
        motorctl.ACC = "1000"
        motorctl.DECC = "1000"
        
        motorctl.MakeCommand() 
        
        
    def cmd_ccw(self, identifier: int):
        self.change_direction = -1
        
        motor_timer = getattr(self, f"motor_{identifier}_timer")
        motor_timer.start(10)
    
        motorctl.MBID = f"{motorID_list[(identifier-1)]}"
        motorctl.CMD = MotorCmdField.WMOVE.name
        motorctl.MODE = ServoParam.MOVE_TYPE_ABS.value
        motorctl.POS = "-100"
        motorctl.SPD = "100"
        motorctl.ACC = "1000"
        motorctl.DECC = "1000"
        
        motorctl.MakeCommand() 
    
    
    def cmd_stop(self, identifier: int):
        getattr(self, f"motor_{identifier}_timer").stop()
        

    def change_value(self, identifier: int):
        
        if identifier == 4 or identifier == 5 or identifier == 8:
            self.angle += self.change_direction
                                         
            getattr(self, f"label_motor_value_{identifier}").setText(str(self.angle))    
            
            transform = QTransform().rotate(self.angle)
            
            painter = QPainter(self.pixmap)
            self.draw_rect(painter)
            painter.end()
            
            rotated_pixmap = self.pixmap.transformed(transform, Qt.SmoothTransformation).scaled(128, 128, Qt.IgnoreAspectRatio, Qt.SmoothTransformation)
            getattr(self, f"Wheel_{identifier}").setPixmap(rotated_pixmap) 
            
        else:
            _slider = getattr(self, f"Slider_{identifier}")
            current_value = _slider.value()
            getattr(self, f"label_motor_value_{identifier}").setText(str(current_value))    

            if self.change_direction == 1 and current_value < _slider.maximum():
                _slider.setValue(current_value + 1)
                
            elif self.change_direction == -1 and current_value > _slider.minimum():
                _slider.setValue(current_value - 1)      
                      
            else:
                getattr(self, f"motor_{identifier}_timer").stop()
                
                
    def Slider_valueChanged(self, identifier: int):
        
        getattr(self, f"Slider_{identifier}").valueChanged.connect(
            lambda: self.sliderMoved(identifier))
    
    
    def sliderMoved(self, identifier: int):
        # 라벨 텍스트 업데이트\
        strValue= str(getattr(self, f"Slider_{identifier}").value())
        motorctl.MBID = f"{motorID_list[(identifier-1)]}"
        motorctl.CMD = MotorCmdField.WMOVE.name
        motorctl.MODE = f"{ServoParam.MOVE_TYPE_ABS.value}"
        motorctl.POS = strValue
        motorctl.SPD = "300"
        motorctl.ACC = "1000"
        motorctl.DECC = "1000"
        
        motorctl.MakeCommand() 

             
    def cmd_setting_load(self, fileName: str):
        """
        LOAD 버튼을 누르면 설정값을 불러옵니다.
        """

        if fileName:
            self.setWindowTitle(os.path.basename(fileName))

            with open(fileName, 'r') as file:
                settings_list = json.load(file)
                print(settings_list)
                
                
if __name__ == "__main__":
    rospy.init_node("TacTracer_Bumblebee_Module_Tester", anonymous=True )
    app = QApplication(sys.argv)
    widget = bumblebeeModuleTester()
    widget.show()

    sys.exit(app.exec_())
