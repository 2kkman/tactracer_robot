import sys
import os
import json

from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog
from PyQt5.QtCore import QDate

import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import Imu, Temperature, RelativeHumidity
from Util import *
from UtilBLB import *
from MotorControl import MOTOR_CONTROL_CMD

from ui.bumblebeeTest_ui import Ui_MainWindow
import time
import threading
from tf.transformations import quaternion_from_euler


recvDataMap = {}
dic_485ex = {}  # 모터 모니터링 데이터
execute_time = getDateTime()
seq = 0


class BumblebeeTester(QMainWindow, Ui_MainWindow):

    def __init__(self):
        super(BumblebeeTester, self).__init__()
        self.date = QDate.currentDate()
        self.setupUi(self)
        self.set_init()

    def set_init_slot(self, identifier: int):
        """슬롯 초기화

        각 버튼에 대한 슬롯을 초기화합니다.

        """

        slot_list = [
            "di_NOT",
            "di_POT",
            "di_ESTOP",
            "di_HOME",
            "operating_RDY",
            "operating_RUN",
            "operating_ERR",
            "operating_HOME",
            "operating_POSITION",
            "operating_SPEED",
        ]
        for slot in slot_list:
            if slot.startswith("di"):
                getattr(self, f"{slot}_{identifier}").textChanged.connect(
                    self.set_sensor_gui
                )

            elif slot.startswith("operating"):
                getattr(self, f"{slot}_{identifier}").textChanged.connect(
                    self.set_state_gui
                )
            getattr(self, f"{slot}_{identifier}").setText("")

        button_list = [
            "pushButton_RESET",
            "pushButton_ALARM",
            "pushButton_START",
            "pushButton_DIRECTION",
            "pushButton_READ",
            "pushButton_STOP",
            "pushButton_HOME",
            "pushButton_ZERO",
        ]
        command_list = [
            "cmd_reset",
            "cmd_alarm_clear",
            "cmd_start",
            "change_direction",
            "cmd_read",
            "cmd_stop",
            "cmd_home",
            "cmd_zero",
        ]

        button_command = dict(zip(button_list, command_list))
        for button_name, command in button_command.items():
            button = getattr(self, f"{button_name}_{identifier}")
            button.clicked.connect(getattr(self, command))

        lineEdit_POS = getattr(self, f"lineEdit_POS_{identifier}")
        lineEdit_POS.textChanged.connect(self.update_button_direction)  # update button direction
        

        lcdNumber_list = [
            "lcdNumber_BUS_VOLTAGE",
            "lcdNumber_CUR_CURRENT",
            "lcdNumber_CUR_TORQUE",
            "lcdNumber_CUR_SPD",
            "lcdNumber_CMD_POS",
            "lcdNumber_CUR_POS",
        ]

        for lcdNumber in lcdNumber_list:
            getattr(self, f"{lcdNumber}_{identifier}").display("")

        
    def set_init(self):
        """
        초기화
        """
        global motorctl
        motorctl = MOTOR_CONTROL_CMD()
        
        for identifier in range(1, 10):
            self.set_init_slot(identifier)
            self.set_init_button(identifier)
            self.set_init_mbid(identifier)
            self.set_init_command(identifier)
            self.set_init_value(identifier)
        
        pushButton_EXECUTE = getattr(self, f"pushButton_EXECUTE")
        pushButton_EXECUTE.clicked.connect(self.cmd_execute)  # Sequence Execute

        pushButton_SAVE = getattr(self, f"pushButton_SAVE")
        pushButton_SAVE.clicked.connect(self.cmd_setting_save)  # Save
        
        pushButton_LOAD = getattr(self, f"pushButton_LOAD")
        pushButton_LOAD.clicked.connect(self.cmd_setting_load)  # Load  
        
        self.checkBox_SAVE_ALL.stateChanged.connect(self.cmd_setting_save_all)
        self.checkBox_SAVE_ALL.setChecked(True)
        
    def set_init_command(self, identifier: int):
        """커맨드 초기화

        Args:
            identifier (int): line number
        """
        comboBox_CMD = getattr(self, f"comboBox_CMD_{identifier}")
        comboBox_CMD.clear()
        for cmd in MotorCmdField:
            comboBox_CMD.addItem(cmd.name)
        
        # MotorCmdField.WCALI.INDEX = 8
        comboBox_CMD.removeItem(8)
        
        
    def set_init_mbid(self, identifier: int):
        """MBID 초기화

        Args:
            identifier (int): line number
        """
        comboBox_MBID = getattr(self, f"comboBox_MBID_{identifier}")
        comboBox_MBID.clear()
        for i in range(3, 33):
            comboBox_MBID.addItem(f"MB_{i}")
        
    def set_init_button(self, identifier: int):
        """버튼 사용여부 초기화

        Args:
            identifier (int): line number
        """
        self.update_button_disenable(identifier)

    def set_init_value(self, identifier: int):
        """초기값 설정

        Args: 
            identifier (int): line number
        """

        comboBox_CMD = getattr(self, f"comboBox_CMD_{identifier}")
        comboBox_MODE = getattr(self, f"comboBox_MODE_{identifier}", None)
        lineEdit_POS = getattr(self, f"lineEdit_POS_{identifier}", None)
        lineEdit_SPD = getattr(self, f"lineEdit_SPD_{identifier}", None)
        lineEdit_ACC = getattr(self, f"lineEdit_ACC_{identifier}", None)
        lineEdit_DECC = getattr(self, f"lineEdit_DECC_{identifier}", None)

        lineEdit_POS.setText("100000")
        lineEdit_SPD.setText("1000")
        lineEdit_ACC.setText("1000")
        lineEdit_DECC.setText("1000")
        comboBox_CMD.setCurrentText(MotorCmdField.WMOVE.name)
        comboBox_MODE.setCurrentIndex(1)

    def set_init_flag(self, identifier: int):
        """플래그 초기화
        """
        getattr(self, f"subscriber{identifier}", None)

    def get_ros_header(self, frame_id_str: str):
        """ ROS 헤더 생성
        """
        msgTmp = Header()
        msgTmp.frame_id = frame_id_str
        return msgTmp

    def publish_ImuData(self, dicARD: dict, frame_id_Range):
        """IMU 데이터 발행
        """
        global seq

        Imu_msg = Imu()
        Imu_msg.header = self.get_ros_header(frame_id_Range)
        GAV_SVN = dicARD.get(TRAY_ARD_Field.GAV_SVN.name, None)
        GLA_SVN = dicARD.get(TRAY_ARD_Field.GLA_SVN.name, None)
        GOR_SVN = dicARD.get(TRAY_ARD_Field.GOR_SVN.name, None)
        if GAV_SVN == None or GLA_SVN == None or GOR_SVN == None:
            return

        GAV_SVNarr = GAV_SVN.split(sDivItemComma)  # angular_velocity
        GLA_SVNarr = GLA_SVN.split(sDivItemComma)  # linear_acceleration
        GOR_SVNarr = GOR_SVN.split(sDivItemComma)  # orientation , roll,pitch,yaw

        roll = float(GOR_SVNarr[0])
        pitch = float(GOR_SVNarr[1])
        yaw = float(GOR_SVNarr[2])
        quaternion = quaternion_from_euler(roll, pitch, yaw)

        # 쿼터니언 설정
        Imu_msg.orientation.x = quaternion[0]
        Imu_msg.orientation.y = quaternion[1]
        Imu_msg.orientation.z = quaternion[2]
        Imu_msg.orientation.w = quaternion[3]

        Imu_msg.linear_acceleration.x = float(GLA_SVNarr[0])
        Imu_msg.linear_acceleration.y = float(GLA_SVNarr[1])
        Imu_msg.linear_acceleration.z = float(GLA_SVNarr[2])

        Imu_msg.angular_velocity.x = float(GAV_SVNarr[0])
        Imu_msg.angular_velocity.y = float(GAV_SVNarr[1])
        Imu_msg.angular_velocity.z = float(GAV_SVNarr[2])
        Imu_msg.header.stamp = rospy.Time.now()
        Imu_msg.header.seq = seq

        self.pub_IMU.publish(Imu_msg)

        intPitch = int(pitch)
        if intPitch > 0:
            print(intPitch)

            motorctl.MotorLeftMove(intPitch)
            motorctl.MotorRightStop()

        elif intPitch < 0:
            print(intPitch)

            motorctl.MotorRightMove(intPitch)
            motorctl.MotorLeftStop()

        elif intPitch == 0:
            print(intPitch)
            motorctl.MotorAllStop()
        print("------------------")
        seq += 1

    def cmd_alarm_clear(self):
        """알람 클리어 명령

        모터의 알람을 클리어합니다.
        """
        sender = self.sender()
        if sender is None:
            return

        identifier = "".join(filter(str.isdigit, sender.objectName()))

        comboBox_MBID = getattr(self, f"comboBox_MBID_{identifier}")

        motorctl.MBID = comboBox_MBID.currentText().split("_")[1]
        motorctl.CMD = MotorCmdField.WALM_C.name
        motorctl.MakeCommand()

    def cmd_execute(self):
        """
        시퀀스 실행

        그룹박스에서 시퀀스를 선택하고 실행 버튼을 누르면 해당 시퀀스의 모터들을 순차적으로 실행합니다.
        """
        seqs = [
            [],
            [],
            [],
            [],
        ]
        for identifier in range(1, 10):  # 우선순위 1부터 9까지
            groupBox_DIRECTION = getattr(self, f"groupBox_{identifier}")
            for radioButtons in groupBox_DIRECTION.children():
                if radioButtons.objectName().startswith("rb_SEQ"):
                    if radioButtons.isChecked():
                        self.update_button_enable(identifier)
                        if radioButtons.objectName().startswith("rb_SEQ1"):
                            seqs[0].append(identifier)
                        elif radioButtons.objectName().startswith("rb_SEQ2"):
                            seqs[1].append(identifier)
                        elif radioButtons.objectName().startswith("rb_SEQ3"):
                            seqs[2].append(identifier)
                        elif radioButtons.objectName().startswith("rb_SEQ4"):
                            seqs[3].append(identifier)

                else:
                    continue
        print(f"seqs: {seqs}")
        
        thread = threading.Thread(target=self.cmd_execute_line, args=(seqs,), daemon=True)
        thread.start()
        
    def cmd_execute_line(self, seqs: List[int]):
        """시퀀스 라인별 실행

        시퀀스 라인별로 실행하며, 라인에 포함된 모터들을 동시에 실행합니다.

        Args:
            seqs (dict): sequence dictionary
        """
        global execute_time
        print(f"Total Seq.map : {seqs}, dic485 : {dic_485ex}")
        for seq in seqs: 
            # 현재 시퀀스의 모든 모터 명령 실행
            threads = []
            threadsEnd = []
            print(f"Remain threads :  {seq}")
            execute_time = getDateTime()
            for identifier in seq:
                print(f"Thread identifier : {identifier} started")
                
                thread = threading.Thread(target=self.start_motor, args=(identifier,), daemon=True)
                thread.start()
                threads.append(thread)
                threadsEnd.append(False)

            # 모든 쓰레드가 완료될 때까지 기다림
            bAllThreadEnd = False
            while not bAllThreadEnd:
                loopCnt = 0
                for thread in threads:
                    t_alive = thread.is_alive()
                    threadsEnd[loopCnt] = not t_alive
                    print(
                        f"Checking threads : {thread}:{t_alive}, threadsEnd = {threadsEnd}"
                    )
                    if t_alive:
                        time.sleep(0.01)  

                    loopCnt += 1
                if all(threadsEnd):
                    bAllThreadEnd = True

    def start_motor(self, identifier: int):
        """모터 시작
        """
        global execute_time
        # 모터 시작 명령
        self.set_execute_command(identifier)

        print(f"startMotor called with {identifier}")
        # 모터 완료 상태 확인
        while True:
            
            comboBox_MBID = getattr(self, f"comboBox_MBID_{identifier}")
            mbid = comboBox_MBID.currentText().split("_")[1]
            try:
                # 딕셔너리 내부의 딕셔너리 접근 시도
                mbid_data = dic_485ex[mbid]
                mbid_finish_status = mbid_data.get(MonitoringField.ST_CMD_FINISH.name, None)
                mbid_running_status = dic_485ex[mbid].get(MonitoringField.ST_RUNNING.name, None)
            except KeyError:
                # mbid 키가 dic_485ex에 존재하지 않을 경우
                print(f"KeyError: '{mbid}' not found in dic_485ex")
                mbid_finish_status = None
                mbid_running_status = None
                
            isMotorStopped = isTrue(mbid_finish_status) or not isTrue(mbid_running_status)
            statusStr = f"mbid_status : {MonitoringField.ST_CMD_FINISH.name}:{mbid_finish_status}, {MonitoringField.ST_RUNNING.name}:{mbid_running_status}"
            
            if isMotorStopped and isTimeExceeded(execute_time, 100):
                print(
                    f"{identifier} : Exit loop MBID : {mbid}, {statusStr}"
                )
                
                break  # 모터 작업 완료
            else:
                print(
                    f"{identifier} : Wait loop MBID : {mbid}, {statusStr}"
                )
            time.sleep(1)      
        
    def update_button_direction(self):
        """lineEdit_POS의 입력값에 따른 버튼 업데이트

        양수면 CW, 음수면 CCW 버튼의 텍스트를 업데이트합니다.
        """
        for identifier in range(1, 10):
            lineEdit_POS = getattr(self, f"lineEdit_POS_{identifier}")
            pushButton_DIRECTION = getattr(self, f"pushButton_DIRECTION_{identifier}")

            text = lineEdit_POS.text()
            try:
                value = float(text)
                direction = "CW" if value > 0 else "CCW"
            except ValueError:
                direction = "Invalid"
            pushButton_DIRECTION.setText(direction)

    def change_direction(self):
        """모터 방향 변경

        CW, CCW 버튼을 누르면 모터의 방향을 변경합니다.
        """
        sender = self.sender()
        if sender is None:
            return

        identifier = "".join(filter(str.isdigit, sender.objectName()))

        lineEdit_POS = getattr(self, f"lineEdit_POS_{identifier}")
        posValue = int(lineEdit_POS.text())

        isplus = self.isPlus(posValue)
        if isplus:
            lineEdit_POS.setText(f"{-posValue}")
        else:
            lineEdit_POS.setText(f"{-posValue}")

    def isPlus(self, value: int):
        """양수 여부 확인

        Args:
            value (int): value

        Returns:
            bool: True if value is plus
        """
        if value >= 0:
            return True
        else:
            return False

    def get_codeMODE(self, mode: str) -> str:
        """모드를 코드로 변환

        Args:
            mode (str): mode

        Returns:
            str: mode code
        """
        if mode == "Absolute":
            return f"{ServoParam.MOVE_TYPE_ABS.value}"
        elif mode == "Relative":
            return f"{ServoParam.MOVE_TYPE_REL.value}"

    def set_codeMODE(self, mode: str) -> str:
        """모드를 코드로 변환

        Args:
            mode (str): mode

        Returns:
            str: mode code
        """
        if mode == "1":
            return "Absolute"
        elif mode == "65":
            return "Relative"
        
    def cmd_start(self):
        """START 버튼 동작
        
        버튼을 누르면 CMD에 설정되 명령어를 시작합니다.
        """
        sender = self.sender()
        if sender is None:
            return

        identifier = "".join(filter(str.isdigit, sender.objectName()))
        # 커맨드 설정 및 실행
        self.set_execute_command(identifier)

    def set_execute_command(self, identifier: int):
        """커맨드 설정 및 실행

        입력되어있는 설정값을 가지고 실행합니다.
        Args:
            identifier (str): line number
            cmd_text: 명령어에 따라 실행되는 명령어가 설정이 변경됨
        """

        # UI 요소 참조
        comboBox_MBID = getattr(self, f"comboBox_MBID_{identifier}")
        comboBox_CMD = getattr(self, f"comboBox_CMD_{identifier}")
        comboBox_MODE = getattr(self, f"comboBox_MODE_{identifier}", None)
        lineEdit_POS = getattr(self, f"lineEdit_POS_{identifier}", None)
        lineEdit_SPD = getattr(self, f"lineEdit_SPD_{identifier}", None)
        lineEdit_ACC = getattr(self, f"lineEdit_ACC_{identifier}", None)
        lineEdit_DECC = getattr(self, f"lineEdit_DECC_{identifier}", None)

        # MBID 설정
        motorctl.MBID = comboBox_MBID.currentText().split("_")[1]
        cmd_text = comboBox_CMD.currentText()

        # 명령어에 따른 동작 설정

        motorctl.MODE = (
            self.get_codeMODE(comboBox_MODE.currentText()) if comboBox_MODE else None
        )
        motorctl.POS = lineEdit_POS.text() if lineEdit_POS else None
        motorctl.SPD = lineEdit_SPD.text() if lineEdit_SPD else None
        motorctl.ACC = lineEdit_ACC.text() if lineEdit_ACC else None
        motorctl.DECC = lineEdit_DECC.text() if lineEdit_DECC else None

        motorctl.CMD = cmd_text

        # 명령 생성 및 실행
        motorctl.MakeCommand()
        
    def cmd_setting_save_all(self):
        """ SAVE ALL 버튼동작
        CheckBox_ALL을 누르면 모든 Save CheckBox를 동일하게 설정합니다.
        """
        for identifier in range(1, 10):
            checkBox_SAVE = getattr(self, f"checkBox_SAVE_{identifier}")
            checkBox_SAVE.setChecked(self.checkBox_SAVE_ALL.isChecked())  
            
    def cmd_setting_save(self):
        """
        SAVE 버튼을 누르면 설정값을 저장합니다.
        """
        setting_value_list = []
        for identifier in range(1, 10): 
            checkBox_SAVE = getattr(self, f"checkBox_SAVE_{identifier}")

            if checkBox_SAVE.isChecked():
                comboBox_CMD = getattr(self, f"comboBox_CMD_{identifier}")
                comboBox_MBID = getattr(self, f"comboBox_MBID_{identifier}")
                comboBox_MODE = getattr(self, f"comboBox_MODE_{identifier}", None)
                lineEdit_POS = getattr(self, f"lineEdit_POS_{identifier}", None)
                lineEdit_SPD = getattr(self, f"lineEdit_SPD_{identifier}", None)
                lineEdit_ACC = getattr(self, f"lineEdit_ACC_{identifier}", None)
                lineEdit_DECC = getattr(self, f"lineEdit_DECC_{identifier}", None)
                
                setting_value = {
                "CMD": comboBox_CMD.currentText() if comboBox_CMD else None,
                "MBID": comboBox_MBID.currentText().split("_")[1] if comboBox_MBID else None,
                "MODE": self.get_codeMODE(comboBox_MODE.currentText()) if comboBox_MODE else None,
                "POS": lineEdit_POS.text() if lineEdit_POS else None,
                "SPD": lineEdit_SPD.text() if lineEdit_SPD else None,
                "ACC": lineEdit_ACC.text() if lineEdit_ACC else None,
                "DECC": lineEdit_DECC.text() if lineEdit_DECC else None
            }
            # Clean up the dictionary to remove None values if you want to exclude them
                setting_value = {k: v for k, v in setting_value.items() if v is not None}
            else:
                setting_value = {}
            setting_value_list.append(setting_value)

        setting_file_path = "/tactracer_robot/bumblebee_gui/config"
        cwd_file_path = os.path.join(os.getcwd()+setting_file_path)
        options = QFileDialog.Options()
        fileName, _ = QFileDialog.getSaveFileName(self, 
                                                  "Save File As", 
                                                  cwd_file_path, 
                                                  "All Files (*);;JSON Files (*.json);;Text Files (*.txt)", 
                                                  options=options)
        if fileName:
        # Writing the values to a JSON file in the current working directory
            with open(fileName, 'w') as file:
                json.dump(setting_value_list, file)
        
    def cmd_setting_load(self):
        """
        LOAD 버튼을 누르면 설정값을 불러옵니다.
        """
        options = QFileDialog.Options()
        setting_file_path = "/tactracer_robot/bumblebee_gui/config/"
        cwd_file_path = os.path.join(os.getcwd()+setting_file_path)
        fileName, _ = QFileDialog.getOpenFileName(self, 
                                                  "BumblebeeTester Config Load", 
                                                  cwd_file_path, 
                                                  "All Files (*);;JSON Files (*.json);;Text Files (*.txt)", 
                                                  options=options)
        # 파일 경로에서 파일 이름 추출
        
        if fileName:
            self.setWindowTitle(os.path.basename(fileName))

            with open(fileName, 'r') as file:
                settings_list = json.load(file)
                self.update_setting_value(settings_list)
                
    def update_setting_value(self, setting_value_list: List[dict]):
        """
        파일에서 불러온 설정값을 업데이트 합니다.
        """
        for identifier in range(1, 9 + 1):
            setting_value = setting_value_list[identifier-1]
            comboBox_CMD = getattr(self, f"comboBox_CMD_{identifier}")
            comboBox_MBID = getattr(self, f"comboBox_MBID_{identifier}")
            comboBox_MODE = getattr(self, f"comboBox_MODE_{identifier}", None)
            lineEdit_POS = getattr(self, f"lineEdit_POS_{identifier}", None)
            lineEdit_SPD = getattr(self, f"lineEdit_SPD_{identifier}", None)
            lineEdit_ACC = getattr(self, f"lineEdit_ACC_{identifier}", None)
            lineEdit_DECC = getattr(self, f"lineEdit_DECC_{identifier}", None)
            
            comboBox_CMD.setCurrentText(setting_value.get("CMD", ""))
            result_MBID = "MB_" + setting_value.get("MBID", "")      
            comboBox_MBID.setCurrentText(result_MBID)
            comboBox_MODE.setCurrentText(self.set_codeMODE(setting_value.get("MODE", "")))
            lineEdit_POS.setText(setting_value.get("POS", ""))
            lineEdit_SPD.setText(setting_value.get("SPD", ""))
            lineEdit_ACC.setText(setting_value.get("ACC", ""))
            lineEdit_DECC.setText(setting_value.get("DECC", ""))
                        
    def cmd_home(self):
        """홈 복귀 명령
        지정해 놓은 홈 위치로 모터를 이동시킵니다.
        """
        sender = self.sender()
        if sender is None:
            return

        identifier = "".join(filter(str.isdigit, sender.objectName()))

        # UI 요소 참조
        comboBox_MBID = getattr(self, f"comboBox_MBID_{identifier}")
        lineEdit_SPD = getattr(self, f"lineEdit_SPD_{identifier}", None)
        lineEdit_ACC = getattr(self, f"lineEdit_ACC_{identifier}", None)
        lineEdit_DECC = getattr(self, f"lineEdit_DECC_{identifier}", None)

        motorctl.MBID = comboBox_MBID.currentText().split("_")[1]
        motorctl.CMD = MotorCmdField.WMOVE.name
        motorctl.MODE = self.get_codeMODE("Absolute")
        motorctl.POS = "0"
        motorctl.SPD = lineEdit_SPD.text() if lineEdit_SPD else None
        motorctl.ACC = lineEdit_ACC.text() if lineEdit_ACC else None
        motorctl.DECC = lineEdit_DECC.text() if lineEdit_DECC else None

        motorctl.MakeCommand()
        
    def cmd_zero(self):
        """영점 명령
        현재 위치를 영점으로 지정.
        """
        sender = self.sender()
        if sender is None:
            return
        
        identifier = self.get_button_identifier(sender.objectName())
        
        comboBox_MBID = getattr(self, f"comboBox_MBID_{identifier}", None)
        motorctl.MBID = comboBox_MBID.currentText().split("_")[1]
        motorctl.CMD = MotorCmdField.WZERO.name
        motorctl.MakeCommand()
        
    def cmd_stop(self):
        """정지 명령
        
        모터를 정지시킵니다.
        """
        sender = self.sender()
        if sender is None:
            return
        identifier = self.get_button_identifier(sender.objectName())
        comboBox_MBID = getattr(self, f"comboBox_MBID_{identifier}", None)

        motorctl.MBID = comboBox_MBID.currentText().split("_")[1]
        motorctl.CMD = MotorCmdField.WSTOP.name
        motorctl.MakeCommand()

    def cmd_calibration(self):
        """캘리브레이션 명령
        
        모터의 캘리브레이션을 수행합니다.
        """
        sender = self.sender()
        if sender is None:
            return
        identifier = self.get_button_identifier(sender.objectName())
        comboBox_MBID = getattr(self, f"comboBox_MBID_{identifier}", None)

        motorctl.MBID = comboBox_MBID.currentText().split("_")[1]
        motorctl.CMD = MotorCmdField.WCALI.name
        motorctl.MakeCommand()

    def cmd_reset(self):
        """RESET 명령

        모터의 상태를 초기화 하고 READ 버튼을 활성화합니다.
        """
        sender = self.sender()
        if sender is None:
            return
        identifier = self.get_button_identifier(sender.objectName())

        self.resetState(identifier)
        self.stop_subscribe(identifier)
        self.update_button_disenable(identifier)

    def resetState(self, identifier: int):
        """모터 상태 초기화

        모터의 상태를 초기화합니다.

        Args:
            identifier (int): line number
        """
        # UI 요소 참조
        reset_list = [
            "operating_RDY",
            "operating_RUN",
            "operating_ERR",
            "operating_HOME",
            "operating_POSITION",
            "operating_SPEED",
            "di_NOT",
            "di_POT",
            "di_ESTOP",
            "di_HOME",
            "lcdNumber_BUS_VOLTAGE",
            "lcdNumber_CUR_CURRENT",
            "lcdNumber_CUR_TORQUE",
            "lcdNumber_CUR_SPD",
            "lcdNumber_CMD_POS",
            "lcdNumber_CUR_POS",
        ]

        for reset in reset_list:
            if reset.startswith("operating"):
                getattr(self, f"{reset}_{identifier}").setText("")
            elif reset.startswith("di"):
                getattr(self, f"{reset}_{identifier}").setText("")
            elif reset.startswith("lcdNumber"):
                getattr(self, f"{reset}_{identifier}").display("")
            else:
                pass

    def cmd_read(self):
        """모터 상태 읽기
        MBID와 일치하는 모터의 상태를 읽어옵니다."""
        sender = self.sender()
        if sender is None:
            return

        button_number = self.get_button_identifier(sender.objectName())

        if button_number is not None:
            self.update_button_enable(button_number)

    def get_button_identifier(self, object_name) -> int:
        """번호 추출

        Args:
            identifier (int): line number

        Returns:
            _type_: int
            _description_: line number
            _value_: line number
        """
        # 객체 이름에서 숫자 추출
        number_part = "".join(filter(str.isdigit, object_name))
        return int(number_part) if number_part.isdigit() else None
    
    def update_button_disenable(self, identifier: int):
        """버튼 비활성화
        
        Args:
            identifier (int): line number
        """
        
        button_list = ["READ", "HOME", "STOP", "RESET", "ALARM", "START", "DIRECTION", "ZERO"]
        for button_name in button_list:
            button = getattr(self, f"pushButton_{button_name}_{identifier}")
            if button_name == "READ":
                button.setEnabled(True)
            else:
                button.setEnabled(False)

            if button_name == "DIRECTION":
                button.setText("CW")
                
    def update_button_enable(self, identifier: int):
        """버튼 동작 수행

        Args:
            identifier (int): line number
        """
        # 적절한 구독 토픽 설정
        getattr(self, f"ros_node_thread")(getattr(self, f"subscribeTopic{identifier}"))
        button_list = ["READ", "HOME", "STOP", "RESET", "ALARM", "START", "DIRECTION", "ZERO"]
        # 버튼 상태 업데이트
        for button_name in button_list:
            button = getattr(self, f"pushButton_{button_name}_{identifier}")
            if button_name == "READ":
                button.setEnabled(False)
            else:
                button.setEnabled(True)

    def ros_node_thread(self, task: str):
        """
        ROS 노드 스레드
        """
        sub_thread = threading.Thread(target=task, daemon=True)
        sub_thread.start()       
        
        sub_t_alive = sub_thread.is_alive()
        
        print(
            f"Checking threads : {sub_thread}:{sub_t_alive}"
        )
        if sub_t_alive:
            time.sleep(0.5)  

    def set_state_gui(self):
        """상태 표시"""
        sender = self.sender()
        if sender is None:
            return

        if sender.text() == "0":
            sender.setStyleSheet("background-color: red; color: red")
        elif sender.text() == "1":
            sender.setStyleSheet("background-color: green; color: green")
        else:
            sender.setStyleSheet("")

    def set_sensor_gui(self):
        """센서 상태 표시"""
        sender = self.sender()
        if sender is None:
            return

        if sender.text() == "0":
            sender.setStyleSheet("background-color: green; color: green")
        elif sender.text() == "1":
            sender.setStyleSheet("background-color: red; color: red")
        else:
            sender.setStyleSheet("")

    def subscribeTopic1(self):
        self.subscriber1 = rospy.Subscriber(
            self.comboBox_MBID_1.currentText(), String, self.callbackCmd1
        )
        rospy.spin()

    def subscribeTopic2(self):
        self.subscriber2 = rospy.Subscriber(
            self.comboBox_MBID_2.currentText(), String, self.callbackCmd2
        )
        rospy.spin()

    def subscribeTopic3(self):
        self.subscriber3 = rospy.Subscriber(
            self.comboBox_MBID_3.currentText(), String, self.callbackCmd3
        )
        rospy.spin()

    def subscribeTopic4(self):
        self.subscriber4 = rospy.Subscriber(
            self.comboBox_MBID_4.currentText(), String, self.callbackCmd4
        )
        rospy.spin()

    def subscribeTopic5(self):
        self.subscriber5 = rospy.Subscriber(
            self.comboBox_MBID_5.currentText(), String, self.callbackCmd5
        )
        rospy.spin()

    def subscribeTopic6(self):
        self.subscriber6 = rospy.Subscriber(
            self.comboBox_MBID_6.currentText(), String, self.callbackCmd6
        )
        rospy.spin()

    def subscribeTopic7(self):
        self.subscriber7 = rospy.Subscriber(
            self.comboBox_MBID_7.currentText(), String, self.callbackCmd7
        )
        rospy.spin()

    def subscribeTopic8(self):
        self.subscriber8 = rospy.Subscriber(
            self.comboBox_MBID_8.currentText(), String, self.callbackCmd8
        )
        rospy.spin()
        
    def subscribeTopic9(self):
        self.subscriber9 = rospy.Subscriber(
            self.comboBox_MBID_9.currentText(), String, self.callbackCmd9
        )
        rospy.spin()
        
    def stop_subscribe(self, identifier: int):
        subscriber = getattr(self, f"subscriber{identifier}")
        if subscriber is not None:
            subscriber.unregister()
            rospy.loginfo(f"{subscriber.name} Stopped.")

    def processKey(self, dicData, identifier: int):
        """키 처리
        수신된 데이터를 해당 라인의 키값과 일치하는 gui로 표시합니다.
        """
        keys_to_check = [
            "di_NOT",
            "di_POT",
            "di_ESTOP",
            "di_HOME",
            "ST_ENABLE",
            "ST_RDY",
            "ST_RUNNING",
            "ST_RUN",
            "ST_FAULTY",
            "ST_ERR",
            "ST_CMD_FINISH",
            "ST_HOME_OK",
            "ST_PATH_FINISH",
            "ST_INP",
            "ST_HOME_FINISH",
            "ST_AT_SPEED",
            "BUS_VOLTAGE",
            "CUR_CURRENT",
            "CUR_TORQUE",
            "CUR_SPD",
            "CMD_POS",
            "CUR_POS",
        ]

        state_mapping = {
            "di_NOT": "di_NOT",
            "di_POT": "di_POT",
            "di_ESTOP": "di_ESTOP",
            "di_HOME": "di_HOME",
            "ST_ENABLE": "operating_RDY",
            "ST_RDY": "operating_RDY",
            "ST_RUNNING": "operating_RUN",
            "ST_RUN": "operating_RUN",
            "ST_FAULTY": "operating_ERR",
            "ST_ERR": "operating_ERR",
            "ST_CMD_FINISH": "operating_HOME",
            "ST_HOME_OK": "operating_HOME",
            "ST_INP": "operating_POSITION",
            "ST_PATH_FINISH": "operating_POSITION",
            "ST_HOME_FINISH": "operating_SPEED",
            "ST_AT_SPEED": "operating_SPEED",
            "BUS_VOLTAGE": "lcdNumber_BUS_VOLTAGE",
            "CUR_CURRENT": "lcdNumber_CUR_CURRENT",
            "CUR_TORQUE": "lcdNumber_CUR_TORQUE",
            "CUR_SPD": "lcdNumber_CUR_SPD",
            "CMD_POS": "lcdNumber_CMD_POS",
            "CUR_POS": "lcdNumber_CUR_POS",
        }
        current_mapping = {
            "BUS_VOLTAGE": "lcdNumber_BUS_VOLTAGE",
            "CUR_CURRENT": "lcdNumber_CUR_CURRENT",
            "CUR_TORQUE": "lcdNumber_CUR_TORQUE",
            "CUR_SPD": "lcdNumber_CUR_SPD",
            "CMD_POS": "lcdNumber_CMD_POS",
            "CUR_POS": "lcdNumber_CUR_POS",
        }
        

        for key in keys_to_check:
            value = dicData.get(key.upper())
            if value:
                widget_suffix = state_mapping.get(key, key)
                widget_name = f"{widget_suffix}_{identifier}"
                if key in current_mapping:
                    try:
                        getattr(self, widget_name).display(value)
                    except AttributeError as e:
                        print(f"AttributeError: {e}, widget_name: {widget_name}")
                else:
                    try:
                        getattr(self, widget_name).setText(value)
                    except AttributeError as e:
                        print(f"AttributeError: {e}, widget_name: {widget_name}")


    def callbackCmd1(self, data):
        global recvDataMap

        recvDataMap = getDic_strArr(data.data, sDivFieldColon, sDivItemComma)
        mbid = recvDataMap.get(MotorWMOVEParams.MBID.name, None)
        if mbid != None:
            dic_485ex[mbid] = recvDataMap
        self.processKey(dic_485ex[mbid], "1")

    def callbackCmd2(self, data):
        global recvDataMap

        recvDataMap = getDic_strArr(data.data, sDivFieldColon, sDivItemComma)
        mbid = recvDataMap.get(MotorWMOVEParams.MBID.name, None)

        if mbid != None:
            dic_485ex[mbid] = recvDataMap

        self.processKey(dic_485ex[mbid], "2")

    def callbackCmd3(self, data):
        global recvDataMap

        recvDataMap = getDic_strArr(data.data, sDivFieldColon, sDivItemComma)
        mbid = recvDataMap.get(MotorWMOVEParams.MBID.name, None)

        if mbid != None:
            dic_485ex[mbid] = recvDataMap

        self.processKey(dic_485ex[mbid], "3")

    def callbackCmd4(self, data):
        global recvDataMap

        recvDataMap = getDic_strArr(data.data, sDivFieldColon, sDivItemComma)
        mbid = recvDataMap.get(MotorWMOVEParams.MBID.name, None)

        if mbid != None:
            dic_485ex[mbid] = recvDataMap

        self.processKey(dic_485ex[mbid], "4")

    def callbackCmd5(self, data):
        global recvDataMap

        recvDataMap = getDic_strArr(data.data, sDivFieldColon, sDivItemComma)
        mbid = recvDataMap.get(MotorWMOVEParams.MBID.name, None)

        if mbid != None:
            dic_485ex[mbid] = recvDataMap

        self.processKey(dic_485ex[mbid], "5")

    def callbackCmd6(self, data):
        global recvDataMap

        recvDataMap = getDic_strArr(data.data, sDivFieldColon, sDivItemComma)
        mbid = recvDataMap.get(MotorWMOVEParams.MBID.name, None)

        if mbid != None:
            dic_485ex[mbid] = recvDataMap

        self.processKey(dic_485ex[mbid], "6")

    def callbackCmd7(self, data):
        global recvDataMap

        recvDataMap = getDic_strArr(data.data, sDivFieldColon, sDivItemComma)
        mbid = recvDataMap.get(MotorWMOVEParams.MBID.name, None)

        if mbid != None:
            dic_485ex[mbid] = recvDataMap

        self.processKey(dic_485ex[mbid], "7")

    def callbackCmd8(self, data):
        global recvDataMap

        recvDataMap = getDic_strArr(data.data, sDivFieldColon, sDivItemComma)
        mbid = recvDataMap.get(MotorWMOVEParams.MBID.name, None)

        if mbid != None:
            dic_485ex[mbid] = recvDataMap

        self.processKey(dic_485ex[mbid], "8")

    def callbackCmd9(self, data):
        global recvDataMap

        recvDataMap = getDic_strArr(data.data, sDivFieldColon, sDivItemComma)
        mbid = recvDataMap.get(MotorWMOVEParams.MBID.name, None)

        if mbid != None:
            dic_485ex[mbid] = recvDataMap

        self.processKey(dic_485ex[mbid], "9")

if __name__ == "__main__":
    rospy.init_node("TacTracer_Bumblebee_Tester", anonymous=True)
    app = QApplication(sys.argv)
    widget = BumblebeeTester()
    widget.show()

    sys.exit(app.exec_())
