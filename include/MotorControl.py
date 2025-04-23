import rospy
from std_msgs.msg import String, Header
from Util import *
from UtilBLB import *
class MOTOR_CONTROL_CMD():
    def __init__(self):
        self.MBID = None
        self.MODE = None
        self.POS = None
        self.SPD = None
        self.ACC = None
        self.DECC = None
        self.CMD = None
        
        self.publisher = rospy.Publisher(TopicName.CMD_DEVICE.name, String, queue_size=10)
        
    def MakeCommand(self):
        """명령어 생성
        """
        try: 
            isabsolute = True if self.MODE == str(ServoParam.MOVE_TYPE_ABS.value) else False 
            current_cmd = []
            if self.CMD == MotorCmdField.WMOVE.name:
                current_cmd = getMotorMoveString(self.MBID, isabsolute, self.POS, self.SPD, self.ACC, self.DECC)
            elif self.CMD == MotorCmdField.WSPD.name:
                current_cmd = getMotorSpeedString(self.MBID, isabsolute, self.SPD, self.ACC, self.DECC)
            elif self.CMD in [MotorCmdField.WSETUP2.name,  
                              MotorCmdField.WSETUP.name,   
                              MotorCmdField.WSTOP.name,
                              MotorCmdField.WALM_C.name,
                              MotorCmdField.WOFF.name,
                              MotorCmdField.WZERO.name,
                              MotorCmdField.WINIT.name,
                              MotorCmdField.WINITREVERSE.name,
                              MotorCmdField.WLIMIT.name,
                              MotorCmdField.WLOC.name,
                              MotorCmdField.WLIMIT_OFF.name,
                              MotorCmdField.WCALI.name,]:
                current_cmd = getMotorDefaultString(self.MBID, self.CMD)

        except ValueError as e:
            rospy.logerr(f"Error in MakeCommand: {e}")
            
        self.MotorTopic(current_cmd)     
           
    def calculateSpeed(self, angle) -> str:
        base_speed = 100
        max_speed = 1500
        speed_increment = 100
        
        speed = base_speed + (abs(angle) * speed_increment)
        if speed > max_speed:
            speed = max_speed

        # 최소 속도 제한 (예: 0)
        min_speed = 0
        if speed < min_speed:
            speed = min_speed

        return speed    
       
    def MotorRightFastMove(self):
        self.MBID = '30'
        self.MODE = '65'
        self.POS = '100000'
        self.SPD = '1000'
        self.ACC = '50'
        self.DECC = '100'
        self.CMD = 'WMOVE'
        self.MakeCommand()

    def MotorRightMove(self, angle):
        self.MBID = '30'
        self.MODE = '65'
        self.POS = '50000'
        self.SPD = self.calculateSpeed(angle)
        self.ACC = '40'
        self.DECC = '1000'
        self.CMD = 'WMOVE'
        # print(type())
        self.MakeCommand()
        
    def MotorRightSlowMove(self):
        self.MBID = '30'
        self.MODE = '65'
        self.POS = '100000'
        self.SPD = '100'
        self.ACC = '10'
        self.DECC = '100'
        self.CMD = 'WMOVE'
        self.MakeCommand()
        
    def MotorLeftFastMove(self):
        self.MBID = '29'
        self.MODE = '65'
        self.POS = '100000'
        self.SPD = '1000'
        self.ACC = '50'
        self.DECC = '100'
        self.CMD = 'WMOVE'
        self.MakeCommand()
        
    def MotorLeftMove(self, angle):
        self.MBID = '29'
        self.MODE = '65'
        self.POS = '50000'
        self.SPD = self.calculateSpeed(angle)
        self.ACC = '40'
        self.DECC = '1000'
        self.CMD = 'WMOVE'
        self.MakeCommand()
        
    def MotorLeftSlowMove(self):
        self.MBID = '29'
        self.MODE = '65'
        self.POS = '100000'
        self.SPD = '100'
        self.ACC = '50'
        self.DECC = '100'
        self.CMD = 'WMOVE'
        self.MakeCommand()         
           
    def MotorRightStop(self):
        self.MBID = '30'
        self.CMD = 'WSTOP'
        self.MakeCommand()
        
    def MotorLeftStop(self):
        self.MBID = '29'
        self.CMD = 'WSTOP'
        self.MakeCommand()
        
    def MotorAllStop(self):
        self.MotorRightStop()
        self.MotorLeftStop()
    
    def MotorDualFastMove(self):
        self.MotorRightFastMove()
        self.MotorLeftFastMove()
        
    def MotorDualMove(self):
        self.MotorRightMove()
        self.MotorLeftMove()   
                
    def MotorDualSlowMove(self):
        self.MotorRightSlowMove()
        self.MotorLeftSlowMove()   
            
    def MotorTopic(self, message):
        # 발행할 메시지 생성
        # message.data = 'MBID:30,MODE:65,POS:-1000000,SPD:10,ACC:20,DECC:20,CMD:Wmove'
        try:
            rospy.logdebug_once(message)
            # 메시지 발행
            self.publisher.publish(message) 
        except ValueError as e:
            rospy.logerr(f"Error in MotorTopic: {e}")
            
class MOTOR_DRIVER():
    def __init__(self):
        self.MBID = None
        self.MODE = None
        self.POS = None
        self.SPD = None
        self.ACC = None
        self.DECC = None
        self.CMD = None
        self.prev_command = None
        self.motorctl = MOTOR_CONTROL_CMD()
        
    def EmergencyStop(self, mbid=ModbusID.MOTOR_H.value):
        current_cmd = getMotorDefaultString(mbid, MotorCmdField.WSTOP.name)
        self.motorctl.MotorTopic(current_cmd)
        return current_cmd
    
    def EmergencyStop_once(self, mbid=ModbusID.MOTOR_H.value):
        current_cmd = getMotorDefaultString(mbid, MotorCmdField.WSTOP.name)
        if current_cmd != self.prev_command:
            self.prev_command = current_cmd
            self.motorctl.MotorTopic(current_cmd)
        else:
            return
        
    def SetHomePosition(self, mbid: str = ModbusID.MOTOR_H.value):
        current_cmd = getMotorDefaultString(mbid, MotorCmdField.WZERO.name)
        self.motorctl.MotorTopic(current_cmd)
        return current_cmd
        
    def SetSpeedTarget(self, mbid: str = ModbusID.MOTOR_H.value, speed: int = 300, acc: int = 300, decc:int = 300):
        current_cmd = getMotorSpeedString(mbid, speed, acc, decc)
        self.motorctl.MotorTopic(current_cmd)
        return current_cmd
    
    def SetSpeedTarget_once(self, mbid: str = ModbusID.MOTOR_H.value, speed: int = 300, acc: int = 300, decc: int = 300):
        current_cmd = getMotorSpeedString(mbid, speed, acc, decc)
        if current_cmd != self.prev_command:
            self.prev_command = current_cmd
            self.motorctl.MotorTopic(current_cmd)
        else:
            return
    
    def SetMoveTarget(self, isMode: bool=True, mbid=ModbusID.MOTOR_H.value, pos=0, speed=300, acc=300, decc=300):
        current_cmd = getMotorMoveString(mbid, isMode, pos, speed, acc, decc)
        return current_cmd
    
    def SetDecelerateToStop(self, isMode: bool=True, mbid=ModbusID.MOTOR_H.value, pos=0, speed=300, acc=3000, decc=3000, pos2=10000):
        
        pos = pos if pos is not None else 0
        speed = speed if speed is not None else 300
        acc = acc if acc is not None else 3000
        decc = decc if decc is not None else 3000
        pos2 = pos2 if pos2 is not None else 10000
        try:              
            if int(pos) < pos2:
                stop_position = int(pos) + pos2
            else:   
                stop_position = int(pos) - pos2
            
            current_cmd = getMotorMoveString(mbid, isMode, stop_position, speed, acc, decc)
            return current_cmd
        except Exception as e:
            rospy.logerr(f"Error in SetDecelerateToStop: {e}")    