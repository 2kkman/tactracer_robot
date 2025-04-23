from enum import Enum, auto
class API_METHOD_NAME(Enum):
    robot_node_info = auto()

class Config:
    DEBUG = False
    SSL_CERT_PATH = '/root/cert.pem'
    SSL_KEY_PATH = '/root/key.pem'
    API_SVR_IP = '127.0.0.1'
    API_ANDROID_IP = '172.30.1.8'
    API_SVR_URL = f'https://{API_SVR_IP}:9000'


    
class API_FIELD_MOTOR(Enum):
  motor_info = auto()
  motor_id = auto()
  motor_name = auto()
  err = auto()
  err_code = auto()
  position = auto()
  speed = auto()
  stop = auto()
    
class API_FIELD_INFO(Enum):
  id = auto()
  model = auto()
  master_ip_address = auto()
  master_host_name = auto()
  slave_ip_address = auto()
  slave_host_name = auto()
  activated_date = auto()
  expiry_eate = auto()
  onlineStatus = auto()
  robot_status = auto()
  robot_status_code = auto()
  video_name = auto()

class API_FIELD_BLOCK(Enum):
  block_x = auto()
  block_y = auto()
  blocked = auto()
  block_reason_code = auto()
  blocked_tagid = auto()
  

class API_FIELD_MOVE_STATUS(Enum):
  angle = auto()
  x = auto()
  y = auto()
  vx = auto()
  vy = auto()

class API_FIELD_BATTERY_STATUS(Enum):
  robot_charge_node = auto()
  charging = auto()
  auto_charging = auto()
  battery_level = auto() #소수점으로 : 0.94 = 94%
  battery_temp = auto() #섭씨온도
  voltage = auto()
  
class API_FIELD_MOVE(Enum):
    error_code = auto()
    error_msg = auto()


class API_FIELD_TARGET(Enum):
    node_type = auto()
    node_code = auto()
    tag_serialno = auto()
    aruco_id = auto()

class API_FIELD_TRAY(Enum):
    trayid = auto()
    tray_x = auto()
    tray_y = auto()
    loading = auto()
    unloading = auto()
    

class API_NAVI_INFO(Enum):
    current_task = auto()
    start_station = auto()
    current_node = auto()
    next_node = auto()
    final_station = auto()
    task_status = auto()
    finished_path = auto()
    unfinished_path = auto()
    containers = auto()
    rack_id = auto()
    desc = auto()
    user_msg = auto()
    has_goods = auto()
    
    
    
    
class robot_alarm_status:
    error_code = ''
    error_msg = ''
    extra2 = ''
