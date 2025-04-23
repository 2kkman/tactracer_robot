import rospy
from datetime import datetime

def getTimestamp():
    current_time = rospy.get_rostime()
    
    # 년-월-일 시:분:초 포맷 예시
    # timestamp_str = datetime.fromtimestamp(current_time.to_sec()).strftime('%Y-%m-%d %H:%M:%S')
    
    # ISO 8601 확장형 포맷 예시
    #timestamp_str = datetime.fromtimestamp(current_time.to_sec()).isoformat(timespec='milliseconds')
    
    # 년월일_시분초 포맷 예시
    timestamp_str = datetime.fromtimestamp(current_time.to_sec()).strftime('%Y%m%d_%H%M%S')
    #timestamp_str = datetime.fromtimestamp(current_time.to_sec()).strftime('%Y-%m-%d %H:%M:%S')
    return timestamp_str

def setTimestamp():
    pass
    # sResult['time'] = getTimestamp() 
    