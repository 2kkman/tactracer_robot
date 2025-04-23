#!/usr/bin/env python3
import smbus
import time, sys
import rospy
import roslib
from std_msgs.msg import String
from sensor_msgs.msg import Temperature
from sensor_msgs.msg import RelativeHumidity

MeasureCmd = [0x33, 0x00]
global seq

def talker():
      frame_id_Temp = 'UtilTemp'
      frame_id_Humid = 'UtilHumid'
      Temp_msg = Temperature()
      Humid_msg = RelativeHumidity()

      #pub = rospy.Publisher('chatter',String,queue_size=10)
      pub2 = rospy.Publisher(frame_id_Temp,Temperature,queue_size=10)
      pub3 = rospy.Publisher(frame_id_Humid,RelativeHumidity,queue_size=10)
      global seq
      Temp_msg.header.frame_id = frame_id_Temp
      Temp_msg.variance = Humid_msg.variance = 0
      
      rate = rospy.Rate(1) #send 1 time per second
      while not rospy.is_shutdown():
        bus.write_i2c_block_data(0x38, 0xAC, MeasureCmd)
        time.sleep(0.5)
        data = bus.read_i2c_block_data(0x38,0x00)
        #print(data)
        temp = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]
        ctemp = ((temp*200) / 1048576) - 50
        print(ctemp)
        result_Temp =u'Temperature: {0:.1f}°C'.format(ctemp) 
        print(result_Temp)
        tmp = ((data[1] << 16) | (data[2] << 8) | data[3]) >> 4
        #print(tmp)
        #ctmp = int(tmp * 100 / 1048576)
        ctmp = tmp /1048576
        result_Hum = u'Humidity: {0}%'.format(ctmp) 
        print(ctmp)
        Humid_msg.relative_humidity = ctmp
        Temp_msg.temperature = ctemp
        print(result_Hum)
        hello_str = f'[{result_Temp},{result_Hum}]'

        seq+=1 
        Humid_msg.header.frame_id = frame_id_Humid
        Temp_msg.header.seq = Humid_msg.header.seq = seq
        Temp_msg.header.stamp =Humid_msg.header.stamp = rospy.Time.now()
        
        rospy.loginfo(Temp_msg)
        rospy.loginfo(Humid_msg)
        pub2.publish(Temp_msg)
        pub3.publish(Humid_msg)
        #pub.publish(hello_str)
        
        rate.sleep()

if __name__=='__main__':
    try:
        seq = 0
        bus = smbus.SMBus(1) # Rev 2 Pi uses 1
        config = [0x08, 0x00]
        bus.write_i2c_block_data(0x38, 0xE1, config)
        time.sleep(0.5)
        byt = bus.read_byte(0x38)
        #print(byt&0x68)
        
        #print(locals())
        rospy.init_node('talker', anonymous = True)
        talker()
    except rospy.ROSInterruptException:
        pass
