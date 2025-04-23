#!/usr/bin/env python3
import smbus
import time, sys
import rospy
import roslib
import datetime
from varname import *
from max30105 import MAX30105, HeartRate
from std_msgs.msg import Header
from std_msgs.msg import String

max30105 = MAX30105()
param_DUST_show = True

def getROS_Header(frame_id_str):
  msgTmp = Header()
  msgTmp.frame_id = frame_id_str
  return msgTmp

def talker():
      global seq
      global param_DUST_show
      pub = rospy.Publisher('UtilDust', String, queue_size=10)

      data = []
      means = []
      rate = rospy.Rate(1) #send 1 time per second

      while not rospy.is_shutdown():
        # if rospy.has_param(nameof(param_DUST_show)):
        #     param_DUST_show = rospy.get_param(nameof(param_DUST_show))
        # else:
        #     rospy.set_param(nameof(param_DUST_show), param_DUST_show)

        samples = max30105.get_samples()
        if samples is not None:
          r = samples[2] & 0xff
          d = hr.low_pass_fir(r)
          data.append(d)
          if len(data) > mean_size:
              data.pop(0)
          mean = sum(data) / float(len(data))
          means.append(mean)
          if len(means) > delta_size:
              delta = means[-1] - means[-delta_size]
          else:
              delta = 0
          if delta > threshold:
              detected = True
          else:
              detected = False
          temp = max30105.get_temperature()
          #hello_str = f'DUST:{d},MEAN:{mean:.2f},DELTA:{delta:.2f},CHANGED:{detected}'
          hello_str = f'DUSTU:{d},DUSTU_TEMPC:{temp:.2f}'
          if param_DUST_show is True:
            rospy.loginfo(hello_str)

          pub.publish(hello_str)

        rate.sleep()

if __name__=='__main__':
    try:
        seq = 0
        max30105.setup(leds_enable=3)
        max30105.set_led_pulse_amplitude(1, 0.0)
        max30105.set_led_pulse_amplitude(2,f 0.0)
        max30105.set_led_pulse_amplitude(3, 12.5)

        max30105.set_slot_mode(1, 'red')
        max30105.set_slot_mode(2, 'ir')
        max30105.set_slot_mode(3, 'green')
        max30105.set_slot_mode(4, 'off')

        hr = HeartRate(max30105)

        # Smooths wobbly data. Increase to increase smoothing.
        #default : 20
        mean_size = 10

        # Compares current smoothed value to smoothed value x
        # readings ago. Decrease this to increase detection
        # speed.
        #default : 10
        delta_size = 5 #default : 10

        # The delta threshold at which a change is detected.
        # Decrease to make the detection more sensitive to
        # fluctuations, increase to make detection less
        # sensitive to fluctuations.
        threshold = 10

        rospy.init_node('node_Dust', anonymous = True)
        talker()
    except rospy.ROSInterruptException:
        pass
