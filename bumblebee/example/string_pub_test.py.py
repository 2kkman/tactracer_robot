#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def generate_payload(size_kb):
    return "A" * (size_kb * 1024)

def publish_string(payload, rate_hz):
    pub = rospy.Publisher('/test_string_topic', String, queue_size=10)
    rospy.init_node('string_pub_test', anonymous=True)
    rate = rospy.Rate(rate_hz)

    msg = String()
    msg.data = payload

    rospy.loginfo(f"Starting publishing at {rate_hz}Hz | Payload size: {len(payload)} bytes")
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        size_kb = int(input("Enter message size in KB (e.g., 1, 10, 100): "))
        rate_hz = int(input("Enter desired publish rate in Hz (e.g., 10, 100, 1000): "))
        payload = generate_payload(size_kb)
        publish_string(payload, rate_hz)
    except rospy.ROSInterruptException:
        pass
