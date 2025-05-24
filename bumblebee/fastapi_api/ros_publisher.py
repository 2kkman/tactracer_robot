import rospy
from std_msgs.msg import String

# 토픽명 -> 퍼블리셔 객체를 캐싱해 재사용
publisher_cache = {}

def publish_string(topic_name: str, data: str) -> str:
    if topic_name not in publisher_cache:
        # 퍼블리셔 생성 (queue_size=10)
        publisher_cache[topic_name] = rospy.Publisher(topic_name, String, queue_size=10)
        rospy.loginfo(f"[ROS] Publisher created for topic: {topic_name}")

    pub = publisher_cache[topic_name]
    rospy.loginfo(f"[ROS] Publishing to {topic_name}: {data}")
    pub.publish(data)
    return f"Published to {topic_name}: {data}"
