import rospy
from std_msgs.msg import String
from ttacer.parser import *

def callback(data):
    # 수신한 문자열 메시지 출력
    # rospy.loginfo("Received message: %s", data.data)
    # 수신한 문자열 메시지 파싱
    # parsed_data = data.data.split(":")
    # key = parsed_data[0]
    # value = parsed_data[1]
    # print(f'{data.data}')
    # sCmd = GetKey(data.data, chSplit1)
    # sParam = GetVal(data.data, chSplit1)
    # print(f'sCmd: {sCmd}, sParam: {sParam}')
    # 파싱한 데이터 출력
    # rospy.loginfo(f"Key: {sCmd}, Value: {sParam}")
    parsed_data = GetParseData(data.data)
    
    # print(parsed_data['state'])
    # 파싱한 데이터 출력
    # print(f'{parsed_data}')
    for key, value in parsed_data.items():
        print(f'key: {key}, value: {value}')
            
    rospy.loginfo(f"{parsed_data}")
    
def publish_message():
    # 발행자(Publisher) 생성
    pub = rospy.Publisher('string_topic', String, queue_size=10)
    
    # 루프 속도 설정 (10Hz)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # 발행할 문자열 메시지 생성
        message = "Hello, World!"
        
        # 문자열 메시지 발행
        pub.publish(message)
        
        # 루프 속도 유지
        rate.sleep()

def subscribe_to_topic():
    # 구독자(Subscriber) 생성
    rospy.Subscriber('DeliveryTable_topic', String, callback)
    
    # 루프 유지
    rospy.spin()

if __name__ == '__main__':
    # 노드 초기화
    rospy.init_node('topic_subscriber_publisher', anonymous=True)
    
    try:
        # 동시에 실행하기 위해 멀티스레드로 실행
        import threading
        pub_thread = threading.Thread(target=publish_message)
        pub_thread.start()
        
        # 토픽 구독
        subscribe_to_topic()
        
    except rospy.ROSInterruptException:
        pass
