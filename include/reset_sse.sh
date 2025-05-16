#!/bin/bash

# 인자가 없으면 사용법 출력
if [ $# -eq 0 ]; then
    echo "사용법: $0 <포트번호1> [포트번호2] [포트번호3] ..."
    echo "예시: $0 8080 3000 5000"
    exit 1
fi

# 각 포트번호에 대해 처리
for PORT in "$@"; do
    # 포트 번호가 유효한 숫자인지 확인
    if ! [[ "$PORT" =~ ^[0-9]+$ ]]; then
        echo "경고: '$PORT'는 유효한 포트 번호가 아닙니다. 건너뜁니다."
        continue
    fi
    
    echo "포트 $PORT를 사용하는 프로세스를 찾는 중..."
    
    # 포트를 사용하는 프로세스 찾기
    PIDS=$(sudo lsof -t -i :$PORT)
    
    if [ -z "$PIDS" ]; then
        echo "포트 $PORT를 사용하는 프로세스가 없습니다."
        continue
    fi
    
    echo "포트 $PORT를 사용하는 프로세스 종료 중..."
    
    # 모든 관련 프로세스 종료
    for PID in $PIDS; do
        echo "프로세스 $PID 종료 중..."
        sudo kill -9 $PID
        if [ $? -eq 0 ]; then
            echo "프로세스 $PID를 성공적으로 종료했습니다."
        else
            echo "프로세스 $PID 종료 실패!"
        fi
    done
    
    echo "포트 $PORT 처리 완료"
    echo "------------------------"
done

echo "모든 지정된 포트의 프로세스 종료 작업이 완료되었습니다."

rosnode kill /BLB_RFID
rosnode kill /ITX_and
# HOSTNAME에 ITX 문자열이 포함되어 있는지 확인
# if [[ "$HOSTNAME" == *"ITX"* ]]; then
#     # ITX가 포함되어 있으면 아래 명령어 실행
#     roslaunch tta_blb BLB_RFID_ESP32.launch &
# fi
/usr/bin/python3 /root/catkin_ws/src/tactracer_robot/bumblebee/flask_ros/run.py &
/usr/bin/python3 /root/catkin_ws/src/tactracer_robot/bumblebee/ros_fastapi_bridge/main.py &
roslaunch tta_blb BLB_ANDROID.launch &
