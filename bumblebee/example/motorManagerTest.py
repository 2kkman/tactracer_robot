import math
from UtilBLB import *
      
# 초기 데이터
initial_data = [
    {
        "MBID": "11",
        "CMD": "WMOVE",
        "MODE": "1",
        "POS": "529368",
        "SPD": "111",
        "ACC": "500",
        "DECC": "500",
        "TIME": 1.081081081081081
    },
    {
        "MBID": "13",
        "CMD": "WMOVE",
        "MODE": "1",
        "POS": "1505681",
        "SPD": "1800",
        "ACC": "300",
        "DECC": "1000",
        "TIME": 0.13333333333333333
    },
    {
        "MBID": "10",
        "CMD": "WMOVE",
        "MODE": "1",
        "POS": "1003787",
        "SPD": "1260",
        "ACC": "300",
        "DECC": "800",
        "TIME": 0.14285714285714285
    }
]

# MotorCommandManager 인스턴스 생성
manager = MotorCommandManager(initial_data)

listmbid = ['11','10','13']

# 1. 특정 MBID와 키로 값 조회
print("MBID 11의 SPD:", manager.get("11", "SPD"))  # 출력: 111

# 1. 특정 MBID와 키로 값 조회
print(f"{listmbid} SPD:", manager.get(listmbid, "SPD"))  # 출력: 111

# 2. 데이터 업데이트
manager.update("11", "SPD", "200")
print("업데이트 후 MBID 11의 SPD:", manager.get("11", "SPD"))  # 출력: 200

# 3. 새 명령 추가
new_command = {
    "MBID": "14",
    "CMD": "WMOVE",
    "MODE": "1",
    "POS": "100000",
    "SPD": "500",
    "ACC": "200",
    "DECC": "200",
    "TIME": 0.5
}
manager.add_command(new_command)
print("명령 추가 후 총 명령 수:", manager.get_command_count())  # 출력: 4

# 4. 명령 제거
manager.remove_command(listmbid)
print("명령 제거 후 총 명령 수:", manager.get_command_count())  # 출력: 3

# 5. 모든 명령 출력
print("\n모든 명령:")
for command in manager.get_all_commands():
    print(command)

# # 6. 평균 속도 계산
# print("\n평균 속도:", manager.get_average_speed())