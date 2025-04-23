import sympy as sp
from Util import *
from UtilBLB import *
pulseCount = 50
spd1st = 500
rpmtime = calculate_rpm_time_accdesc(pulseCount, spd1st, 1,1)
print(f'첫번째 모터를 {spd1st} 속도로 {pulseCount} 바퀴 돌릴때 {rpmtime}초 걸립니다 ')

pulseCount2nd = 100
print(f'두번째 모터는 rpm {calculate_targetRPM_fromtime(pulseCount2nd,rpmtime)} 으로 설정해야 합니다.')
print(calculate_rpm_time_accdesc(10, 300, 1000,1000))

def calculate_second_motor_rpm(n, s1, a1, d1, m, a2, d2):
    # 첫 번째 모터의 가속, 정상 상태, 감속 시간 계산
    t_accel_1 = s1 / a1  # 첫 번째 모터의 가속 시간
    t_decel_1 = s1 / d1  # 첫 번째 모터의 감속 시간

    # 첫 번째 모터가 가속 및 감속하는 동안 회전하는 거리
    dist_accel_1 = 0.5 * a1 * t_accel_1**2
    dist_decel_1 = 0.5 * d1 * t_decel_1**2

    # 정상 상태에서 회전하는 거리
    dist_constant_1 = n - dist_accel_1 - dist_decel_1

    # 정상 상태에서 회전하는 시간
    t_constant_1 = dist_constant_1 / s1

    # 첫 번째 모터의 총 시간
    t_total_1 = t_accel_1 + t_constant_1 + t_decel_1

    # 두 번째 모터의 목표 RPM을 계산하기 위한 방정식 설정
    target_rpm = sp.symbols("target_rpm")

    # 두 번째 모터의 가속, 감속 시간 계산
    t_accel_2 = target_rpm / a2
    t_decel_2 = target_rpm / d2

    # 두 번째 모터가 가속 및 감속하는 동안 회전하는 거리
    dist_accel_2 = 0.5 * a2 * t_accel_2**2
    dist_decel_2 = 0.5 * d2 * t_decel_2**2

    # 두 번째 모터의 정상 상태에서 회전하는 시간 방정식 설정
    t_constant_2 = t_total_1 - t_accel_2 - t_decel_2

    # 두 번째 모터의 총 회전 거리 방정식
    equation = sp.Eq(m, dist_accel_2 + target_rpm * t_constant_2 + dist_decel_2)

    # 방정식 풀기
    solution = sp.solve(equation, target_rpm)

    # 현실적인 값을 얻기 위해 양수 해를 반환
    for sol in solution:
        if sol > 0:
            target_rpm_value = float(sol)
            t_accel_2_value = target_rpm_value / a2
            t_decel_2_value = target_rpm_value / d2
            t_constant_2_value = t_total_1 - t_accel_2_value - t_decel_2_value
            t_total_2 = t_accel_2_value + t_constant_2_value + t_decel_2_value

            # 각 시간들을 출력하여 디버깅
            print(f"t_accel_1: {t_accel_1}, t_constant_1: {t_constant_1}, t_decel_1: {t_decel_1}")
            print(f"t_accel_2_value: {t_accel_2_value}, t_constant_2_value: {t_constant_2_value}, t_decel_2_value: {t_decel_2_value}")
            
            return target_rpm_value, t_total_2 * 1000  # 밀리초 단위로 변환

# 예제 호출
n = 200  # 첫 번째 모터의 총 회전수
s1 = 1500  # 첫 번째 모터의 목표 RPM
a1 = 1000  # 첫 번째 모터의 가속도
d1 = 1000  # 첫 번째 모터의 감속도
m = 100  # 두 번째 모터의 총 회전수
a2 = 1000  # 두 번째 모터의 가속도
d2 = 1000  # 두 번째 모터의 감속도

second_motor_rpm, total_time_ms = calculate_second_motor_rpm(n, s1, a1, d1, m, a2, d2)
print(f"두 번째 서보모터의 목표 RPM: {second_motor_rpm:.1f}")
print(f"총 시간: {total_time_ms:.1f} ms")

initial_rpm = 0
final_rpm = 1000
time_seconds = 5

# 평균 RPM 계산
average_rpm = (initial_rpm + final_rpm) / 2

# 초당 회전수 계산
revolutions_per_second = average_rpm / 60

# 총 회전수 계산
total_revolutions = revolutions_per_second * time_seconds

print(f"{time_seconds}초 동안의 총 회전수: {total_revolutions:.2f} 바퀴")
