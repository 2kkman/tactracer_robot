def calculate_rpm_change(r2, t2, rpm1):
    # 2번 모터의 일정한 RPM 계산
    rpm2 = (r2 / t2) * 60
    
    # 1번 모터가 처음 t2/2 동안 회전하는 회전수 계산
    t_half = t2 / 2
    revolutions_first_half = (rpm1 / 60) * t_half
    
    # 1번 모터가 남은 t2/2 동안 회전해야 하는 회전수 계산
    revolutions_needed = r2 - revolutions_first_half
    
    # 1번 모터가 남은 t2/2 동안 필요한 RPM 계산
    rpm1_2 = (revolutions_needed / t_half) * 60
    
    return rpm1_2

# 예제 호출
r2 = 200  # 2번 모터의 목표 회전수
t2 = 10  # 동작 완료에 걸리는 시간 (초)
rpm2 = 60*r2/t2
rpm1 = 500  # 1번 모터의 초기 RPM

rpm1_2 = calculate_rpm_change(r2, t2, rpm1)
print(f"모터의 초기 속도는 {rpm1},{rpm2} RPM이고, t2/2 시점에서 속도를 {rpm1_2:.2f} RPM으로 변경해야 합니다.")
