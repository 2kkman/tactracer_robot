import math

def mapRangeExp(y, in_min, in_max, out_min, out_max, k):
    norm_y = (y - in_min) / (in_max - in_min)
    exp_value = (math.exp(norm_y * k * math.log(math.e)) - 1) / (math.e - 1)
    output_value = exp_value * (out_max - out_min) + out_min
    return output_value

def calculate_speed_adjustment(cur_pos_arm1, cur_pos_arm2, not_cur_arm1, pot_cur_arm1, not_cur_arm2, pot_cur_arm2, spd_cur_arm1, spd_cur_arm2, k, time_seconds):
    # 펄스 계산 비율
    pulse_rate = 10000 / 60  # 1분에 10000 펄스

    # 모터2의 예상 위치 계산
    expected_pos_arm2 = cur_pos_arm2 + spd_cur_arm2 * pulse_rate * time_seconds
    print(f'모터2의 {time_seconds}초 후 예상 위치 계산: {expected_pos_arm2 : .1f}')

    # 모터1의 목표 위치 계산 (모터2의 예상 위치 기반)
    target_tmp_arm1 = mapRangeExp(expected_pos_arm2, not_cur_arm2, pot_cur_arm2, not_cur_arm1, pot_cur_arm1, k)
    print(f'모터1의 목표 위치 계산 (모터2의 예상 위치 기반): {target_tmp_arm1 : .1f}')

    # 모터1의 예상 위치
    expected_pos_arm1 = cur_pos_arm1 + spd_cur_arm1 * pulse_rate * time_seconds
    print(f'모터1의 {time_seconds}초 후 예상 위치: {expected_pos_arm1 : .1f}')

    # 필요한 총 펄스 계산
    needed_pulses = target_tmp_arm1 - expected_pos_arm1
    print(f'필요한 총 펄스 계산: {needed_pulses : .1f}')

    # 추가적으로 필요한 RPM 계산
    additional_rpm = needed_pulses / (pulse_rate * time_seconds)
    print(f'추가적으로 필요한 RPM 계산: {additional_rpm : .1f}')

    # 새로운 RPM 계산
    new_rpm = spd_cur_arm1 + additional_rpm
    print(f'새로운 RPM 계산: {new_rpm : .1f}')

    return new_rpm

# 예시 변수 설정
cur_pos_arm1 = 0   # 현재 모터1 위치
cur_pos_arm2 = 0   # 현재 모터2 위치
not_cur_arm1 = 0      # 모터1 최소 위치
pot_cur_arm1 = 500000  # 모터1 최대 위치
not_cur_arm2 = 0      # 모터2 최소 위치
pot_cur_arm2 = 2000000  # 모터2 최대 위치
spd_cur_arm1 = 100     # 현재 모터1의 RPM
spd_cur_arm2 = 300     # 현재 모터2의 RPM
k = 1                 # 지수 매핑 계수
time_seconds = 1      # 시간 간격 (초)

# 필요한 RPM 계산
new_rpm = calculate_speed_adjustment(cur_pos_arm1, cur_pos_arm2, not_cur_arm1, pot_cur_arm1, not_cur_arm2, pot_cur_arm2, spd_cur_arm1, spd_cur_arm2, k, time_seconds)
print("New RPM for Motor 1:", new_rpm)
