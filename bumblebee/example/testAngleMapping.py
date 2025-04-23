def angle_to_pulse(angle):
    # 서보모터 특성
    min_pulse = -1885000
    max_pulse = 150000
    total_pulse_range = max_pulse - min_pulse
    
    # 펄스 0을 기준으로 한 각도 조정
    # 0 펄스에서의 각도 계산
    zero_pulse_angle = (abs(min_pulse) / total_pulse_range) * 360
    
    # 입력 각도 조정
    adjusted_angle = (angle + zero_pulse_angle) % 360
    
    # 조정된 각도를 펄스로 변환
    pulse = min_pulse + (adjusted_angle / 360) * total_pulse_range
    
    # 정수로 반올림
    return round(pulse)

# 테스트
for angle in [0, 90, 180, 270, 360]:
    pulse = angle_to_pulse(angle)
    print(f"각도: {angle}° -> 펄스: {pulse}")