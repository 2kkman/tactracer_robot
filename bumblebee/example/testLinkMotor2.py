import numpy as np
import matplotlib.pyplot as plt
import math

def mapRangeExp(y, in_min, in_max, out_min, out_max, k):
    """
    지수 매핑 함수로 입력 범위 내의 값을 출력 범위로 변환한다.
    
    Parameters:
    y (float): 변환하려는 원래 값
    in_min (float): 입력 범위의 최소값
    in_max (float): 입력 범위의 최대값
    out_min (float): 출력 범위의 최소값
    out_max (float): 출력 범위의 최대값
    k (float): 지수 변환의 강도를 조절하는 계수
    
    Returns:
    float: 변환된 값
    """    
    # y의 범위를 0에서 1 사이로 정규화
    norm_y = (y - in_min) / (in_max - in_min)
    
    # 정규화된 값을 지수 함수에 적용하고 정규화된 입력 범위로 변환
    #exp_value = (math.exp(norm_y * k * math.log(math.e)) - 1) / (math.e - 1)
    exp_value = (math.exp(norm_y * k) - 1) / (math.exp(k) - 1)
    # 입력 범위로 매핑
    output_value = exp_value * (out_max - out_min) + out_min
    
    return output_value

# y 값을 0에서 250000까지 100 단위로 생성
y_values = np.arange(0, 250001, 100)
mapped_values = [mapRangeExp(y, 0, 250000, 0, 120000, 1.5) for y in y_values]

# 그래프 그리기
plt.figure(figsize=(10, 6))
plt.plot(y_values, mapped_values, label='Exponential Mapping')
plt.xlabel('Input Value (y)')
plt.ylabel('Mapped Value')
plt.title('Exponential Mapping of Input Values to Output Range')
plt.legend()
plt.grid(True)
plt.show()
#plt.savefig('/mnt/data/exponential_mapping_new.png')
