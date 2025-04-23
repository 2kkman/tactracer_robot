import pandas as pd
import numpy as np

def interpolate_pos_abs(df):
    """
    연속된 노드들 사이의 distance를 기반으로 POS_ABS 값을 보간합니다.
    
    Parameters:
    df (pandas.DataFrame): 원본 데이터프레임
    
    Returns:
    pandas.DataFrame: POS_ABS가 보간된 데이터프레임
    """
    # 데이터프레임 복사
    result_df = df.copy()
    
    # 0값을 가진 행들의 인덱스를 찾습니다 (마지막 행 제외)
    zero_indices = result_df[result_df['POS_ABS'] == 0].index[:-1]
    
    # 각 구간별로 처리
    current_start_idx = 0
    while current_start_idx < len(result_df):
        # 다음 비제로 값을 찾습니다
        next_idx = current_start_idx + 1
        while next_idx < len(result_df) and result_df.iloc[next_idx]['POS_ABS'] == 0:
            next_idx += 1
            
        # 구간의 끝에 도달했거나 마지막 행인 경우 중단
        if next_idx >= len(result_df):
            break
            
        # 시작과 끝 값을 가져옵니다
        start_pos = result_df.iloc[current_start_idx]['POS_ABS']
        end_pos = result_df.iloc[next_idx]['POS_ABS']
        
        # 구간 내 전체 거리를 계산합니다
        total_distance = sum(result_df.iloc[current_start_idx:next_idx+1]['distance'])
        
        # 누적 거리를 계산하고 비율에 따라 값을 할당합니다
        cumulative_distance = 0
        for idx in range(current_start_idx + 1, next_idx):
            cumulative_distance += result_df.iloc[idx-1]['distance']
            ratio = cumulative_distance / total_distance
            interpolated_value = start_pos + (end_pos - start_pos) * ratio
            result_df.at[idx, 'POS_ABS'] = round(interpolated_value)
        
        current_start_idx = next_idx
    
    return result_df

# 데이터프레임 생성
df = pd.DataFrame({
    'distance': [1000, 1000, 1000, 1000, 1000, 1205, 1000],
    'POS_ABS': [-100851, 0, 0, 0, 0, 324681, 0]
})

# 함수 실행
result = interpolate_pos_abs(df)
print(result)