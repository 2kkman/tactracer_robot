from scipy.interpolate import interp1d
# import numpy as np
from UtilBLB import *
dirPath = getConfigPath(UbuntuEnv.ITX.name)
dfAruco = pd.DataFrame()
strFileAruco = f"{dirPath}/Table_aruco.txt"
CUR_POS = 3500000
DISTANCE = pulseH_to_distance(CUR_POS)

def PrintDFAruco(dfReceived):
  columns_to_keep = [field.name for field in MAP_ARUCO if field.name in dfReceived.columns]
  print(dfReceived[columns_to_keep].to_string())  

if isFileExist(strFileAruco):
    dfAruco = pd.read_csv(strFileAruco, delimiter=sDivTab)

def estimate_cur_pos(data, marker_value: int, x_value: float) -> dict:
    # Filter data based on the specified MARKER_VALUE
    marker_data = data[data['MARKER_VALUE'] == marker_value]
    
    # If there's no data for the specified MARKER_VALUE, return None or an empty dict
    if marker_data.empty:
        return {}
    
    # Prepare for interpolation: extract X and CUR_POS columns for the specified MARKER_VALUE
    x_values = marker_data['X'].values
    cur_pos_values = marker_data['CUR_POS'].values
    
    # If there's only one unique X value, interpolation cannot proceed, so use that CUR_POS value directly
    if len(np.unique(x_values)) == 1:
        interpolated_cur_pos = cur_pos_values[0]
    else:
        # Linear interpolation of CUR_POS based on X
        interpolation_func = interp1d(x_values, cur_pos_values, kind='linear', fill_value='extrapolate')
        interpolated_cur_pos = interpolation_func(x_value)
    
    # Convert interpolated_cur_pos to a scalar if it's an ndarray
    if isinstance(interpolated_cur_pos, np.ndarray):
        interpolated_cur_pos = interpolated_cur_pos.item()
        
    # Calculate average values for other columns based on MARKER_VALUE
    averages = marker_data.mean(numeric_only=True).to_dict()
    print(type(interpolated_cur_pos))
    print(interpolated_cur_pos)
    # Construct the result dictionary with the structure required
    result = {
        'DIFF_X': averages.get('DIFF_X', 0),
        'DIFF_Y': averages.get('DIFF_Y', 0),
        'ANGLE': averages.get('ANGLE', 0),
        'CAM_ID': averages.get('CAM_ID', 0),
        'MARKER_VALUE': marker_value,
        'X': x_value,
        'Y': averages.get('Y', 0),
        'Z': averages.get('Z', 0),
        'LASTSEEN': averages.get('LASTSEEN', 0),
        'CUR_POS': round(interpolated_cur_pos),
        'CUR_SPD': averages.get('CUR_SPD', 0),
        'DISTANCE_FROM_HOME': averages.get('DISTANCE_FROM_HOME', 0),
        'DISTANCE_ARMEXTENDED': averages.get('DISTANCE_ARMEXTENDED', 0),
        'ANGLE_540': averages.get('ANGLE_540', 0),
        'ANGLE_360': averages.get('ANGLE_360', 0)
    }
    
    return result


def estimate_all_marker_values(data: pd.DataFrame, x_value: float) -> pd.DataFrame:
    results = []  # List to store each row result
    new_tables = []

    # Get unique MARKER_VALUEs
    unique_marker_values = data[MAP_ARUCO.MARKER_VALUE.name].unique()

    for marker_value in unique_marker_values:
        # Filter data based on the current MARKER_VALUE
        marker_data = data[data[MAP_ARUCO.MARKER_VALUE.name] == marker_value]
        # Prepare for interpolation: extract X and CUR_POS columns for the specified MARKER_VALUE
        x_values = marker_data[ARUCO_RESULT_FIELD.X.name].values
        cur_pos_values = marker_data[MonitoringField.CUR_POS.name].values
        
        # Handle case with only one unique X value
        if len(np.unique(x_values)) == 1:
            interpolated_cur_pos = cur_pos_values[0]
        else:
            # Linear interpolation of CUR_POS based on X
            interpolation_func = interp1d(x_values, cur_pos_values, kind='linear', fill_value='extrapolate')
            interpolated_cur_pos = interpolation_func(x_value)

        # Convert interpolated_cur_pos to a scalar if it's an ndarray
        if isinstance(interpolated_cur_pos, np.ndarray):
            interpolated_cur_pos = interpolated_cur_pos.item()

        # Calculate average values for other columns based on MARKER_VALUE
        averages = marker_data.mean(numeric_only=True).to_dict()

        # Construct the result dictionary
        result = {
            ARUCO_RESULT_FIELD.DIFF_X.name: averages.get(ARUCO_RESULT_FIELD.DIFF_X.name, 0),
            ARUCO_RESULT_FIELD.DIFF_Y.name: averages.get(ARUCO_RESULT_FIELD.DIFF_Y.name, 0),
            ARUCO_RESULT_FIELD.ANGLE.name: averages.get(ARUCO_RESULT_FIELD.ANGLE.name, 0),
            ARUCO_RESULT_FIELD.CAM_ID.name: averages.get(ARUCO_RESULT_FIELD.CAM_ID.name, 0),
            MAP_ARUCO.MARKER_VALUE.name: marker_value,
            ARUCO_RESULT_FIELD.X.name: x_value,
            ARUCO_RESULT_FIELD.Y.name: averages.get('Y', 0),
            ARUCO_RESULT_FIELD.Z.name: averages.get('Z', 0),
            ARUCO_RESULT_FIELD.LASTSEEN.name: averages.get(ARUCO_RESULT_FIELD.LASTSEEN.name, 0),
            MonitoringField.CUR_POS.name: round(interpolated_cur_pos),
            MonitoringField.CUR_SPD.name: averages.get(MonitoringField.CUR_SPD.name, 0),
            BLB_LOCATION.DISTANCE_FROM_HOME.name: averages.get(BLB_LOCATION.DISTANCE_FROM_HOME.name, 0),
            BLB_LOCATION.DISTANCE_ARMEXTENDED.name: averages.get(BLB_LOCATION.DISTANCE_ARMEXTENDED.name, 0),
            BLB_LOCATION.ANGLE_540.name: averages.get(BLB_LOCATION.ANGLE_540.name, 0),
            BLB_LOCATION.ANGLE_360.name: averages.get(BLB_LOCATION.ANGLE_360.name, 0)
        }
        
        # Append result to the list
        results.append(result)
        new_tb_info={}
        new_tb_info[TableInfo.TABLE_ID.name] = marker_value
        new_tb_info[TableInfo.NODE_ID.name] = marker_value
        new_tb_info[TableInfo.SERVING_DISTANCE.name] = 0
        new_tb_info[TableInfo.SERVING_ANGLE.name] = 0
        new_tb_info[TableInfo.MARKER_ANGLE.name] = 0
        new_tables.append(new_tb_info)

    # Convert the list of results to a DataFrame
    new_tables_df = pd.DataFrame(new_tables)
    result_df = pd.DataFrame(results)
    print(new_tables_df)
    return result_df,new_tables_df

def insert_nodes_with_relative_distances(main_df, estimate_df):
    # CUR_POS 기준으로 estimate_df 정렬
    sorted_estimate_df = estimate_df.sort_values(by='CUR_POS').reset_index(drop=True)
    
    # 원래 노드들 사이에 상대적인 거리로 노드 연결 생성
    new_rows = []
    prev_node = main_df['START_NODE'].iloc[0]
    prev_pos = 0  # 절대 시작 위치를 0으로 설정
    
    # 모든 새 노드를 처리하며 상대 거리 계산
    for _, row in sorted_estimate_df.iterrows():
        new_tb_info = {}
        cur_node = int(row['MARKER_VALUE'])        
        cur_pos = pulseH_to_distance(int(row['CUR_POS']))
        
        # 노드 사이의 상대 거리 계산
        relative_distance = cur_pos - prev_pos
        
        # 새 행 추가
        new_rows.append({
            'START_NODE': prev_node,
            'END_NODE': cur_node,
            'DISTANCE': relative_distance,
            'CUR_POS': row['CUR_POS']
        })
        
        # 현재 노드를 이전 노드로 업데이트
        prev_node = cur_node
        prev_pos = cur_pos

    # 마지막으로 원래의 END_NODE 연결
    final_node = main_df['END_NODE'].iloc[0]
    final_distance = main_df['DISTANCE'].iloc[0] - prev_pos
    
    new_rows.append({
        'START_NODE': int(round(prev_node)),
        'END_NODE': int(round(final_node)),
        'DISTANCE': int(round(final_distance))
    })

    # 기존 main_df에 새로운 노드들을 포함한 데이터프레임으로 반환
    extended_df = pd.DataFrame(new_rows)
    return extended_df

# # Example usage of the function with a sample MARKER_VALUE and X value
# unique_values_in_B = dfAruco['MARKER_VALUE'].unique().tolist()
# for marker_value in unique_values_in_B:
#     example_result = estimate_cur_pos(dfAruco, marker_value,370)
#     print(example_result)
example_df,new_tables_df = estimate_all_marker_values(dfAruco, x_value=370)
PrintDFAruco(example_df)
print(new_tables_df)

# 예제 데이터로 함수 사용
main_df = pd.DataFrame({
    'START_NODE': [1],
    'END_NODE': [4],
    'DISTANCE': [DISTANCE]
})
# result_df = insert_nodes_with_relative_distances(main_df, example_df)
# print(result_df)