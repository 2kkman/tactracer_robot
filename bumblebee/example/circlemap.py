def get_shortest_path_position(node_positions, start_node, target_node):
    """
    원형 레일에서 시작 노드부터 목표 노드까지의 최단거리 절대좌표를 계산하는 함수
    
    Args:
        node_positions (dict): 노드 번호를 키로, 절대위치를 값으로 하는 딕셔너리
        start_node (int): 시작 노드 번호
        target_node (int): 목표 노드 번호
    
    Returns:
        int: 목표 노드의 절대 좌표 (최단거리 경로)
    """
    
    # 한 바퀴 총 펄스 수
    FULL_CIRCLE_PULSE = 2560000
    
    # 기본 노드 데이터 (절대 위치값)
    default_node_positions = {
        24: -10826581, 23: -10187950, 22: -9689609, 20: -8786334, 19: -8318576,
        42: -8191329, 43: -7951329, 15: -6620000, 14: -5168416, 6: -4443210,
        13: -3985639, 12: -3175579, 11: -2816437, 16: -2486000, 9: -1667940,
        5: -1266959, 8: -467408, 21: 312454, 4: 875000, 10: 1030000,
        7: 1125000, 17: 1697600, 47: 1873214, 46: 2083369, 18: 2479622,
        25: 3157862, 26: 3782684, 27: 4225517, 28: 5050935, 29: 5553023,
        45: 6267555, 44: 6295535, 32: 6764534, 30: 6942267, 31: 7120000,
        33: 8335784, 34: 8666701, 41: 9457071, 40: 9751578, 35: 10022183,
        39: 10815500, 36: 11113526, 37: 12503679, 38: 14165494
    }
    
    # 파라미터로 받은 노드 위치 정보가 없으면 기본값 사용
    if node_positions is None:
        node_positions = default_node_positions
    
    # 입력 검증
    if start_node not in node_positions:
        raise ValueError(f"시작 노드 {start_node}이 존재하지 않습니다.")
    if target_node not in node_positions:
        raise ValueError(f"목표 노드 {target_node}이 존재하지 않습니다.")
    
    # 시작 노드와 목표 노드의 절대 위치
    start_pos = node_positions[start_node]
    target_pos = node_positions[target_node]
    
    # 목표 노드까지의 직접 거리
    direct_distance = target_pos - start_pos
    
    # 원형 레일에서의 최단 거리 계산
    # 시계방향 거리
    if direct_distance >= 0:
        clockwise_distance = direct_distance
    else:
        clockwise_distance = direct_distance + FULL_CIRCLE_PULSE
    
    # 반시계방향 거리  
    if direct_distance <= 0:
        counterclockwise_distance = direct_distance
    else:
        counterclockwise_distance = direct_distance - FULL_CIRCLE_PULSE
    
    # 절댓값이 더 작은 쪽이 최단거리
    if abs(clockwise_distance) <= abs(counterclockwise_distance):
        # 시계방향이 더 가까움 - 시작점에서 시계방향으로 이동한 절대 위치
        result_position = start_pos + clockwise_distance
    else:
        # 반시계방향이 더 가까움 - 시작점에서 반시계방향으로 이동한 절대 위치
        result_position = start_pos + counterclockwise_distance
    
    # 원형 레일 범위 내로 정규화 (0 ~ FULL_CIRCLE_PULSE-1)
    result_position = result_position % FULL_CIRCLE_PULSE
    
    return result_position


# 사용 예시 및 테스트
if __name__ == "__main__":
    # 예시: 10번 노드에서 38번 노드까지 최단경로
    try:
        target_position = get_shortest_path_position(None, 20, 38)
        print(f"10번 노드에서 38번 노드까지 최단경로 목표 절대위치: {target_position}")
        
        # 다른 예시들
        test_cases = [
            (10, 24),  # 10번에서 24번까지
            (24, 38),  # 24번에서 38번까지
            (38, 24),  # 38번에서 24번까지
            (5, 45),   # 5번에서 45번까지
            (24, 10),  # 24번에서 10번까지
        ]
        
        print("\n=== 테스트 결과 ===")
        for start, target in test_cases:
            result_pos = get_shortest_path_position(None, start, target)
            print(f"{start}번 → {target}번: 목표 절대위치 = {result_pos}")
            
        # 거리 계산도 보여주기
        print("\n=== 거리 정보 포함 ===")
        node_pos = {
            24: -10826581, 23: -10187950, 22: -9689609, 20: -8786334, 19: -8318576,
            42: -8191329, 43: -7951329, 15: -6620000, 14: -5168416, 6: -4443210,
            13: -3985639, 12: -3175579, 11: -2816437, 16: -2486000, 9: -1667940,
            5: -1266959, 8: -467408, 21: 312454, 4: 875000, 10: 1030000,
            7: 1125000, 17: 1697600, 47: 1873214, 46: 2083369, 18: 2479622,
            25: 3157862, 26: 3782684, 27: 4225517, 28: 5050935, 29: 5553023,
            45: 6267555, 44: 6295535, 32: 6764534, 30: 6942267, 31: 7120000,
            33: 8335784, 34: 8666701, 41: 9457071, 40: 9751578, 35: 10022183,
            39: 10815500, 36: 11113526, 37: 12503679, 38: 14165494
        }
        
        for start, target in [(10, 38), (24, 10)]:
            result_pos = get_shortest_path_position(None, start, target)
            start_pos = node_pos[start]
            target_pos = node_pos[target]
            direct_dist = target_pos - start_pos
            actual_dist = result_pos - start_pos
            
            print(f"\n{start}번({start_pos}) → {target}번({target_pos})")
            print(f"  직선거리: {direct_dist}")
            print(f"  최단거리: {actual_dist}")
            print(f"  목표 절대위치: {result_pos}")
            
    except ValueError as e:
        print(f"오류: {e}")