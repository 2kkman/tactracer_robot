import pandas as pd

# Example graphTmp data (Python dictionary)
graphTmp = {
    1: [(2, 2500)],
    2: [(1, 2500), (3, 5000)],
    3: [(2, 5000), (4, 2000)],
    4: [(3, 2000), (5, 833), (14, 600)],
    5: [(4, 833), (6, 833)],
    6: [(5, 833), (7, 833)],
    7: [(6, 833), (8, 833)],
    8: [(7, 833), (9, 833)],
    9: [(8, 833), (10, 833)],
    10: [(9, 833), (11, 833)],
    11: [(10, 833), (12, 833)],
    12: [(11, 833), (13, 833)],
    13: [(12, 833)],
    14: [(4, 600), (15, 833)],
    15: [(14, 833), (16, 833)],
    16: [(15, 833), (17, 833)],
    17: [(16, 833), (18, 833)],
    18: [(17, 833), (19, 2000)],
    19: [(18, 2000), (20, 833)],
    20: [(19, 833), (21, 833)],
    21: [(20, 833), (22, 833)],
    22: [(21, 833)]
}

# Function to convert graphTmp to DataFrame
def graph_to_dataframe(graph):
    edges = set()  # To avoid duplicate edges
    for node, neighbors in graph.items():
        for neighbor, cost in neighbors:
            # Ensure smaller node number comes first to avoid duplicates
            edge = tuple(sorted((node, neighbor)))
            edges.add((edge[0], edge[1], cost))
    
    # Create DataFrame
    df = pd.DataFrame(edges, columns=["node1", "node2", "distance"])
    return df

# Convert to DataFrame
df_graph = graph_to_dataframe(graphTmp)
import heapq
from typing import Dict, List, Tuple

def calculate_distance(graph: Dict[int, List[Tuple[int, int]]], start: int, end: int) -> int:
    """
    두 노드 간 최단 거리를 계산하는 함수 (다익스트라 알고리즘 사용)
    :param graph: 그래프 데이터 (예: {1: [(2, 2500), (3, 5000)], ...})
    :param start: 출발 노드 번호
    :param end: 도착 노드 번호
    :return: 최단 거리 값 또는 None (경로가 없는 경우)
    """
    # 우선순위 큐와 거리 초기화
    queue = [(0, start)]  # (누적 거리, 현재 노드)
    distances = {node: float('inf') for node in graph}
    distances[start] = 0

    # 다익스트라 알고리즘
    while queue:
        current_distance, current_node = heapq.heappop(queue)

        # 이미 처리된 노드 무시
        if current_distance > distances[current_node]:
            continue

        # 인접 노드 확인
        for neighbor, weight in graph.get(current_node, []):
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(queue, (distance, neighbor))

    # 결과 반환
    return distances[end] if distances[end] != float('inf') else None


# 테스트
if __name__ == "__main__":
    # graphTmp는 기존에 정의된 그래프 데이터
    start_node = 1
    end_node = 4
    distance = calculate_distance(graphTmp, start_node, end_node)
    if distance is not None:
        print(f"노드 {start_node}에서 노드 {end_node}까지의 최단 거리는 {distance}입니다.")
    else:
        print(f"노드 {start_node}에서 노드 {end_node}까지 경로가 없습니다.")
