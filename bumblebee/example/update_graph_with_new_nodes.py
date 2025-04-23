def update_graph_with_new_nodes(node1, node2, total_distance, new_node_distances, start_number):
    """
    기존 두 노드 사이에 새로운 노드들을 추가하고 그래프를 갱신하는 함수
    
    Parameters:
    node1: 시작 노드 번호
    node2: 끝 노드 번호
    total_distance: 전체 구간 거리
    new_node_distances: 시작점으로부터의 절대 거리 리스트
    start_number: 신규 노드 번호 시작값
    
    Returns:
    list: [node1, node2, distance] 형태의 엣지 정보 리스트
    """
    # 거리순으로 정렬
    new_node_distances.sort()
    
    # 새로운 노드 번호 할당
    new_nodes = [start_number + i for i in range(len(new_node_distances))]
    
    # 모든 노드의 위치를 순서대로 정렬
    all_nodes = [node1] + new_nodes + [node2]
    all_distances = [0] + new_node_distances + [total_distance]
    
    # 엣지 정보 생성
    edges = []
    for i in range(len(all_nodes)-1):
        current_node = all_nodes[i]
        next_node = all_nodes[i+1]
        distance = all_distances[i+1] - all_distances[i]
        edges.append([current_node, next_node, distance])
    
    return edges

# 테스트
node1, node2 = 1, 4
total_distance = 10
new_node_distances = [3, 5, 7, 8, 9]
start_number = 5

result = update_graph_with_new_nodes(node1, node2, total_distance, new_node_distances, start_number)

# 결과 출력
for edge in result:
    print(f"{edge[0]} {edge[1]} {edge[2]}")