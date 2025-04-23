class Graph:
    def __init__(self):
        self.graph = {}  # 그래프 데이터를 저장할 딕셔너리

    def add_edge(self, node1, node2, distance):
        """노드 간의 간선을 추가"""
        if node1 not in self.graph:
            self.graph[node1] = []
        if node2 not in self.graph:
            self.graph[node2] = []
        
        # 간선 추가 (중복 방지)
        if [node2, distance] not in self.graph[node1]:
            self.graph[node1].append([node2, distance])
        if [node1, distance] not in self.graph[node2]:
            self.graph[node2].append([node1, distance])

    def to_text_file(self, file_path):
        """그래프를 텍스트 형식으로 저장"""
        visited = set()
        lines = []

        for node, edges in self.graph.items():
            for neighbor, distance in edges:
                # 중복된 간선 방지
                if (node, neighbor) not in visited and (neighbor, node) not in visited:
                    lines.append(f"{node} {neighbor} {distance}")
                    visited.add((node, neighbor))

        # 파일에 저장
        with open(file_path, "w") as file:
            file.write("\n".join(lines))

    def remove_edge(self, node1, node2):
        """노드 간의 간선을 제거"""
        if node1 in self.graph:
            self.graph[node1] = [edge for edge in self.graph[node1] if edge[0] != node2]
        if node2 in self.graph:
            self.graph[node2] = [edge for edge in self.graph[node2] if edge[0] != node1]

    def add_node_between(self, node1, node2, listNode):
        """
        두 노드 사이에 여러 새로운 노드를 추가.
        listNode: [(노드 ID, 거리)]의 리스트
        """
        # 기존 간선 제거
        self.remove_edge(node1, node2)

        # 기존 노드 정보를 복사 (node2와 연결된 다른 간선 유지)
        original_edges = [edge for edge in self.graph[node2] if edge[0] != node1]

        # 중간 노드 추가
        prev_node = node1
        for new_node, distance in listNode:
            self.add_edge(prev_node, new_node, distance)
            prev_node = new_node

        # 마지막 노드에서 node2와 연결
        self.add_edge(prev_node, node2, listNode[-1][1])

        # node2의 기존 간선 복원
        self.graph[node2] = [[prev_node, listNode[-1][1]]] + original_edges

    def remove_node(self, node):
        """노드를 삭제하고 연결된 모든 간선 제거"""
        if node in self.graph:
            for neighbor, _ in self.graph[node]:
                self.graph[neighbor] = [edge for edge in self.graph[neighbor] if edge[0] != node]
            del self.graph[node]

    def __str__(self):
        """그래프를 문자열로 출력"""
        return str(self.graph)

g = Graph()
g.add_edge(1, 2, 2500)
g.add_edge(2, 3, 5000)
g.add_edge(3, 4, 2000)
print("Initial Graph:")
print(g)

# 1번과 4번 사이에 [23번 노드, 거리 1000], [24번 노드, 거리 1500] 추가
listNode = [(23, 1000), (24, 1500)]
g.add_node_between(1, 4, listNode)

print("\nGraph after adding nodes 23 and 24:")
print(g)

# 그래프를 텍스트 파일로 저장
g.to_text_file("graph.txt")

print("Graph saved to 'graph.txt'")