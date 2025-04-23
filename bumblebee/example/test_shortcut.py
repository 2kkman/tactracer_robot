from collections import defaultdict, deque

def add_edge(graph, u, v, w):
    graph[u].append((v, w))
    graph[v].append((u, w))

def find_eulerian_path(graph):
    # Check if an Eulerian path or circuit exists by finding vertices with odd degrees
    odd_nodes = [node for node in graph if len(graph[node]) % 2 == 1]
    start = odd_nodes[0] if len(odd_nodes) == 2 else next(iter(graph))
    
    path = []
    stack = [start]
    
    while stack:
        u = stack[-1]
        if graph[u]:
            v, _ = graph[u].pop()
            stack.append(v)
            graph[v] = [(n, w) for n, w in graph[v] if n != u]
        else:
            path.append(stack.pop())
    
    return path[::-1]

# 그래프 초기화
graph = defaultdict(list)

# 입력된 간선 추가
edges = [
    # (1, 2, 2500),
    # (2, 3, 5000),
    (3, 4, 2000),
    (4, 5, 833),
    (4, 14, 600)
]

for u, v, w in edges:
    add_edge(graph, u, v, w)

# 오일러 경로 찾기
euler_path = find_eulerian_path(graph)
print(euler_path)
