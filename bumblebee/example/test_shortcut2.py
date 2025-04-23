from Util import *
from UtilGPIO import *
#소요 시간을 초기화할 때 사용할 무한대(inf)를 이용하기 위해 수학 모듈 사용
import math
import heapq


n = 5
start = 1
endPoint = 5
# 노드별로 연결된 노드 정보를 저장할 리스트 선언
graph = {}


# graph = {
#     "A": [("B", 1), ("C", 4), ("D", 5)],
#     "B": [("A", 1), ("D", 2)],
#     "C": [("A", 4), ("D", 4), ("E", 3)],
#     "D": [("A", 5), ("B", 2), ("C", 4), ("F", 3)],
#     "E": [("C", 3), ("F", 2)],
#     "F": [("D", 3), ("E", 2)]
# }

dirPath = os.path.dirname(__file__)
strFilePath = f'{dirPath}/SHORTCUT.txt'

def LoadGraph(strFilePath:str) -> dict:
    """LoadGraph 아래와 같은 텍스트를 읽어들여 경로정보를 생성한다.
    
    1 200 5
    200 3 10
    2 100 10
    3 5 12
    100 5 10

    Args:
        strFilePath (str): _description_

    Returns:
        dict: _description_
    """
    graphTmp : Dict[int,int] = {}
    file_list = getLines_FromFile(strFilePath)
    for i in file_list:
        splitI = i.split(' ')
        #print(mapRange(int, splitI))
        n_a, n_b, cost = map(int, splitI)
        strA = (n_a)
        strB = (n_b)
        #strA = str(n_a)
        #strB = str(n_b)
        
        if graphTmp.get(strA, None) == None:
            graphTmp[strA] = []            
        if graphTmp.get(strB, None) == None:
            graphTmp[strB] = []
        
        graphTmp[strA].append((strB, cost))
        graphTmp[strB].append((strA, cost))
        
        # graph[n_a].append((str(n_b), cost))
        # graph[n_b].append((str(n_a), cost))
    print(graphTmp, len(graphTmp))
    return graphTmp 

def dijkstra(graph : dict, node):
    """최단 경로 전체정보를 구하는데 필요한 lead_time 과 node_from 값을 구한다.
    예제 : lead_time, node_from = dijkstra(graph, "A") 
    Args:
        graph = {
            "A": [("B", 1), ("C", 4), ("D", 5)],
            "B": [("A", 1), ("D", 2)],
            "C": [("A", 4), ("D", 4), ("E", 3)],
            "D": [("A", 5), ("B", 2), ("C", 4), ("F", 3)],
            "E": [("C", 3), ("F", 2)],
            "F": [("D", 3), ("E", 2)]
        }
        node (_type_): 시작노드 지정

    Returns:
        lead_time = {'1': None, '2': '1', '3': '2', '4': '2', '5': '4'} #node 를 중심으로 한 경로가중치값 
        node_from = {'1': 0, '2': 5, '3': 15, '4': 15, '5': 25} #각 노드까지의 최단거리.
    """
    #소요시간을 기록할 사전과 현재 노드의 이전 노드를 기록할 사전을 초기화
    lead_time = {node: math.inf for node in graph}
    node_from = {node: None for node in graph}

    #출발 노드(node)의 소요 시간을 0으로 만든다.
    lead_time[node] = 0
    visited = set()

    #(소요 시간, 노드)의 자료를 넣는 최소 힙을 만든다.
    #그리고 출발 노드를 힙에 넣는다.
    heap = []
    heapq.heappush(heap, (0, node))

    #힙이 빌 때까지 반복한다.
    while heap:
        #힙에서 자료를 꺼내서 시간과 노드를 변수에 저장한다.
        prev_time, u = heapq.heappop(heap)

        #이미 방문한 노드이면 다음 노드를 꺼내고, 그렇지 않으면 방문처리한다.
        if u in visited:
            continue
        visited.add(u)

        #현재 노드에 인접한 노드와 가중치(소요 시간)를 가져와서 반복한다.
        for v, weight in graph[u]:
            #인접한 노드로 가는 소요 시간을 계산하여 기본 값보다 작으면,
            #시간 정보를 갱신하고, 이전 노드를 기록한다. 그리고 힙에 넣는다.
            if (new_time := prev_time + weight) < lead_time[v]:
                lead_time[v] = new_time
                node_from[v] = u
                heapq.heappush(heap, (lead_time[v], v))
    #소요 시간과 이전 노드를 기록한 사전을 반환한다.
    return lead_time, node_from

def shortest_path(node_from, lead_time, end):
    returnPath = []
    path = ""
    node = end
    while node_from[node]:
        returnPath.insert(0,node)        
        path = " → " + str(node) + path
        node = node_from[node]
    lead_time_end = str(lead_time[end])
        
    print(f"{str(node) + path} (cost = {lead_time_end})")
    #returnPath.append(end)
    return returnPath,lead_time_end

graph = LoadGraph(strFilePath)
lead_time, node_from = dijkstra(graph, start)

print("A에서 다익스트라 탐색 후의 node_from과 lead_time의 상태")
print(node_from)
print(lead_time)
print()
print(f"{start}에서 {endPoint}로 가는 최단 경로:", end = " ")
lsResult,cost = shortest_path(node_from, lead_time, endPoint)
print(lsResult,cost)