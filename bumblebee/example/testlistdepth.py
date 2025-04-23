def find_list_depth(lst):
    # 만약 입력값이 리스트가 아니면 중첩이 없다고 간주하고 깊이를 0으로 반환
    if not isinstance(lst, list):
        return 0
    # 리스트 안의 모든 요소에 대해 깊이를 계산하고, 그 중 가장 깊은 값을 반환
    else:
        return 1 + max(find_list_depth(item) for item in lst) if lst else 1  # 빈 리스트는 깊이가 1

# 예시 테스트
list1 = []
list2 = [[]]
list3 = [[[[]]]]

print(find_list_depth(list1))  # 출력: 1
print(find_list_depth(list2))  # 출력: 2
print(find_list_depth(list3))  # 출력: 4
