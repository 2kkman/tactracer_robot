def update_node_config(file_path, node_id, old_config, new_config):
    # 파일 읽기
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    # 파일 내용 업데이트
    updated_lines = []
    for line in lines:
        parts = line.strip().split()
        # 첫 번째 항목이 node_id와 일치하는지 확인
        if parts and int(parts[0]) == node_id:
            # 기존 설정값이 일치하는지 확인
            if parts[1:] == old_config:
                # 신규 설정값으로 변경
                updated_line = f"{node_id} " + " ".join(map(str, new_config)) + "\n"
            else:
                # 기존 설정값이 일치하지 않으면 변경하지 않음
                updated_line = line
        else:
            updated_line = line
        updated_lines.append(updated_line)
    
    # 파일에 업데이트된 내용 쓰기
    with open(file_path, 'w') as file:
        file.writelines(updated_lines)

# 사용 예시
file_path = 'node_config.txt'
node_id = 4
old_config = [3, 5, -1, 1]
new_config = [6, 7, 8, 9]
update_node_config(file_path, node_id, old_config, new_config)
