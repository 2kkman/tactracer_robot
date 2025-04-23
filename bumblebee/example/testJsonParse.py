import json

# 제공된 조각들을 하나의 리스트로
json_fragments = [
    '{"\\uc544\\uc774\\uc2a4 \\uc544\\uba54\\ub9ac\\uce74\\ub178": {"id": 1',
    ' "name": "\\uc544\\uc774\\uc2a4 \\uc544\\uba54\\ub9ac\\uce74\\ub178"',
    ' "englishName": "Iced Americano"',
    ' "price": "4500"',
    ' "count": 2',
    ' "price_total": "9000"}',
    ' "\\ud56b \\uc544\\uba54\\ub9ac\\uce74\\ub178": {"id": 2',
    ' "name": "\\ud56b \\uc544\\uba54\\ub9ac\\uce74\\ub178"',
    ' "englishName": "Hot Americano"',
    ' "price": "4000"',
    ' "count": 2',
    ' "price_total": "8000"}',
    ' "total_price": "17000"',
    ' "total_count": "4"',
    ' "timestamp": "140343"}',
]

# 모든 조각을 연결하여 하나의 문자열로 만듭니다.
json_string = "".join(json_fragments)

# 연결된 문자열을 출력하여 확인
print(json_string)

# JSON 문자열을 파이썬 객체로 변환
parsed_json = json.loads(json_string)

# 변환된 객체를 출력
print(parsed_json)
