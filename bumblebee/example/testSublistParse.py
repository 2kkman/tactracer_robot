def create_mbid_pos_dict(recvDataMap):
    mbid_pos_dict = {}
    for sublist in recvDataMap:
        for item in sublist:
            mbid = item.get('MBID')
            pos = item.get('POS')
            if mbid and pos:  # MBID와 POS 값이 존재할 경우에만 추가
                mbid_pos_dict[mbid] = pos
    return mbid_pos_dict

recvDataMap = [
    [{'MBID': '13', 'CMD': 'WMOVE', 'MODE': '1', 'POS': '1000000', 'SPD': '300', 'ACC': '500', 'DECC': '500'}, 
     {'MBID': '10', 'CMD': 'WMOVE', 'MODE': '1', 'POS': '2000000', 'SPD': '500', 'ACC': '500', 'DECC': '500'}, 
     {'MBID': '29', 'CMD': 'WMOVE', 'MODE': '1', 'POS': '783263', 'SPD': '183', 'ACC': '500', 'DECC': '500'}], 
    [{'MBID': '30', 'CMD': 'WMOVE', 'MODE': '1', 'POS': '145660', 'SPD': '500', 'ACC': '500', 'DECC': '500'}]
]
rc2 = [[{'MBID': '12', 'CMD': 'WMOVE', 'MODE': '1', 'POS': '92878', 'SPD': '500', 'ACC': '500', 'DECC': '500'}], [{'MBID': '28', 'CMD': 'WMOVE', 'MODE': '1', 'POS': '43576', 'SPD': '500', 'ACC': '500', 'DECC': '500'}], [[{'MBID': '13', 'CMD': 'WMOVE', 'MODE': '1', 'POS': '1000000', 'SPD': '300', 'ACC': '500', 'DECC': '500'}, {'MBID': '10', 'CMD': 'WMOVE', 'MODE': '1', 'POS': '2000000', 'SPD': '500', 'ACC': '500', 'DECC': '500'}, {'MBID': '29', 'CMD': 'WMOVE', 'MODE': '1', 'POS': '783263', 'SPD': '183', 'ACC': '500', 'DECC': '500'}], [{'MBID': '30', 'CMD': 'WMOVE', 'MODE': '1', 'POS': '145660', 'SPD': '500', 'ACC': '500', 'DECC': '500'}]], [{'MBID': '31', 'CMD': 'WMOVE', 'MODE': '1', 'POS': '8484', 'SPD': '500', 'ACC': '500', 'DECC': '500'}]]
result = create_mbid_pos_dict(rc2)
print(result)
