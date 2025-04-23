import pathlib
import os
from smb.SMBConnection import SMBConnection
import time
import tarfile
import shutil
import socket
# dicRFID = {'28575' : '303443D11C27104000000710',  '73566' : '303443D11C27104000000710', '36583' : '303443D11C27104000000710'}
# if len(dicRFID) > 0:
#     mapping = False
#     dicRFIDTEMP = {}
#     dicRFIDFinal = {}
#     for k,v in dicRFID.items():
#         if v in dicRFIDTEMP.keys():
#             dicRFIDTEMP[v].append(k)
#         else:
#             dicRFIDTEMP[v] = []
#             dicRFIDTEMP[v].append(k)
    
#     for k,v in dicRFIDTEMP.items():
#         sum = 0
#         for strPos in v:
#             sum = sum + int(strPos)
#         average = sum / len(v)
#         dicRFIDFinal[k] = int(average)
#     #saveDic_ToFile(dicRFIDFinal, filePath_map_default, sDivEmart)
#     print(f'Save the map : {dicRFIDFinal}')
# folder path
dir_path = '/root/SpiderGo/'
client_name = socket.gethostname()
share_name = 'orgimg'
ip = '172.30.1.191'
server = SMBConnection('L','543678',client_name,share_name,is_direct_tcp=True)
server.connect(ip,445)
# list file and directories
fullPath = []
validFileExt = ['.jpg', '.bag']
res = os.listdir(dir_path)
for filename in res:
    fullPath.append(os.path.join(dir_path, filename))
cnt = 0
for fullPathFile in fullPath:
    cnt += 1
    extFile = pathlib.Path(fullPathFile).suffix
    if extFile in validFileExt and  os.path.getsize(fullPathFile) > 100000:
        #print(fullPathFile)
        data = open(fullPathFile,'rb')
        #file = '/' + fullPathFile
        file =  os.path.basename(fullPathFile)
        server.storeFile(share_name,file,data)
        print(f'{cnt}:{file} has been uploaded')
    else:
        print(f'{cnt} - {fullPathFile} has been removed')
    os.remove(fullPathFile)
        