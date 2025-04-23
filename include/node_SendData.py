import time
import sys
import os
import time
import tarfile
import shutil
import socket
import datetime

def get_size_format(b, factor=1024, suffix="B"):
    """
    Scale bytes to its proper byte format
    e.g:
        1253656 => '1.20MB'
        1253656678 => '1.17GB'
    """
    for unit in ["", "K", "M", "G", "T", "P", "E", "Z"]:
        if b < factor:
            return f"{b:.2f}{unit}{suffix}"
        b /= factor
    return f"{b:.2f}Y{suffix}"

def get_directory_size(directory):
    """Returns the `directory` size in bytes."""
    total = 0
    try:
        # print("[+] Getting the size of", directory)
        for entry in os.scandir(directory):
            if entry.is_file():
                # if it's a file, use stat() function
                total += entry.stat().st_size
            elif entry.is_dir():
                # if it's a directory, recursively call this function
                try:
                    total += get_directory_size(entry.path)
                except FileNotFoundError:
                    pass
    except NotADirectoryError:
        # if `directory` isn't a directory, get the file size then
        return os.path.getsize(directory)
    except PermissionError:
        # if for whatever reason we can't open the folder, return 0
        return 0
    return total

dir_path = '/root/SpiderGo/'
mount_path = '/mnt'
clueFileFullPath = f'{mount_path}/Thumbs.db'
client_name = socket.gethostname()
share_name = 'orgimg'
ip = '172.30.1.191'

while os.path.exists(clueFileFullPath) == False:
    os.system(f'mount -t cifs -o username=L,password=543678 //172.30.1.191/orgimg/ {mount_path}')
    time.sleep(5)

# list file and directories
fullPath = []
res = os.listdir(dir_path)
for filename in res:
    fullPath.append(os.path.join(dir_path, filename))
cnt = 0

for fullPathFile in fullPath:
    cnt += 1
    if os.path.isdir(fullPathFile):
        time_move = datetime.datetime.now()
        dirSize = get_directory_size(fullPathFile)
        dirMB = get_size_format(dirSize)
        shutil.move(fullPathFile, mount_path)
        td = datetime.datetime.now() - time_move
        print(f'{cnt}:{fullPathFile} has been uploaded with {td.total_seconds()} secs. size = {dirMB}')
    else:
        print(f'{cnt} - {fullPathFile} is not a directory.')
    #os.remove(fullPathFile)

os.system(f'umount {mount_path}')
print(f'Job finished : {cnt} Folders')