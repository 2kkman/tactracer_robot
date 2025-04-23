#!/usr/bin/env python3
from UtilBLB import *
import time
import os
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import ftplib
import rospy

class Watcher:
    def __init__(self, directory_to_watch, ftp_server, ftp_port, ftp_username, ftp_password, ftp_target_dir):
        self.DIRECTORY_TO_WATCH = directory_to_watch
        self.FTP_SERVER = ftp_server
        self.FTP_PORT = ftp_port
        self.FTP_USERNAME = ftp_username
        self.FTP_PASSWORD = ftp_password
        self.FTP_TARGET_DIR = ftp_target_dir
        self.observer = Observer()

    def run(self):
        event_handler = Handler(self.FTP_SERVER, self.FTP_PORT, self.FTP_USERNAME, self.FTP_PASSWORD, self.FTP_TARGET_DIR)
        self.observer.schedule(event_handler, self.DIRECTORY_TO_WATCH, recursive=True)
        self.observer.start()
        self.process_existing_files(event_handler)
        try:
            while not rospy.is_shutdown():
                time.sleep(5)
        except KeyboardInterrupt:
            self.observer.stop()
        self.observer.join()

    def process_existing_files(self, event_handler):
        for root, _, files in os.walk(self.DIRECTORY_TO_WATCH):
            for file in files:
                if file.endswith(".mp4"):
                    file_path = os.path.join(root, file)
                    if event_handler.is_file_too_small(file_path):
                        os.remove(file_path)
                        rospy.loginfo(f"Deleted {file_path} as it is smaller than 1MB")
                    else:
                        event_handler.upload_file(file_path)

class Handler(FileSystemEventHandler):
    def __init__(self, ftp_server, ftp_port, ftp_username, ftp_password, ftp_target_dir):
        self.FTP_SERVER = ftp_server
        self.FTP_PORT = ftp_port
        self.FTP_USERNAME = ftp_username
        self.FTP_PASSWORD = ftp_password
        self.FTP_TARGET_DIR = ftp_target_dir
        self.failure_count = 0

    def on_created(self, event):
        if not event.is_directory and event.src_path.endswith(".mp4"):
            while not self.is_file_ready(event.src_path):
                time.sleep(1)
            if self.is_file_too_small(event.src_path):
                os.remove(event.src_path)
                rospy.loginfo(f"Deleted {event.src_path} as it is smaller than 1MB")
            else:
                self.upload_file(event.src_path)

    def is_file_ready(self, file_path):
        try:
            os.rename(file_path, file_path)  # 파일이 다른 프로세스에 의해 사용 중인지 확인
            return True
        except OSError:
            return False

    def is_file_too_small(self, file_path):
        return os.path.getsize(file_path) < 1 * 1024 * 1024  # 1MB보다 작은지 확인

    def upload_file(self, file_path):
        try:
            with ftplib.FTP() as ftp:
                ftp.connect(self.FTP_SERVER, self.FTP_PORT)
                ftp.login(self.FTP_USERNAME, self.FTP_PASSWORD)
                ftp.cwd(self.FTP_TARGET_DIR)
                # 파일 이름을 UTF-8로 인코딩하여 전송
                file_name_utf8 = os.path.basename(file_path).encode('utf-8')
                with open(file_path, 'rb') as f:
                    ftp.storbinary(f'STOR {file_name_utf8.decode("utf-8")}', f)
            os.remove(file_path)
            rospy.loginfo(f"Successfully uploaded and deleted: {file_path}")
            self.failure_count = 0  # 성공 시 실패 카운트 초기화
        except Exception as e:
            rospy.logerr(f"Failed to upload {file_path}: {e}")
            self.failure_count += 1
            if self.failure_count >= 5:
                rospy.logerr("5번 연속 업로드 실패. 노드를 재시작합니다.")
                rospy.signal_shutdown("5번 연속 업로드 실패")
            else:
                # 실패 시 5초 후 재시도
                time.sleep(5)
                self.upload_file(file_path)

if __name__ == '__main__':
    rospy.init_node('BLB_RECORDER', anonymous=True)
    
    ftp_target_dir = rospy.get_param('~dirPath', 'BLB_1')
    ftp_server = rospy.get_param('~ftpaddr','iot.tactracer.com')
    ftp_port = rospy.get_param('~port',60131)
    ftp_username = rospy.get_param('~ftp_username', 'rfid')  # 기본 사용자 이름을 지정하세요
    ftp_password = rospy.get_param('~ftp_password', '0000')  # 기본 비밀번호를 지정하세요
    directory_to_watch  = PATH_RECORDING

    w = Watcher(directory_to_watch, ftp_server, ftp_port, ftp_username, ftp_password, ftp_target_dir)
    w.run()
