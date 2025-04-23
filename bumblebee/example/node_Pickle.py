
import os
import pickle

from UtilBLB import *

#print(f"Minimum pulse to move from current position to {target_angle} degrees: {minimum_pulse}")

class PickleHandler:
    def __init__(self, file_path):
        self.file_path = file_path

    def save_data(self, data):
        with open(self.file_path, 'wb') as file:
            pickle.dump(data, file)
        print(f'Data saved to {self.file_path}')

    def load_data(self):
        if os.path.exists(self.file_path):
            with open(self.file_path, 'rb') as file:
                return pickle.load(file)
        else:
            return None

    def check_and_handle_pickle(self, default_data=None):
        if not os.path.exists(self.file_path):
            print(f'{self.file_path} not found, creating new with default data...')
            self.save_data(default_data if default_data is not None else {})
            return default_data
        else:
            print(f'{self.file_path} found, loading data...')
            return self.load_data()

dirPath2 = getConfigPath(UbuntuEnv.ITX.name)
filePath_modbusconfig = f"{dirPath2}/ModbusConfig.txt"

filePath_modbusconfig_pkl = f"{dirPath2}/ModbusConfig.pkl"
# pickle 파일 체크 후 적절히 처리
dicConfigTmp = getDic_FromFile(filePath_modbusconfig, sDivEmart)

pickle_handler = PickleHandler(filePath_modbusconfig_pkl)
loaded_data = pickle_handler.check_and_handle_pickle(dicConfigTmp)

print(f'loaded_data: {loaded_data}')
