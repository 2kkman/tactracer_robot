from flask import Blueprint, request, jsonify
from flask.views import MethodView
import inspect
from UtilBLB import *

pan_blueprint = Blueprint("pan", __name__)
dirPath = getConfigPath(UbuntuEnv.ITX.name) 
strFileCross = f"{dirPath}/CROSS.txt"
strFileShortCut = f"{dirPath}/SHORTCUT.txt"
strFileTable = f"{dirPath}/TableNode_ex.txt"
import pandas as pd
class DataFrameManager:
    def __init__(self, file_path):
        self.df = pd.read_csv(file_path, sep='\t')  # Reading the file as a DataFrame
        
    # 1. Method to filter rows based on a specific key value
    def filter_by_key(self, key_column, key_value):
        filtered_df = self.df[self.df[key_column] == key_value]
        return filtered_df

    # 2. Method to convert DataFrame (or filtered DataFrame) to a list of dictionaries
    def to_dict_list(self, df=None):
        if df is None:
            df = self.df
        return df.to_dict(orient='records')

    # 3. Method to remove rows based on a specific key value
    def remove_by_key(self, key_column, key_value):
        self.df = self.df[self.df[key_column] != key_value]

    # 4. Method to update specific key-value pairs in rows that match a certain key
    def update_by_key(self, key_column, key_value, update_column, new_value):
        self.df.loc[self.df[key_column] == key_value, update_column] = new_value

    # 5. Method to transform DataFrame as per the given structure
    def transform_to_custom_dict_list(self):
        dict_list = []
        for _, row in self.df.iterrows():
            tableno = f"T{row[TableInfo.TABLE_ID.name]}"
            end = row[TableInfo.NODE_ID.name]
            iSERVING_ANGLE = row[TableInfo.SERVING_ANGLE.name]
            distance = row[TableInfo.SERVING_DISTANCE.name]
            time = round(10 + (iSERVING_ANGLE/ 100) + (iSERVING_ANGLE / 10))
            
            # Determine direction based on MARKER_ANGLE
            if 46 <= iSERVING_ANGLE <= 135:
                direction = 'N'
            elif 136 <= iSERVING_ANGLE <= 225:
                direction = 'W'
            elif 226 <= iSERVING_ANGLE <= 315:
                direction = 'S'
            else:
                direction = 'E'
                
            dict_list.append({
                "tableno": tableno,
                "end": end,
                "distance": distance,
                "time": time,
                "direction": direction
            })
        
        return dict_list

# Simulating with the uploaded file
file_path = strFileTable
df_manager = DataFrameManager(file_path)

# Using the new transform_to_custom_dict_list method
custom_dict_list = df_manager.transform_to_custom_dict_list()
#custom_dict_list[:5]  # Displaying the first 5 elements to check output
print(custom_dict_list)
