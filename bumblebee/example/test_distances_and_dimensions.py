import pandas as pd
import os

# Create a dataframe with the calculated distances and dimensions
# print("Distance (mm) | Width (mm) | Height (mm)")
# print("----------------------------------------")
# for i in range(len(distances)):
#     # print("{:14} | {:11} | {:11}".format(distances[i], widths[i], heights[i]))
#     print(f"{distances[i]:14} | {widths[i]:.3f} | {heights[i]:.3f}")       
        
def read_file(file_path):
    res = pd.read_excel(file_path)
    return res

def create_file(file_path, initial_distance, initial_width, initial_height, start=500, end=3000, step=10):
    distances = list(range(start, end, step))
    widths = [(distance / initial_distance) * initial_width for distance in distances]
    heights = [(distance / initial_distance) * initial_height for distance in distances]
    
    df_new = pd.DataFrame({
        'Distance (mm)': distances,
        'Width (mm)': widths,
        'Height (mm)': heights
    })

    df_new['Width (mm)'] = df_new['Width (mm)'].round(3)
    df_new['Height (mm)'] = df_new['Height (mm)'].round(3)
    df_new.to_excel(file_path, index=False)
    print(f"{file_path} 파일이 생성되었습니다.")    
    

def get_closest_dimensions(data_frame, distance):
    # Find the row in the dataframe that is closest to the given distance
    closest_distance_idx = (data_frame['Distance (mm)'] - distance).abs().idxmin()
    closest_row = data_frame.iloc[closest_distance_idx]
    
    # Get the half width and half height for the given distance
    width = closest_row['Width (mm)']
    height = closest_row['Height (mm)']
    return width, height


def get_rectangle_coordinates(data_frame, target_x, target_y, distance, box_size = 10):
    
    # width, height = get_closest_dimensions(data_frame, distance)
    

    
    # Calculate the top-left, top-right, bottom-left, and bottom-right coordinates
    top_left = (target_x - box_size, target_y - box_size)
    top_right = (target_x + box_size, target_y - box_size)
    bottom_left = (target_x - box_size, target_y + box_size)
    bottom_right = (target_x + box_size, target_y + box_size)

    # top_left = top_left[0].round(3), top_left[1].round(3)
    # top_right = top_right[0].round(3), top_right[1].round(3)
    # bottom_left = bottom_left[0].round(3), bottom_left[1].round(3)
    # bottom_right = bottom_right[0].round(3), bottom_right[1].round(3)
    
    return top_left, top_right, bottom_left, bottom_right


def calculate_translation(data_frame, target_x, target_y, distance, video_width=4000, video_height=3000):
    # Find the row in the dataframe that is closest to the given distance

    width, height = get_closest_dimensions(data_frame, distance)
    # print(f"width: {width}, height: {height}")
    # Get the half width and half height for the given distance
    half_width = width / 2
    half_height = height / 2
    width_per_pixel = width / video_width
    height_per_pixel = height / video_height
    
    # Determine the translation needed in x and y directions
    shift_x = half_width - (width_per_pixel * target_x)
    shift_y = half_height - (height_per_pixel * target_y)
    
    return shift_x, shift_y



if __name__ == "__main__":
    print("-------------------start------------------------")
    # Example usage:
    distance_example = 750  # Example distance in mm
    center_x_example = 2000   # Example center x-coordinate
    center_y_example = 1500   # Example center y-coordinate
    
    # Given parameters
    initial_distance = 1000  # in mm
    initial_width = 1186.6   # in mm
    initial_height = 890     # in mm (4:3 ratio)

    video_width  = 4000
    video_height = 3000

    # Example usage:
    target_x_example = 2300   # Example target x-coordinate
    target_y_example = 700   # Example target y-coordinate


    # Save the dataframe to an Excel file
    file_path = "/root/catkin_ws/src/tta_blb/example/new_distances_and_dimensions.xlsx"

    try:
        df_new = read_file(file_path)
        print(f"{file_path} 파일이 이미 존재합니다.")
    except FileNotFoundError:
        
        create_file(file_path, initial_distance, initial_width, initial_height, start=500, end=3000, step=10)
        
    print("caluculate translation")
    shift_x, shift_y = calculate_translation(df_new, target_x_example, target_y_example, distance_example, video_width, video_height)
    print(f'shift x: {shift_x}, shift y: {shift_y}')
    
    print("get rectangle coordinates")
    topleft, topright, bottomleft, bottomright= get_rectangle_coordinates(df_new, shift_x, shift_y, distance_example)
    print(f"top left: {topleft}, top right: {topright}, bottom left: {bottomleft}, bottom right: {bottomright}")
