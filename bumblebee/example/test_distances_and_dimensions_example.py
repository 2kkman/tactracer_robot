import test_distances_and_dimensions as tdd

if __name__ == "__main__":
    distance_example = 750  # Example distance in mm
    
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
        df_new = tdd.read_file(file_path)
        print(f"{file_path} 파일이 이미 존재합니다.")
        
    except FileNotFoundError:
        df_new = tdd.create_file(file_path, initial_distance, initial_width, initial_height, start=500, end=3000, step=10)
        
    shift_x, shift_y = tdd.calculate_translation(df_new, target_x_example, target_y_example, distance_example)
    print(f'shift x: {shift_x:.3f}, shift y: {shift_y:.3f}')

    print("get rectangle coordinates")
    topleft, topright, bottomleft, bottomright= tdd.get_rectangle_coordinates(df_new, shift_x, shift_y, distance_example)
    print(f"top left: {topleft}, top right: {topright}, bottom left: {bottomleft}, bottom right: {bottomright}")