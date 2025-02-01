import pyzed.sl as sl
import numpy as np
import time

def initialize_camera():
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_units = sl.UNIT.METER

    # Open the camera
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print("Camera Open : "+repr(status)+". Exit program.")
        exit()
    return zed

def get_camera_data(zed):
    image = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()

    runtime_parameters = sl.RuntimeParameters()
    while zed.grab(runtime_parameters) != sl.ERROR_CODE.SUCCESS:
        time.sleep(1/60)

    zed.retrieve_image(image, sl.VIEW.LEFT)
    zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
    zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

    return image, depth, point_cloud

def main():
    # Initialize camera
    zed = initialize_camera()

    # Get data
    image, depth, points = get_camera_data(zed)
    
    print(f"Image: {image}\n")
    print(f"Depth: {depth}\n")
    print(f"Points: {points}")

    zed.close()
        
if __name__ == "__main__":
    main()
