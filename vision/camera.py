import pyzed.sl as sl
import numpy as np

def initialize_camera():
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.AUTO
    init_params.camera_fps = 30
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
        pass

    zed.retrieve_image(image, sl.VIEW.LEFT)
    zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
    zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

    image_np = np.array(image.numpy()[:,:,:3], dtype=np.uint8)
    depth_np = np.array(depth.numpy(), dtype=np.float32)
    point_cloud_np = np.array(point_cloud.numpy()[:,:,:3], dtype=np.float32)

    return image_np, depth_np, point_cloud_np

def main():
    # Initialize camera
    zed = initialize_camera()

    # Get data (image is in BGR)
    image, depth, points = get_camera_data(zed)
    
    # print(f"Image: {image}\n")
    # print(f"Depth: {depth}\n")
    # print(f"Points: {points}")

    # Close the ZED
    zed.close()
        
if __name__ == "__main__":
    main()
