from pydrake.all import (
    StartMeshcat,
    AddDefaultVisualization,
    Simulator,
    RobotDiagramBuilder,
    SceneGraphCollisionChecker,
    RandomGenerator,
    PointCloud,
    Rgba,
    Quaternion,
    RigidTransform,
    RotationMatrix,
)

from camera import *
from data_loader import *
from point_correspondences import *
from planner import *

import cv2
import numpy as np
import time
import threading
import json

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

src_directory = os.path.dirname(os.path.abspath(__file__))
parent_directory = os.path.dirname(src_directory)
data_directory = os.path.join(parent_directory)
scene_yaml_file = os.path.join(data_directory, "assets", "robot.dmd.yaml")

meshcat = StartMeshcat()

robot_diagram_builder = RobotDiagramBuilder()
parser = robot_diagram_builder.parser()
scene_graph = robot_diagram_builder.scene_graph()
parser.package_map().Add("Robot.SLDASM", os.path.join(data_directory, "assets/Robot.SLDASM"))
parser.package_map().Add("Endowrist Mockup.SLDASM", os.path.join(data_directory, "assets/Endowrist Mockup.SLDASM"))
parser.package_map().Add("assets", os.path.join(data_directory, "assets"))
robot_model_instances = parser.AddModels(scene_yaml_file)
plant = robot_diagram_builder.plant()
plant.Finalize()
diagram = robot_diagram_builder.Build()
simulator = Simulator(diagram)
context = simulator.get_mutable_context()
plant_context = plant.GetMyMutableContextFromRoot(context)

camera_frame = plant.GetFrameByName("zed2i_left_camera_optical_frame")
X_W_Cam = plant.CalcRelativeTransform(plant_context, plant.world_frame(), camera_frame)
print(f"X_W_Cam: {X_W_Cam.translation()}, {X_W_Cam.rotation().matrix()}")

FPS = 4
NUM_CLUSTERS = 500
USE_KMEANS = False
record_data = True

# Store data
class StringState:
    def __init__(self):
        self.orange_data = {}
        self.blue_data = {}
        #self.corners = np.array([(367, 269), (853, 264), (919, 598), (338, 577)]) 
        self.corners = None
        self.orange_lock = threading.Lock()
        self.blue_lock = threading.Lock()
    
    def set_orange_data(self, new_data):
        with self.orange_lock:
            self.orange_data["target_points"] = new_data
            self.orange_data["source_points"] = new_data
    
    def set_blue_data(self, new_data):
        with self.blue_lock:
            self.blue_data["target_points"] = new_data
            self.blue_data["source_points"] = new_data
    
    def get_orange_data(self):
        with self.orange_lock:
            return self.orange_data["target_points"]
    
    def get_blue_data(self):
        with self.blue_lock:
            return self.blue_data["target_points"]

string_state = StringState()

def get_feature_index(string_name, reference_pc):
    """
    string_name is either "left" or "right"
    Return the best fit index of the feature described by reference_pc
    """
    target_pc = string_state.get_orange_data() if string_name == "left" else string_state.get_blue_data()
    best_index = match_template(target_pc, reference_pc, window_size=5)
    return best_index
    # return 4
    
def get_position_index(string_name, pos):
    """
    string_name is either "left" or "right"
    Return the index of the position pos (0 <= pos < 1)
    """
    target_pc = string_state.get_orange_data() if string_name == "left" else string_state.get_blue_data()
    return int(pos * len(target_pc))
    # if pos == 0:
    #     return 0
    # elif pos == 0.5:
    #     return 5
    # elif pos == 0.6:
    #     return 6
    # elif pos == 0.95:
    #     return 9
    
def get_position_from_index(string_name, idx):
    """
    string_name is either "left" or "right"
    Return the (x, y, z) position of the index idx (0 <= idx < len(string_name))
    """
    target_pc = string_state.get_orange_data() if string_name == "left" else string_state.get_blue_data()
    return target_pc[idx]
    # if string_name == "left":
    #     if idx == 0:
    #         return np.array([0, 0.05, 0.35])
    #     elif idx == 5:
    #         return np.array([0, -0.05, 0.4])
    #     elif idx == 6:
    #         return np.array([0, -0.06, 0.41])
    #     elif idx == 9:
    #         return np.array([0, -0.1, 0.45])
    # else:
    #     if idx == 0:
    #         return np.array([0.12, 0.05, 0.35])
    #     elif idx == 5:
    #         return np.array([0.12, -0.05, 0.4])
    #     elif idx == 6:
    #         return np.array([0.12, -0.06, 0.41])
    #     elif idx == 9:
    #         return np.array([0.12, -0.1, 0.45])
    #     else:
    #         return np.array([0.12, -0.03, 0.37])


def get_mask_orange(image):
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_bound = np.array([15, 160, 150])
    upper_bound = np.array([30, 255, 255])
    return cv2.inRange(image_hsv, lower_bound, upper_bound)

def get_mask_blue(image):
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_bound = np.array([135, 0, 60])
    upper_bound = np.array([179, 85, 140])
    return cv2.inRange(image_hsv, lower_bound, upper_bound)

def get_masked_image(image, mask):
    return cv2.bitwise_and(image, image, mask=mask)

def get_initial_pointcloud_order(cleaned_points, visualize=False):
    ordered_initial_points = initial_pointcloud_order(cleaned_points, visualize=visualize)
    return ordered_initial_points

def get_state(source_pointcloud, cleaned_points, visualize=False):
    ordered_target_pointcloud = track_state(source_pointcloud, cleaned_points, visualize=visualize)
    return ordered_target_pointcloud

def get_corner_points(frame):
    """
    Let user select 4 corner points by clicking on the frame.
    Returns the corner points in clockwise order starting from top-left.
    """
    corners = []
    window_name = "Select 4 Corner Points (Clockwise from Top-Left)"
    
    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            corners.append((x, y))
            print(f"Selected point: ({x}, {y})")
            # Draw the point
            cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
            # Draw the order number
            cv2.putText(frame, str(len(corners)), (x+10, y+10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.imshow(window_name, frame)

    # Create window and set mouse callback
    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)
    
    # Display instructions
    instruction_frame = frame.copy()
    instructions = [
        "Click to select 4 corner points in clockwise order:",
        "1. Top-Left",
        "2. Top-Right",
        "3. Bottom-Right",
        "4. Bottom-Left",
        "Press 'q' to cancel"
    ]
    
    y_offset = 30
    for i, instruction in enumerate(instructions):
        cv2.putText(instruction_frame, instruction, (10, y_offset + i*25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    cv2.imshow(window_name, instruction_frame)
    
    # # Wait for points selection or quit
    while len(corners) < 4:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyWindow(window_name)
            print("Returning from get_corner_points: None")
            return None
    
    assert len(corners) == 4, "Need 4 corner points"
    print("Returning from get_corner_points:", corners)
    cv2.destroyWindow(window_name)
    cv2.waitKey(1)
    return np.array(corners)

def main(stop_event):
    # Initialize camera
    zed = initialize_camera()
    prev_time = time.time()
    prev_image_time = time.time()

    # Corner choosing stage
    while not stop_event.is_set():
        # Get data
        image, depth, points = get_camera_data(zed)

        # Store data
        if time.time() - prev_time > 3 and record_data:
            if string_state.corners is None:
                # Pause recording and get corner points
                string_state.corners = get_corner_points(image)
                print("Corner points selected:", string_state.corners)
                # Create the quad mask once we have the corners
                quad_mask = create_quad_mask(depth.shape, string_state.corners)
            else:
                print("Corners already selected")
                quad_mask = create_quad_mask(depth.shape, string_state.corners)
                break

        # Move on to next step
        if cv2.waitKey(1) == ord("\n"):
            break

    # initialization stage
    while not stop_event.is_set():
        # Get data
        image, depth, points = get_camera_data(zed)

        # Get masked image
        mask_orange = get_mask_orange(image)
        mask_blue = get_mask_blue(image)
        # image_orange = get_masked_image(image, mask_orange)
        # image_blue = get_masked_image(image, mask_blue)

        # Store data
        if time.time() - prev_time > 5 and record_data:
            prev_time = time.time()
            mask_blue
            cleaned_blue = clean_data(mask_blue, points, quad_mask, voxel_size=0.005, visualize=False, kmeans=USE_KMEANS, k=NUM_CLUSTERS)
            cleaned_orange = clean_data(mask_orange, points, quad_mask, voxel_size=0.005, visualize=False, kmeans=USE_KMEANS, k=NUM_CLUSTERS)
            
            improved_mask_orange = improve_mask(mask_orange, quad_mask, visualize=False)
            improved_mask_blue = improve_mask(mask_blue, quad_mask, visualize=False)
            cleaned_orange_image = image.copy()
            cleaned_orange_image[~improved_mask_orange] = 0
            cleaned_blue_image = image.copy()
            cleaned_blue_image[~improved_mask_blue] = 0
            side_by_side = cv2.hconcat([cleaned_blue_image, cleaned_orange_image])
            cv2.namedWindow("Blue | Orange", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Blue | Orange", 800, 600)
            cv2.imshow("Blue | Orange", side_by_side)

            string_state.orange_data["source_points"] = get_initial_pointcloud_order(cleaned_orange, visualize=True)
            string_state.blue_data["source_points"] = get_initial_pointcloud_order(cleaned_blue, visualize=True)

        # Quit
        if cv2.waitKey(1) == ord("n"):
            print("Initial states set. Begin Tracking.")
            print("There are {} orange points and {} blue points".format(len(string_state.orange_data["source_points"]), len(string_state.blue_data["source_points"])))
            break
    
    data = []
    while not stop_event.is_set():
        # Get data
        image, depth, points = get_camera_data(zed)

        # Get masked image
        mask_orange = get_mask_orange(image)
        mask_blue = get_mask_blue(image)
        # image_orange = get_masked_image(image, mask_orange)
        # image_blue = get_masked_image(image, mask_blue)

        # Store data
        if time.time() - prev_time > 1 / FPS and record_data:
            prev_time = time.time()
            cleaned_blue = clean_data(mask_blue, points, quad_mask, voxel_size=0.005, visualize=False, kmeans=USE_KMEANS, k=NUM_CLUSTERS)
            cleaned_orange = clean_data(mask_orange, points, quad_mask, voxel_size=0.005, visualize=False, kmeans=USE_KMEANS, k=NUM_CLUSTERS)

            improved_mask_orange = improve_mask(mask_orange, quad_mask, visualize=False)
            improved_mask_blue = improve_mask(mask_blue, quad_mask, visualize=False)
            cleaned_orange_image = image.copy()
            cleaned_orange_image[~improved_mask_orange] = 0
            cleaned_blue_image = image.copy()
            cleaned_blue_image[~improved_mask_blue] = 0
            side_by_side = cv2.hconcat([cleaned_blue_image, cleaned_orange_image])
            cv2.resizeWindow("Blue | Orange", 800, 600)
            cv2.imshow("Blue | Orange", side_by_side)
            # relevant_depth = points[improved_mask_orange]
            # print(relevant_depth.shape)
            # data.append(relevant_depth.tolist())
            

            if time.time() - prev_image_time > 5:
                prev_image_time = time.time()
                new_orange_target_points = get_state(string_state.orange_data["source_points"], cleaned_orange, visualize=True)
                new_blue_target_points = get_state(string_state.blue_data["source_points"], cleaned_blue, visualize=False)
                segments = pointcloud_to_segments(new_orange_target_points, template=SLIGHT_BEND_TEMPLATE, visualize=False)
                visualize_segments_2d(new_orange_target_points, segments)
            else:
                new_orange_target_points = get_state(string_state.orange_data["source_points"], cleaned_orange, visualize=False)
                new_blue_target_points = get_state(string_state.blue_data["source_points"], cleaned_blue, visualize=False)
            string_state.set_orange_data(new_orange_target_points)
            string_state.set_blue_data(new_blue_target_points)

        # Quit
        if cv2.waitKey(1) == ord("q"):
            # with open('data/data.json', 'w+') as file:
                # json.dump(data, file)
            break

    # Close the ZED
    zed.close()
    
if __name__ == "__main__":
    stop_event = threading.Event()
    main(stop_event)
