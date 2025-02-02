from camera import *
from data_loader import *
from point_correspondences import *
from planner import *

import cv2
import numpy as np
import time

FPS = 2
record_data = True

# Store data
orange_data = {}
blue_data = {}

def get_feature_index(string_name, reference_pc):
    """
    string_name is either "left" or "right"
    Return the best fit index of the feature described by reference_pc
    """
    target_pc = orange_data["target_points"] if string_name == "left" else blue_data["target_points"]
    best_index = segment_shoelace(target_pc)
    return best_index
    
def get_position_index(string_name, pos):
    """
    string_name is either "left" or "right"
    Return the index of the position pos (0 <= pos < 1)
    """
    target_pc = orange_data["target_points"] if string_name == "left" else blue_data["target_points"]
    return int(pos * len(target_pc))
    
def get_position_from_index(string_name, idx):
    """
    string_name is either "left" or "right"
    Return the (x, y, z) position of the index idx (0 <= idx < len(string_name))
    """
    target_pc = orange_data["target_points"] if string_name == "left" else blue_data["target_points"]
    return target_pc[idx]


def get_mask_orange(image):
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_bound = np.array([0, 50, 130])
    upper_bound = np.array([25, 255, 255])
    return cv2.inRange(image_hsv, lower_bound, upper_bound)

def get_mask_blue(image):
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_bound = np.array([100, 20, 30])
    upper_bound = np.array([179, 255, 255])
    return cv2.inRange(image_hsv, lower_bound, upper_bound)

def get_masked_image(image, mask):
    return cv2.bitwise_and(image, image, mask=mask)

def get_initial_pointcloud_order(mask, points):
    data = {
        "mask": [mask],
        "points": [points]
    }
    cleaned_data = clean_data(data)
    ordered_initial_points = initial_pointcloud_order(cleaned_data["initial_points"])
    
    return ordered_initial_points

def realtime_track_state(source_pointcloud, mask, points):
    target_data = {
        "mask": [mask],
        "points": [points]
    }
    cleaned_data = clean_data(target_data)
    ordered_target_pointcloud = track_state(source_pointcloud, cleaned_data["initial_points"])
    
    return ordered_target_pointcloud

def main():
    # Initialize camera
    zed = initialize_camera()
    prev_time = time.time()

    while True:
        # Get data
        image, depth, points = get_camera_data(zed)

        # Get masked image
        mask_orange = get_mask_orange(image)
        mask_blue = get_mask_blue(image)
        image_orange = get_masked_image(image, mask_orange)
        image_blue = get_masked_image(image, mask_blue)

        # Show frame
        cv2.imshow("image", image)
        cv2.imshow("orange", image_orange)
        cv2.imshow("blue", image_blue)

        # Store data
        if time.time() - prev_time > 1 / FPS and record_data:
            prev_time = time.time()
            orange_data["source_points"] = get_initial_pointcloud_order(mask_orange, points)
            blue_data["source_points"] = get_initial_pointcloud_order(mask_blue, points)

        # Quit
        if cv2.waitKey(1) == ord(" "):
            print("Initial states set. Begin Tracking.")
            break

    while True:
        # Get data
        image, depth, points = get_camera_data(zed)

        # Get masked image
        mask_orange = get_mask_orange(image)
        mask_blue = get_mask_blue(image)
        image_orange = get_masked_image(image, mask_orange)
        image_blue = get_masked_image(image, mask_blue)

        # Show frame
        cv2.imshow("image", image)
        cv2.imshow("orange", image_orange)
        cv2.imshow("blue", image_blue)

        # Store data
        if time.time() - prev_time > 1 / FPS and record_data:
            prev_time = time.time()
            blue_data["target_points"] = realtime_track_state(blue_data["source_points"], mask_blue, points)
            orange_data["target_points"] = realtime_track_state(orange_data["source_points"], mask_orange, points)
            blue_data["source_points"] = blue_data["target_points"]
            orange_data["source_points"] = orange_data["target_points"]


        # Quit
        if cv2.waitKey(1) == ord("q"):
            break

    # Close the ZED
    zed.close()
    
if __name__ == "__main__":
    main()

