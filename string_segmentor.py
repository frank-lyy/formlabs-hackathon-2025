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
corners = None

def get_feature_index(string_name, reference_pc):
    """
    string_name is either "left" or "right"
    Return the best fit index of the feature described by reference_pc
    """
    target_pc = orange_data["target_points"] if string_name == "left" else blue_data["target_points"]
    best_index = segment_shoelace(target_pc, reference_pc)
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

def get_initial_pointcloud_order(mask, points, quad_mask, visualize=False):
    cleaned_points = clean_data(mask, points, quad_mask, visualize=visualize)
    ordered_initial_points = initial_pointcloud_order(cleaned_points)
    
    return ordered_initial_points

def realtime_track_state(source_pointcloud, mask, points):
    target_data = {
        "mask": [mask],
        "points": [points]
    }
    cleaned_data = clean_data(target_data)
    ordered_target_pointcloud = track_state(source_pointcloud, cleaned_data["initial_points"])
    
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
            # Draw the point
            cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
            # Draw the order number
            cv2.putText(frame, str(len(corners)), (x+10, y+10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.imshow(window_name, frame)
            
            if len(corners) == 4:
                cv2.waitKey(500)  # Give time to see the last point
                cv2.destroyWindow(window_name)
    
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
    
    # Wait for points selection or quit
    while len(corners) < 4:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            cv2.destroyWindow(window_name)
            return None
    
    return np.array(corners)

def main():
    # Initialize camera
    zed = initialize_camera()
    prev_time = time.time()

    # corner choosing stage
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
        if time.time() - prev_time > 3 and record_data:
            if corners is None:
                # Pause recording and get corner points
                corners = get_corner_points(image)
                print("Corner points selected:", corners)
                # Create the quad mask once we have the corners
                quad_mask = create_quad_mask(image.shape, corners)
            else:
                print("Corners already selected")
                break

        # Move on to next step
        if cv2.waitKey(1) == ord("\n"):
            break

    # initialization stage
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
        if time.time() - prev_time > 3 and record_data:
            prev_time = time.time()
            orange_data["source_points"] = get_initial_pointcloud_order(mask_orange, points, quad_mask, visualize=True)
            blue_data["source_points"] = get_initial_pointcloud_order(mask_blue, points, quad_mask, visualize=True)

        # Quit
        if cv2.waitKey(1) == ord("\n"):
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
