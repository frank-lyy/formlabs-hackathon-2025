from camera import *
from data_loader import clean_data

import cv2
import numpy as np
import time

FPS = 2
record_data = True

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

def main():
    # Initialize camera
    zed = initialize_camera()
    prev_time = time.time()

    # Store data
    data = {
        "mask": [],
        "points": [],
    }

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
            data["mask"].append(mask_blue)
            data["points"].append(points)

        # Quit
        if cv2.waitKey(1) == ord("q"):
            cv2.imwrite("./images/test_zed.png", image)
            break
        
    # Save data
    if record_data:
        print("recording data...")
        for key, val in data.items():
            data[key] = np.array(val)
        cleaned_data = clean_data(data)
        np.savez("data/video.npz", **cleaned_data)

    # Close the ZED
    zed.close()
    
if __name__ == "__main__":
    main()

