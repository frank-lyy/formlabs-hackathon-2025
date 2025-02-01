from camera import *
import cv2
import numpy as np
import time

FPS = 5

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
        "image": [],
        "mask_orange": [],
        "mask_blue": [],
        "depth": [],
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
        if time.time() - prev_time > 1 / FPS:
            prev_time = time.time()
            data["image"].append(image)
            data["mask_blue"].append(mask_blue)
            data["mask_orange"].append(mask_orange)
            data["depth"].append(depth)
            data["points"].append(points)

        # Quit
        if cv2.waitKey(1) == ord("q"):
            cv2.imwrite("./images/test_zed.png", image)
            break
        
    # Save data
    np.savez("data.npz", data)

    # Close the ZED
    zed.close()
    
if __name__ == "__main__":
    main()

