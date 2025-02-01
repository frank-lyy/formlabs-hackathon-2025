from camera import *
import cv2
import numpy as np

def get_mask(image):
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_bound = np.array([200, 50, 40])
    upper_bound = np.array([270, 100, 100])
    return cv2.inRange(image_hsv, lower_bound, upper_bound)

def get_masked_image(image, mask):
    return cv2.bitwise_and(image, image, mask=mask)

def main():
    # Initialize camera
    zed = initialize_camera()

    while True:
        # Get data
        image, depth, points = get_camera_data(zed)

        # Get masked image
        mask = get_mask(image)
        image_masked = get_masked_image(image, mask)

        # Show frame
        cv2.imshow("frame", image_masked)
        if cv2.waitKey(1) == ord("q"):
            break

    # Close the ZED
    zed.close()
    
if __name__ == "__main__":
    main()

