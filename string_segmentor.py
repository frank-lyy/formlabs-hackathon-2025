from camera import *
import cv2
import numpy as np

def get_mask(image):
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_bound = np.array([40, 0, 0])
    upper_bound = np.array([140, 100, 90])
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
        print(mask)
        image_masked = get_masked_image(image, mask)

        # Show frame
        cv2.imshow("image", image)
        cv2.imshow("mask", mask)
        cv2.imshow("image_masked", image_masked)
        if cv2.waitKey(1) == ord("q"):
            cv2.imwrite("./images/test_zed.png", image)
            break

    # Close the ZED
    zed.close()
    
if __name__ == "__main__":
    main()

