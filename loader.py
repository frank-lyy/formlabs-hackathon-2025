# Load in Images from /images directory
import os
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

def view_raw_depth(filename, width, height):
    # Read the raw data
    depth_data = np.fromfile(filename, dtype=np.uint16)
    
    # Reshape the data into a 2D array
    depth_image = depth_data.reshape((height, width))
    
    # Display the depth image
    plt.imshow(depth_image, cmap='viridis')  # viridis is good for depth visualization
    plt.colorbar(label='Depth')
    plt.title('Depth Image')
    plt.show()

def view_raw_color(filename, width, height):
    # Read data from png file
    color_data = plt.imread(filename)
    
    # Display the color image
    plt.imshow(color_data)
    plt.title('Color Image')
    plt.show()

def get_depth(filename, width, height):
    # Read the raw data
    depth_data = np.fromfile(filename, dtype=np.uint16)
    
    # Reshape the data into a 2D array
    depth_image = depth_data.reshape((height, width))
    
    return depth_image

def get_mask(filename, width, height):
    # Read data from png file
    image = Image.open(filename)

    # Resize the image
    resized_image = image.resize((width, height))  # e.g., (800, 600)
    color_data = np.array(resized_image)
    
    # Create a mask where the object is colored and the background is white
    mask = np.zeros((height, width), dtype=np.bool_)
    mask[color_data[:, :, 0] < 50] = True  # Assuming RGB values
    
    # # Display the mask
    # plt.imshow(mask, cmap='gray')
    # plt.title('Mask')
    # plt.show()

    return mask

if __name__ == "__main__":
    # Update with your actual .raw file path
    first_file = os.path.join('images', '1_Depth.raw')  # Assuming the raw file is in the images directory
    # view_raw_depth(first_file, width=424, height=240)
    get_mask("images/1_Color.png", width=424, height=240)
    second_file = os.path.join('images', '2_Depth.raw')  # Assuming the raw file is in the images directory
    # view_raw_depth(second_file, width=424, height=240)
    get_mask("images/2_Color.png", width=424, height=240)
