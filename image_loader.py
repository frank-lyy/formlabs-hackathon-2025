# Load in Images from /images directory
import os
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

def view_depth(filename, width, height):
    # Read the raw data
    depth_data = np.fromfile(filename, dtype=np.uint16)
    
    # Reshape the data into a 2D array
    depth_image = depth_data.reshape((height, width))
    
    # Display the depth image
    plt.imshow(depth_image, cmap='viridis')  # viridis is good for depth visualization
    plt.colorbar(label='Depth')
    plt.title('Depth Image')
    plt.show()

def view_color(filename, width, height):
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

def get_color(filename, width, height):
    # Read data from png file
    image = Image.open(filename)
    resized_image = image.resize((width, height))  # e.g., (800, 600)
    color_data = np.array(resized_image)
    
    return color_data

def get_mask(filename, width, height, remove_sides=False):
    color_data = get_color(filename, width, height)

    mask = np.zeros((height, width), dtype=np.bool_)
    mask[color_data[:, :, 0] < 50] = True  # Assuming RGB values
    
    # My poor heuristic that deletes the sides of the image from the mask 
    # (in case some of those areas are dark)
    if remove_sides:
        for i in range(0, 20):
            mask[:, i] = False
            mask[:, width - i - 1] = False
            mask[i, :] = False
            mask[height - i - 1, :] = False
    
    # print(mask.shape)
    return mask

if __name__ == "__main__":
    # view_raw_depth(first_file, width=424, height=240)
    image1 = get_color("images/1_Color.png", width=424, height=240)
    mask1 = get_mask("images/1_Color.png", width=424, height=240)
    # view_raw_depth(second_file, width=424, height=240)
    image2 = get_color("images/2_Color.png", width=424, height=240)
    mask2 = get_mask("images/2_Color.png", width=424, height=240)
    image1[~mask1] = 0
    image2[~mask2] = 0

    plt.subplot(1, 2, 1)
    plt.imshow(image1)
    plt.subplot(1, 2, 2)
    plt.imshow(image2)
    plt.show()
