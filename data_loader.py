import numpy as np
import matplotlib.pyplot as plt
import cv2
from pydrake.perception import PointCloud

CORNERS = np.array([(745, 148), (913, 335), (539, 620), (485, 330)])

def load_data(filename):
    data = np.load(filename)
    return data

def visualize_mask(mask):
    plt.imshow(mask)
    plt.show()

def improve_mask(mask, quad_mask, visualize=False):
    mask[mask > 0] = True
    mask[mask == 0] = False
    mask = np.logical_and(mask, quad_mask)

    if visualize:
        visualize_mask(quad_mask)
        
    return mask

def clean_data(data):
    """ 
    outputs a dictionary with 'points' key and a list of pointclouds as values
    pointclouds have been downsampled and (improved) masks have been applied
    """
    n = len(data['mask'])

    cleaned_data = {'points': []}
    
    # mask all points outside of the corners
    image_shape = data['mask'][0].shape
    quad_mask = np.zeros(image_shape, dtype=np.uint8)
    corners_array = CORNERS.reshape((-1, 1, 2)).astype(np.int32)
    cv2.fillPoly(quad_mask, [corners_array], 1)

    for i in range(n):
        mask = data['mask'][i]
        mask = improve_mask(mask, quad_mask, visualize=False)
        points = data['points'][i]
        cleaned_points = points[mask]

        pc = PointCloud(cleaned_points.shape[0])
        mutable_xyzs = pc.mutable_xyzs()
        mutable_xyzs[:] = cleaned_points.T
        downsampled_pc = pc.VoxelizedDownSample(voxel_size=0.01)
        downsampled_points = downsampled_pc.xyzs().T

        if i == 0:
            print("Downsampling removed {:d} points".format(cleaned_points.shape[0] - downsampled_points.shape[0]))
            cleaned_data['initial_points'] = downsampled_points
        else:
            cleaned_data['points'].append(downsampled_points)
        
    return cleaned_data

def visualize_helper(iteration, points, ax):
    ax.cla()
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='red', label='String (Masked)')
    ax.text2D(0.87, 0.92, 'Iteration: {:d}'.format(iteration), 
              horizontalalignment='center', verticalalignment='center', 
              transform=ax.transAxes, fontsize='x-large')
    ax.legend(loc='upper left', fontsize='x-large')
    
    # Add control instructions
    ax.text2D(0.02, 0.02, 
              'Controls:\nSpace: Play/Pause\nLeft/Right: Previous/Next\nQ: Quit', 
              horizontalalignment='left', verticalalignment='bottom',
              transform=ax.transAxes, fontsize='medium')
    
    plt.draw()

def visualize_data(cleaned_data):
    # Data is a dictionary of np arrays (mask and points)
    # This plots the different points in the image
    n = len(cleaned_data['points'])
    
    # Store all masked points for fixed axes
    all_cleaned_points = []

    all_cleaned_points.append(cleaned_data['initial_points'])

    for i in range(n):
        cleaned_points = cleaned_data['points'][i]
        all_cleaned_points.append(cleaned_points)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Initialize visualization state
    current_frame = [n]  # Use list to allow modification in callback
    paused = [True]
    
    def on_key(event):
        if event.key == 'q':
            plt.close(fig)
        elif event.key == ' ':  # Space bar
            paused[0] = not paused[0]
        elif event.key == 'left':
            current_frame[0] = max(0, current_frame[0] - 1)
            visualize_helper(current_frame[0] + 1, all_cleaned_points[current_frame[0]], ax)
        elif event.key == 'right':
            current_frame[0] = min(n, current_frame[0] + 1)
            visualize_helper(current_frame[0] + 1, all_cleaned_points[current_frame[0]], ax)
    
    fig.canvas.mpl_connect('key_press_event', on_key)

    for i in range(n+1):
        visualize_helper(i+1, all_cleaned_points[i], ax)

    # Animation loop
    print("Beginning animation")
    while plt.fignum_exists(fig.number):
        if not paused[0] and current_frame[0] <= n:
            print("Playing")
            visualize_helper(current_frame[0]+1, all_cleaned_points[current_frame[0]], ax)
            current_frame[0] += 1
        plt.pause(0.1)
    
if __name__ == "__main__":
    data = load_data("data/data2.npz")
    cleaned_data = clean_data(data)
    visualize_data(cleaned_data)