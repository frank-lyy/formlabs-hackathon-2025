import numpy as np
import matplotlib.pyplot as plt
import cv2
import json
from pydrake.perception import PointCloud
from sklearn.cluster import DBSCAN, KMeans

from visualize_helpers import *

# CORNERS = np.array([(745, 148), (913, 335), (539, 620), (485, 330)])
CORNERS = None

def load_data(filename):
    if filename.endswith(".npz"):
        # this format corresponds to a raw (uncleaned) output from the ZED camera
        data = np.load(filename, allow_pickle=True)
        return data
    elif filename.endswith(".json"):
        # this format corresponds to a list of cleaned point clouds (after masking)
        with open(filename, "r") as f:
            data = json.load(f)
        cleaned_data = []
        for frame in data:
            cleaned_data.append(np.array(frame))
        return cleaned_data
    else:
        raise ValueError("File must be .npz or .json")

def visualize_mask(mask):
    plt.imshow(mask)
    plt.show()

def create_quad_mask(image_shape, corners=CORNERS):
    quad_mask = np.zeros(image_shape, dtype=np.uint8)
    corners_array = corners.reshape((-1, 1, 2)).astype(np.int32)
    cv2.fillPoly(quad_mask, [corners_array], 1)
    return quad_mask

def improve_mask(mask, quad_mask, visualize=False):
    mask[mask > 0] = True
    mask[mask == 0] = False
    mask = np.logical_and(mask, quad_mask)

    if visualize: visualize_mask(mask)
    return mask

def downsample_points(points, voxel_size=0.005, kmeans=False, k=500):
    if kmeans:
        kmeans = KMeans(n_clusters=k, random_state=0, n_init="auto").fit(points)
        downsampled_points = kmeans.cluster_centers_
    else:
        pc = PointCloud(points.shape[0])
        mutable_xyzs = pc.mutable_xyzs()
        mutable_xyzs[:] = points.T
        downsampled_pc = pc.VoxelizedDownSample(voxel_size=voxel_size)
        downsampled_points = downsampled_pc.xyzs().T
    return downsampled_points

def remove_outliers(points, eps=None, min_samples=5):
    """
    Remove outliers from point cloud data using DBSCAN clustering.
    
    Args:
        points (np.ndarray): Array of shape (n_points, 3) containing 3D points
        eps (float, optional): The maximum distance between two samples for them to be considered neighbors.
                             If None, automatically estimated based on data.
        min_samples (int): The minimum number of samples in a neighborhood for a point to be considered a core point.
    
    Returns:
        np.ndarray: Filtered points with outliers removed
    """
    if eps is None:
        # Estimate eps based on average distance between points; this step is computationally intensive
        distances = np.sqrt(np.sum((points[:, np.newaxis, :] - points[np.newaxis, :, :]) ** 2, axis=2))
        # Use the mean of the 10th percentile of distances as eps
        eps = np.percentile(distances[distances > 0], 10).mean()
        # from testing, it seems a good eps is ~0.01 for an undownsampled point cloud
    
    # Apply DBSCAN clustering
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
    
    # The labels_ array contains -1 for outliers and cluster indices (≥0) for inlier points
    inlier_mask = clustering.labels_ != -1
    # Get the largest cluster (assuming it's the main string)
    unique_labels, counts = np.unique(clustering.labels_[inlier_mask], return_counts=True)
    if len(unique_labels) > 0:
        main_cluster = unique_labels[np.argmax(counts)]
        main_cluster_mask = clustering.labels_ == main_cluster
        filtered_points = points[main_cluster_mask]
    else:
        # If no clusters found, return original points
        filtered_points = points
        print("Warning: No clusters found. Check eps and min_samples parameters.")
    
    # print(f"Removed {len(points) - len(filtered_points)} outlier points out of {len(points)} total points")
    return filtered_points

def clean_data(mask, points, quad_mask, voxel_size=0.005, visualize=False, kmeans=False, k=500):
    """ 
    takes in a single mask and pointcloud and returns a cleaned version
    pointclouds have been downsampled and (improved) masks have been applied
    """    
    mask = improve_mask(mask, quad_mask, visualize=visualize)
    masked_points = points[mask]
    masked_points = masked_points[~np.isnan(masked_points).any(axis=1)]
    cleaned_points = remove_outliers(masked_points, eps=0.01)
    downsampled_points = downsample_points(cleaned_points, voxel_size=voxel_size, kmeans=kmeans, k=k)
        
    return downsampled_points

def visualize_helper(iteration, points, ax):
    ax.cla()
    normalize_plot(points, ax, three_d=True)
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='red', label='String (Masked)')
    ax.text2D(1.1, 1.1, 'Iteration: {:d}'.format(iteration), 
              horizontalalignment='center', verticalalignment='center', 
              transform=ax.transAxes, fontsize='x-large')
    ax.text2D(-0.1, -0.1, 'Controls:\nSpace: Play/Pause\nLeft/Right: Previous/Next\nQ: Quit', 
              horizontalalignment='left', verticalalignment='bottom',
              transform=ax.transAxes, fontsize='medium')
    plt.draw()

def visualize_data(cleaned_data):
    # Data is a list of np arrays of shape (num_points, 3)
    # This plots the different points in the image
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Initialize visualization state
    n = len(cleaned_data)
    current_frame = [n-1]  # Use list to allow modification in callback
    paused = [True]
    
    def on_key(event):
        if event.key == 'q':
            plt.close(fig)
        elif event.key == ' ':  # Space bar
            paused[0] = not paused[0]
        elif event.key == 'left':
            current_frame[0] = (current_frame[0]-1) % n
            visualize_helper(current_frame[0]+1, cleaned_data[current_frame[0]], ax)
        elif event.key == 'right':
            current_frame[0] = (current_frame[0]+1) % n
            visualize_helper(current_frame[0]+1, cleaned_data[current_frame[0]], ax)
    
    fig.canvas.mpl_connect('key_press_event', on_key)

    for i in range(n):
        visualize_helper(i+1, cleaned_data[i], ax)

    # Animation loop
    print("Beginning animation")
    while plt.fignum_exists(fig.number):
        if not paused[0] and current_frame[0] < n:
            visualize_helper(current_frame[0]+1, cleaned_data[current_frame[0]], ax)
            current_frame[0] += 1
        plt.pause(0.1)
    
if __name__ == "__main__":
    cleaned_data = load_data("data/data.json")
    cleaned_data = [remove_outliers(points, eps=0.01) for points in cleaned_data]
    visualize_data(cleaned_data)
