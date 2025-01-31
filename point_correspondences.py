from image_loader import *
import numpy as np
from pycpd import DeformableRegistration
import matplotlib.pyplot as plt
from numpy.testing import assert_array_almost_equal
from functools import partial
from mpl_toolkits.mplot3d import Axes3D

def get_pointcloud(filename, width, height, remove_sides=False):
    depth_image = get_depth(filename+"_Depth.raw", width, height)
    mask = get_mask(filename+"_Color.png", width, height, remove_sides=remove_sides)
    
    pointcloud = []
    for i in range(depth_image.shape[0]):
        for j in range(depth_image.shape[1]):
            if mask[i, j]:
                pointcloud.append([i, j, depth_image[i, j]])
                
    return np.array(pointcloud)

def normalize_pointcloud(points):
    centroid = np.mean(points, axis=0)
    points_centered = points - centroid
    scale = np.max(np.linalg.norm(points_centered, axis=1))
    points_normalized = points_centered / scale
    return points_normalized, centroid, scale

def visualize_cpd(iteration, error, X, Y, ax):
    plt.cla()
    ax.scatter(X[:, 0],  X[:, 1], X[:, 2], color='red', label='Target')
    ax.scatter(Y[:, 0],  Y[:, 1], Y[:, 2], color='blue', label='Transformed Source')
    ax.text2D(0.87, 0.92, 'Iteration: {:d}'.format(
        iteration), horizontalalignment='center', verticalalignment='center', transform=ax.transAxes, fontsize='x-large')
    ax.legend(loc='upper left', fontsize='x-large')
    plt.draw()
    plt.pause(0.1)

def pc_registration(pointcloud_1, pointcloud_2, visualize=False):
    """ 
    Returns the transformed initial pointcloud
    """
    pc1_norm, pc1_centroid, pc1_scale = normalize_pointcloud(pointcloud_1)
    pc2_norm, pc2_centroid, pc2_scale = normalize_pointcloud(pointcloud_2)
    
    reg = DeformableRegistration(
        X=pc2_norm,
        Y=pc1_norm,
        beta=2,
        alpha=3,
        max_iterations=150
    )
    
    if visualize:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        callback = partial(visualize_cpd, ax=ax)
        
        TY_norm, _ = reg.register(callback)
        plt.show()
    else:
        TY_norm, _ = reg.register()
    
    TY = (TY_norm * pc2_scale) + pc2_centroid
    
    return TY

def visualize_labeled_pointclouds(filename_t1, filename_t2):
    image1 = get_color(filename_t1+"_Color.png", width=424, height=240)
    image2 = get_color(filename_t2+"_Color.png", width=424, height=240)
    pointcloud_1 = get_pointcloud(filename_t1, width=424, height=240, remove_sides=True)
    pointcloud_2 = get_pointcloud(filename_t2, width=424, height=240, remove_sides=True)
    
    # Get transformed points (normalization handled in pc_registration)
    TY = pc_registration(pointcloud_1, pointcloud_2, visualize=True)

    # sanity check
    assert pointcloud_1.shape[0] == TY.shape[0]

    # sample a few points from each image
    num_points = 5
    indices = np.random.choice(pointcloud_1.shape[0], num_points, replace=False)
    pointcloud_1 = pointcloud_1[indices, :]
    pointcloud_2 = np.round(TY[indices, :], 0).astype(int)
    
    # plot the images
    plt.subplot(1, 2, 1)
    plt.imshow(image1)
    plt.scatter(pointcloud_1[:, 1], pointcloud_1[:, 0])
    # After your scatter plot line, add:
    for idx, (x, y, _) in enumerate(pointcloud_1):
        plt.annotate(f'Point {idx}', (y, x), xytext=(5, 5), textcoords='offset points')
    
    plt.subplot(1, 2, 2)
    plt.imshow(image2)
    plt.scatter(pointcloud_2[:, 1], pointcloud_2[:, 0])
    # After your scatter plot line, add:
    for idx, (x, y, _) in enumerate(pointcloud_2):
        plt.annotate(f'Point {idx}', (y, x), xytext=(5, 5), textcoords='offset points')
    
    plt.show()
    
if __name__ == "__main__":
    visualize_labeled_pointclouds("images/1", "images/2")