from data_loader import *
from point_ordering import *
from visualize_helpers import *

import numpy as np
from pycpd import DeformableRegistration
import matplotlib.pyplot as plt
from functools import partial
from sklearn.neighbors import NearestNeighbors
from scipy.interpolate import splprep, splev

def normalize_pointcloud(points):
    centroid = np.mean(points, axis=0)
    points_centered = points - centroid
    scale = np.max(np.linalg.norm(points_centered, axis=1))
    points_normalized = points_centered / scale
    return points_normalized, centroid, scale

def initial_pointcloud_order(pointcloud, visualize=False):
    # Normalize the pointcloud around unit hypersphere
    normed_pointcloud, _, _ = normalize_pointcloud(pointcloud)
    
    ordered_pointcloud, colors = order_points_by_z(pointcloud, normed_pointcloud)
    
    # visualize this pointcloud, on a color scale based on angle
    if visualize:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax = normalize_plot(normed_pointcloud, ax, three_d=True)
        ax.scatter(normed_pointcloud[:, 0], normed_pointcloud[:, 1], normed_pointcloud[:, 2], c=colors)
        plt.show()

    return ordered_pointcloud

def visualize_cpd(iteration, error, X, Y, ax):
    plt.cla()
    ax = normalize_plot(X, ax, three_d=True)
    ax.scatter(X[:, 0],  X[:, 1], X[:, 2], color='red', label='Target')
    ax.scatter(Y[:, 0],  Y[:, 1], Y[:, 2], color='blue', label='Transformed Source')
    ax.text2D(1.0, 1.0, 'Iteration: {:d}'.format(iteration), horizontalalignment='center', verticalalignment='center', transform=ax.transAxes, fontsize='x-large')
    ax.legend(loc='upper left', fontsize='medium')
    plt.draw()
    plt.pause(.1)

def pc_registration(pointcloud_1, pointcloud_2, visualize=False):
    """ 
    Returns the transformed initial pointcloud
    """
    pc1_norm, pc1_centroid, pc1_scale = normalize_pointcloud(pointcloud_1)
    pc2_norm, pc2_centroid, pc2_scale = normalize_pointcloud(pointcloud_2)
    
    reg = DeformableRegistration(
        X=pc2_norm,
        Y=pc1_norm,
        beta=50,
        alpha=5,
        max_iterations=20
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

def resample_points(points, s=0.1, k=5, num_resampled_points=100):
    """
    Fit a B-spline to the points and resample to get at least min_points points.
    Maintains the ordering of points along the spline.
    """
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    tck, u = splprep([x, y, z], s=s, k=k)

    # Calculate number of points needed
    x_new, y_new, z_new = splev(np.linspace(0, 1, num_resampled_points), tck)
    
    return np.column_stack([x_new, y_new, z_new])

def track_state(source_pointcloud, target_pointcloud, visualize=False):
    TY = pc_registration(source_pointcloud, target_pointcloud, visualize=visualize)

    nn = NearestNeighbors(n_neighbors=1, algorithm='kd_tree')
    nn.fit(TY)

    distances, indices = nn.kneighbors(target_pointcloud)

    # Create a mapping of each unique target index to all the TY points that map to it
    source_to_target = {}
    for target_idx, source_idx in enumerate(indices.flatten()):
        source_to_target.setdefault(source_idx, []).append(target_idx)
    
    # Iterate through the source indices and find the target indices that map to each source index
    # ordering by the minimum distance to the previous picked point
    ordered_target_indices = []
    for source_idx in range(TY.shape[0]):
        if source_idx not in source_to_target: continue
        target_indices = source_to_target[source_idx]
        if ordered_target_indices:
            while len(target_indices) > 0:
                previous_point = target_pointcloud[ordered_target_indices[-1]]
                target_indices = sorted(target_indices, key=lambda x: np.linalg.norm(target_pointcloud[x] - previous_point), reverse=True)
                next_index = target_indices.pop()
                ordered_target_indices.append(next_index)
        else:
            target_indices = sorted(target_indices, key=lambda x: target_pointcloud[x][2])
            ordered_target_indices.extend(target_indices)
    
    short_ordered_target_pointcloud = target_pointcloud[ordered_target_indices, :]
    ordered_target_pointcloud = short_ordered_target_pointcloud
    
    if visualize:
        # color based on increasing index
        num_points = len(ordered_target_pointcloud)
        colors = np.zeros((num_points, 3))
        t = np.linspace(0, 1, num_points)
        colors[:, 0] = np.interp(t, [0, 1], [50, 200])
        colors[:, 2] = np.interp(t, [0, 1], [200, 50])
        # highlight first point
        colors[:5] = np.array([[255, 20, 147]] * 5)
        colors[-5:] = np.array([[0, 255, 255]] * 5)
        colors = colors / 255.0

        
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax = normalize_plot(ordered_target_pointcloud, ax, three_d=True)
        ax.scatter(ordered_target_pointcloud[:, 0], ordered_target_pointcloud[:, 1], ordered_target_pointcloud[:, 2], c=colors)
        plt.show()

    return ordered_target_pointcloud


if __name__ == "__main__":
    cleaned_data = load_data("../data/data.json")
    cleaned_data = [remove_outliers(points, eps=0.01) for points in cleaned_data]
    cleaned_data = [downsample_points(points, voxel_size=0.005) for points in cleaned_data]
    source_pointcloud, pointclouds = cleaned_data[0], cleaned_data[1:]
    source_pointcloud = initial_pointcloud_order(source_pointcloud, visualize=True)
    
    for i in range(len(pointclouds)):
        visualize = i % 5 == 0
        target_pointcloud = pointclouds[i]
        ordered_target_pointcloud = track_state(source_pointcloud, target_pointcloud, visualize=visualize)
        source_pointcloud = ordered_target_pointcloud
