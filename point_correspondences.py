from image_loader import *
from data_loader import *
from point_ordering import *

import numpy as np
from pycpd import DeformableRegistration
import matplotlib.pyplot as plt
from functools import partial
from sklearn.neighbors import NearestNeighbors

def get_pointcloud_from_image(filename, width, height, remove_sides=False):
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

def initial_pointcloud_order(pointcloud, visualize=False):
    # Normalize the pointcloud around unit hypersphere
    normed_pointcloud, _, _ = normalize_pointcloud(pointcloud)
    
    ordered_pointcloud, colors = order_points_by_y(pointcloud, normed_pointcloud)
    
    # visualize this pointcloud, on a color scale based on angle
    if visualize:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(normed_pointcloud[:, 0], normed_pointcloud[:, 1], normed_pointcloud[:, 2], c=colors)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()

    return ordered_pointcloud

def visualize_cpd(iteration, error, X, Y, ax):
    plt.cla()
    ax.scatter(X[:, 0],  X[:, 1], X[:, 2], color='red', label='Target')
    ax.scatter(Y[:, 0],  Y[:, 1], Y[:, 2], color='blue', label='Transformed Source')
    ax.text2D(0.87, 0.92, 'Iteration: {:d}'.format(
        iteration), horizontalalignment='center', verticalalignment='center', transform=ax.transAxes, fontsize='x-large')
    ax.legend(loc='upper left', fontsize='x-large')
    plt.draw()
    plt.pause(0.01)

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

def track_state(source_pointcloud, target_pointcloud, visualize=False):
    TY = pc_registration(source_pointcloud, target_pointcloud, visualize=visualize)

    nn = NearestNeighbors(n_neighbors=1, algorithm='kd_tree')
    nn.fit(target_pointcloud)

    distances, indices = nn.kneighbors(TY)

    # Create a mapping of each unique target index to all the TY points that map to it
    target_to_source = {}
    for source_idx, (target_idx, dist) in enumerate(zip(indices.flatten(), distances.flatten())):
        if target_idx not in target_to_source:
            target_to_source[target_idx] = (source_idx, dist)
        elif dist < target_to_source[target_idx][1]:
            target_to_source[target_idx] = (source_idx, dist)
    
    # For each target point that has multiple matches, keep only the closest one
    final_source_indices = []
    for target_idx, (source_idx, dist) in target_to_source.items():
        final_source_indices.append((source_idx, target_idx))
    
    # Sort by source index to maintain original order
    final_source_indices = sorted(final_source_indices, key=lambda x: x[0])
    ordered_target_indices = [target_idx for _, target_idx in final_source_indices]
    ordered_target_pointcloud = target_pointcloud[ordered_target_indices, :]

    if visualize:
        # color based on increasing index
        colors = np.interp(range(len(ordered_target_pointcloud)), [0, len(ordered_target_pointcloud)], [0, 1])
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(ordered_target_pointcloud[:, 0], ordered_target_pointcloud[:, 1], ordered_target_pointcloud[:, 2], c=colors)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.show()

    return ordered_target_pointcloud

def track_state_from_images(filename_t1, filename_t2, visualize=False):
    pointcloud_1 = get_pointcloud_from_image(filename_t1, width=424, height=240, remove_sides=True)
    pointcloud_1 = initial_pointcloud_order(pointcloud_1)
    pointcloud_2 = get_pointcloud_from_image(filename_t2, width=424, height=240, remove_sides=True)

    TY = pc_registration(pointcloud_1, pointcloud_2, visualize=visualize)

    # order the points in pointcloud_2 based on the order of the closest points in TY
    nn = NearestNeighbors(n_neighbors=1, algorithm='kd_tree')
    nn.fit(pointcloud_2)
    
    distances, indices = nn.kneighbors(TY)
    # could probably also use the more complicated unique_indices method but this is easier for now
    unique_indices = indices[np.sort(np.unique(indices, return_index=True)[1])].flatten()
    
    new_pointcloud_2 = pointcloud_2[unique_indices, :]
    # print(pointcloud_2)
    # print(new_pointcloud_2)
    return new_pointcloud_2

def visualize_labeled_pointclouds_from_images(filename_t1, filename_t2):
    image1 = get_color(filename_t1+"_Color.png", width=424, height=240)
    image2 = get_color(filename_t2+"_Color.png", width=424, height=240)
    pointcloud_1 = get_pointcloud_from_image(filename_t1, width=424, height=240, remove_sides=True)
    pointcloud_2 = get_pointcloud_from_image(filename_t2, width=424, height=240, remove_sides=True)
    
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
    data = load_data("data/data2.npz")
    quad_mask = create_quad_mask(data['mask'][0].shape, corners=CORNERS)
    cleaned_data = {'points': []}
    cleaned_data['initial_points'] = clean_data(data['mask'][0], data['points'][0], quad_mask)
    for i in range(1, len(data['points'])):
        cleaned_data['points'].append(clean_data(data['mask'][i], data['points'][i], quad_mask))
    source_pointcloud = cleaned_data['initial_points']
    source_pointcloud = initial_pointcloud_order(source_pointcloud, visualize=True)
    
    for i in range(len(cleaned_data['points'])):
        target_pointcloud = cleaned_data['points'][i]
        ordered_target_pointcloud = track_state(source_pointcloud, target_pointcloud, visualize=True)
        source_pointcloud = ordered_target_pointcloud
        # if i > 3:
        #     break
