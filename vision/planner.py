from point_correspondences import *

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
from sklearn.decomposition import PCA
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
from enum import Enum

SHARP_BEND_TEMPLATE = np.array([[-1, 1], [0, 0], [1, 1]])  # 90-degree bend
U_BEND_TEMPLATE = np.array([[0, 1], [0, 0], [1, 0], [1, 1]])  # 180-degree bend
SLIGHT_BEND_TEMPLATE = np.array([[-1, .2], [0, 0], [1, .2]])
C_BEND_TEMPLATE = np.array([[1, 1], [0, 0], [1, -1]])
WINDOW_SIZE = 15

class SegmentType(Enum):
    SEGMENT = "tan" # cool but boring color
    FEATURE = "mediumorchid" # unique but vibrant color
    START = "orangered"

    @staticmethod
    def color(segment_type):
        return segment_type.value

# Resample a curve using arc-length parameterization
def resample_curve(points, num_points=20):
    tck, _ = splprep(points.T, s=0, k=2)
    new_u = np.linspace(0, 1, num_points)
    return np.array(splev(new_u, tck)).T

# Compute DTW distance between two curves
def dtw_distance(curve1, curve2):
    distance, _ = fastdtw(curve1, curve2, dist=euclidean)
    return distance

def project_pointcloud_to_plane(points):
    # Project a 3D point cloud onto its best-fit 2D plane
    # Compute PCA to find the best-fit plane
    centroid = np.mean(points, axis=0)
    centered_points = points - centroid
    U, S, Vt = np.linalg.svd(centered_points)

    # Define a 2D coordinate system on the plane
    plane_x = Vt[0]
    plane_y = Vt[1]
    
    # Project onto the plane coordinate system
    projected_2d = np.stack([centered_points @ plane_x, centered_points @ plane_y], axis=1)
    return projected_2d

def normalized_window(window):
    # Normalize window to unit square, but above the origin
    centroid = np.mean(window, axis=0)
    centered_window = window - centroid
    scale = np.max(np.linalg.norm(centered_window, axis=1))
    normalized_window = centered_window / scale
    normalized_window[:, 1] -= np.min(normalized_window[:, 1])
    return normalized_window, centroid, scale

# Match template using DTW similarity
def match_template(pointcloud, template_points, window_size, error_threshold=float('inf'), visualize=False):
    best_match = None
    best_error = float('inf')

    pointcloud_projected = project_pointcloud_to_plane(pointcloud)
    
    # Resample template with arc-length parameterization
    template_curve_points = resample_curve(template_points)

    # Sliding window search
    for i in range(window_size, len(pointcloud) - window_size + 1):
        window = pointcloud_projected[i-window_size:i+window_size]
        window, _, _ = normalized_window(window)
        window_curve_points = window # resample_curve(window)

        # Compute DTW similarity
        error = dtw_distance(template_curve_points, window_curve_points)

        if error < best_error and error < error_threshold:
            best_error = error
            best_match = i

            # Plot best match
            if visualize:
                plt.figure(figsize=(6, 6))
                plt.plot(template_curve_points[:, 0], template_curve_points[:, 1], 'r-', label="Template Curve")
                plt.plot(window_curve_points[:, 0], window_curve_points[:, 1], 'bo-', label="Window Points")
                plt.plot(window[:, 0], window[:, 1], 'go')
                plt.legend()
                plt.title(f"Best Match at index {i} (Error: {best_error:.2f})")
                plt.xlabel("X")
                plt.ylabel("Y")
                plt.show()
    
    return best_match, best_error


def pointcloud_to_segments(points, template, window_size=WINDOW_SIZE, visualize=False):
    """
    Labels the points of an ordered shoelace point cloud into indices for the segment or feature
    """
    labels = [SegmentType.color(SegmentType.SEGMENT) for _ in range(points.shape[0])]

    # Detect Feature
    best_index, best_error = match_template(points, template_points=template, window_size=window_size, error_threshold=1.0, visualize=visualize)
    if best_index is not None:
        for i in range(best_index-window_size, best_index+window_size):
            labels[i] = SegmentType.color(SegmentType.FEATURE)
    labels[0] = SegmentType.color(SegmentType.START)

    return labels

def visualize_segments_3d(points, labels):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=labels)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Ensure all axes have the same scale
    min_vals = np.min(points, axis=0)
    max_vals = np.max(points, axis=0)
    max_range = np.max(max_vals - min_vals) / 2.0

    mid_x = (max_vals[0] + min_vals[0]) / 2.0
    mid_y = (max_vals[1] + min_vals[1]) / 2.0
    mid_z = (max_vals[2] + min_vals[2]) / 2.0

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    plt.show()
    return ax

def visualize_segments_2d(points, labels):
    projected_2d = project_pointcloud_to_plane(points)
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.scatter(projected_2d[:, 0], projected_2d[:, 1], c=labels)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')

    # Ensure all axes have the same scale
    min_vals = np.min(projected_2d, axis=0)
    max_vals = np.max(projected_2d, axis=0)
    max_range = np.max(max_vals - min_vals) / 2.0

    mid_x = (max_vals[0] + min_vals[0]) / 2.0
    mid_y = (max_vals[1] + min_vals[1]) / 2.0

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)

    plt.show()
    return ax

if __name__ == "__main__":
    template = C_BEND_TEMPLATE
    cleaned_data = load_data("../data/data.json")
    cleaned_data = [remove_outliers(points, eps=0.01) for points in cleaned_data]
    cleaned_data = [downsample_points(points, voxel_size=0.005) for points in cleaned_data]
    source_pointcloud, pointclouds = cleaned_data[0], cleaned_data[1:]
    source_pointcloud = initial_pointcloud_order(source_pointcloud, visualize=False)
    segments = pointcloud_to_segments(source_pointcloud, template=template, visualize=False)
    visualize_segments_2d(source_pointcloud, segments)

    for i in range(len(pointclouds)):
        target_pointcloud = pointclouds[i]
        ordered_target_pointcloud = track_state(source_pointcloud, target_pointcloud, visualize=False)
        visualize = i % 5 == 0
        segments = pointcloud_to_segments(ordered_target_pointcloud, template=template, visualize=visualize)
        visualize_segments_2d(ordered_target_pointcloud, segments)
        source_pointcloud = ordered_target_pointcloud
