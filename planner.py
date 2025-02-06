from point_correspondences import *

import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import splprep, splev
from sklearn.decomposition import PCA
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
from enum import Enum

TEST_TEMPLATE = np.array([[-1, 0], [0, 0], [0, 1]])  # 90-degree bend
WINDOW_SIZE = 5

class SegmentType(Enum):
    SEGMENT = "tan" # cool but boring color
    FEATURE = "mediumorchid" # unique but vibrant color

    @staticmethod
    def color(segment_type):
        return segment_type.value

# Compute arc-length parameterization
def arc_length_param(points):
    distances = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cumulative_length = np.insert(np.cumsum(distances), 0, 0)
    return cumulative_length / cumulative_length[-1]  # Normalize to [0, 1]

# Resample a curve using arc-length parameterization
def resample_curve(points, num_points=100):
    tck, _ = splprep(points.T, s=0, k=2)
    arc_lens = arc_length_param(points)
    new_u = np.linspace(0, 1, num_points)
    return np.array(splev(new_u, tck)).T

# Compute DTW distance between two curves
def dtw_distance(curve1, curve2):
    distance, _ = fastdtw(curve1, curve2, dist=euclidean)
    return distance

# Match template using DTW similarity
def match_template(pointcloud, template_points, window_size, error_threshold=float('inf'), visualize=False):
    best_match = None
    best_error = float('inf')

    # Resample template with arc-length parameterization
    template_curve_points = resample_curve(template_points)

    # Sliding window search
    for i in range(len(pointcloud) - window_size + 1):
        window = pointcloud[i:i+window_size]
        window, _, _ = normalize_pointcloud(window)

        pca = PCA(n_components=2)
        projected_pts = pca.fit_transform(window)
        # projected_pts = window
        window = projected_pts[:, :2]  # Main direction

        window_curve_points = resample_curve(window)

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
                plt.plot(window[:, 0], window[:, 1], 'g-')
                plt.legend()
                plt.title(f"Best Match at index {i} (Error: {best_error:.2f})")
                plt.xlabel("X")
                plt.ylabel("Y")
                plt.show()
    
    return best_match, best_error


def pointcloud_to_segments(points, template=TEST_TEMPLATE, window_size=WINDOW_SIZE, visualize=False):
    """
    Labels the points of an ordered shoelace point cloud into indices for the segment or feature
    """
    labels = [SegmentType.color(SegmentType.SEGMENT) for _ in range(points.shape[0])]

    # Detect Feature
    best_index, best_error = match_template(points, template_points=template, window_size=window_size, visualize=visualize)
    if best_index is not None:
        for i in range(best_index-window_size, best_index+window_size):
            labels[i] = SegmentType.color(SegmentType.FEATURE)

    return labels

def visualize_segments(points, labels):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=labels)

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

if __name__ == "__main__":
    data = load_data("data/data2.npz")
    quad_mask = create_quad_mask(data['mask'][0].shape, corners=CORNERS)
    cleaned_data = {'points': []}
    cleaned_data['initial_points'] = clean_data(data['mask'][0], data['points'][0], quad_mask)
    for i in range(1, len(data['points'])):
        cleaned_data['points'].append(clean_data(data['mask'][i], data['points'][i], quad_mask))
    source_pointcloud = cleaned_data['initial_points']
    source_pointcloud = initial_pointcloud_order(source_pointcloud)
    segments = pointcloud_to_segments(source_pointcloud, visualize=False)
    visualize_segments(source_pointcloud, segments)

    for i in range(len(cleaned_data['points'])):
        target_pointcloud = cleaned_data['points'][i]
        ordered_target_pointcloud = track_state(source_pointcloud, target_pointcloud, visualize=False)
        if i < 5:
            visualize = False
        else:
            visualize = True
        segments = pointcloud_to_segments(ordered_target_pointcloud, visualize=visualize)
        visualize_segments(ordered_target_pointcloud, segments)
        source_pointcloud = ordered_target_pointcloud