from point_correspondences import *

import numpy as np
from scipy.spatial import KDTree
from scipy.signal import savgol_filter
from sklearn.decomposition import PCA
from scipy.optimize import curve_fit
from enum import Enum

TEST_TEMPLATE = np.array([[-2, 4], [-1, 1], [0, 0], [0, 0], [1, 1], [2, 4]])
WINDOW_SIZE = 3

class SegmentType(Enum):
    SEGMENT = "tan" # cool but boring color
    FEATURE = "mediumorchid" # unique but vibrant color

    @staticmethod
    def color(segment_type):
        return segment_type.value

def template_curve(x, a, b, c):
    """ Quadratic function to fit the template shape. """
    return a * x**2 + b * x + c

# def pca_align(points):
#     """ Aligns points using PCA and returns transformed points + rotation matrix. """
#     mean = np.mean(points, axis=0)
#     centered = points - mean
#     U, S, Vt = np.linalg.svd(centered)
#     return centered @ Vt.T, Vt  # Rotate to align principal axes

def embed_template_3d(template_points):
    """ Embeds a 2D template into 3D space by assuming it lies in the XY plane. """
    return np.hstack((template_points, np.zeros((template_points.shape[0], 1))))

# def project_to_plane(points, plane_origin, plane_normal):
#     """ Projects points onto a plane defined by an origin and normal vector. """
#     normal = plane_normal / np.linalg.norm(plane_normal)
#     vectors = points - plane_origin
#     distances = np.dot(vectors, normal)
#     projected = points - np.outer(distances, normal)
#     return projected

def find_optimal_rotation(scatter_points, curve_points):
    # Center the points
    # scatter_centered = scatter_points - np.mean(scatter_points, axis=0)
    # curve_centered = curve_points - np.mean(curve_points, axis=0)
    
    # Compute cross-covariance matrix
    H = np.dot(scatter_points.T, curve_points)
    
    # Perform SVD
    U, S, Vt = np.linalg.svd(H)
    
    # Compute rotation matrix
    R = np.dot(Vt.T, U.T)
    
    # Ensure proper rotation (det(R) = 1)
    if np.linalg.det(R) < 0:
        Vt[-1,:] *= -1
        R = np.dot(Vt.T, U.T)
    
    return R

def match_template(pointcloud, template, window_size, error_threshold=float('inf'), visualize=False):
    best_match = None
    best_error = float('inf')

    # Embed 2D template into 3D
    template_3d = embed_template_3d(template)
    template_3d_normalized, _, _ = normalize_pointcloud(template_3d)
    x_template = template_3d_normalized[:, 0]
    y_template = template_3d_normalized[:, 1]
    params, _ = curve_fit(template_curve, x_template, y_template)
    print(params)

    # Sliding window search
    for i in range(window_size, len(pointcloud) - window_size + 1):
        window = pointcloud[i-window_size:i+window_size]
        window_normalized, _, _ = normalize_pointcloud(window)

        window_rotation = find_optimal_rotation(window_normalized, template_3d_normalized)
        rotated_window = window_normalized @ window_rotation.T

        x_window = rotated_window[:, 0]
        y_window = rotated_window[:, 1]
        # Fit the template curve to the window
        y_fitted = template_curve(x_window, *params)
        error = np.sum((y_window - y_fitted) ** 2)

        if error < best_error and error < error_threshold:
            best_error = error
            best_match = i
            
            if visualize:
                # Plot best match
                fig = plt.figure(figsize=(6, 6))
                ax = fig.add_subplot(111, projection='3d')
                ax.plot(x_window, y_window, 'bo-', label="Window Points")
                ax.plot(x_window, y_fitted, 'r-', label="Fitted Curve")
                ax.legend()
                ax.set_title(f"Best Match at index {i} (Error: {best_error:.2f})")
                ax.set_xlabel("X")
                ax.set_ylabel("Y")
                ax.set_zlabel("Z")
                plt.show()
    
    return best_match, best_error


def pointcloud_to_segments(points, template=TEST_TEMPLATE, window_size=WINDOW_SIZE, visualize=False):
    """
    Labels the points of an ordered shoelace point cloud into indices for the segment or feature
    """
    labels = [SegmentType.color(SegmentType.SEGMENT) for _ in range(points.shape[0])]

    # Detect Feature
    best_index, best_error = match_template(points, template=template, window_size=window_size, visualize=visualize)
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