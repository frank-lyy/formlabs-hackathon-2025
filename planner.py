from point_correspondences import *

import numpy as np
from scipy.spatial import KDTree
from scipy.signal import savgol_filter
from sklearn.decomposition import PCA
from enum import Enum

class SegmentType(Enum):
    SEGMENT = "red"
    BEND = "blue"
    LOOP = "green"
    END = "purple"

    @staticmethod
    def color(segment_type):
        return segment_type.value

def segment_shoelace(points, curvature_threshold=2, loop_threshold=0.005):
    """
    Labels the points of an ordered shoelace point cloud into segments, bends, and loops.
    """
    normalized_points, _, _ = normalize_pointcloud(points)
    tree = KDTree(normalized_points)
    
    def detect_bends(pts, tree, k=10, smooth_window=7, poly_order=2):
        """ Detects bends by computing curvature with a polynomial fit. """
        curvature = np.zeros(len(pts))

        for i in range(len(pts)):
            neighbor_indices = range(max(0, i-k), min(len(pts), i+k+1))
            if len(neighbor_indices) < 2*k:
                continue
            local_pts = pts[neighbor_indices]

            # Project to 2D using PCA
            pca = PCA(n_components=2)
            projected_pts = pca.fit_transform(local_pts)
            x, y = projected_pts[:, 0], projected_pts[:, 1]  # Main direction

            x = (x - np.mean(x)) / (np.std(x) + 1e-8)
            y = (y - np.mean(y)) / (np.std(y) + 1e-8)

            # Fit quadratic y = ax^2 + bx + c
            coeffs = np.polyfit(x, y, 2)
            a = coeffs[0]

            # Curvature approximation
            curvature[i] = 2 * abs(a)  # Higher a = more curvature

        # Smooth curvature values using Savitzky-Golay filter
        # curvature = savgol_filter(curvature, window_length=smooth_window, polyorder=poly_order, mode='interp')
        print(np.max(curvature), np.min(curvature))
        bend_indices = set(np.where(np.abs(curvature) > curvature_threshold)[0])
        return bend_indices

    def detect_loops(pts, tree, loop_threshold=loop_threshold):
        """ Detect possible loops by looking at proximity """
        loop_indices = []
        for i, point in enumerate(pts):
            neighbors = tree.query_ball_point(point, r=loop_threshold)
            for j in neighbors:
                if j < i:
                    continue # skip out on previous points
                elif j-i > 20:  # Ensure it's a non-trivial loop
                    loop_indices.append((i, j))
                    i = j  # Skip ahead to the next point
                    break
        return loop_indices

    labels = [SegmentType.color(SegmentType.SEGMENT) for _ in range(points.shape[0])]

    # Detect bends
    bend_indices = detect_bends(normalized_points, tree)
    for bend_idx in bend_indices:
        labels[bend_idx] = SegmentType.color(SegmentType.BEND)

    # Detect loops
    loop_indices = detect_loops(normalized_points, tree)
    for loop_start, loop_end in loop_indices:
        for i in range(loop_start, loop_end):
            labels[i] = SegmentType.color(SegmentType.LOOP)

    labels[-1] = SegmentType.color(SegmentType.END)

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
    cleaned_data = clean_data(data)
    source_pointcloud = cleaned_data['initial_points']
    source_pointcloud = initial_pointcloud_order(source_pointcloud)
    segments = segment_shoelace(source_pointcloud)
    visualize_segments(source_pointcloud, segments)

    for i in range(len(cleaned_data['points'])):
        target_pointcloud = cleaned_data['points'][i]
        ordered_target_pointcloud = track_state(source_pointcloud, target_pointcloud, visualize=False)
        segments = segment_shoelace(ordered_target_pointcloud)
        visualize_segments(ordered_target_pointcloud, segments)
        source_pointcloud = ordered_target_pointcloud