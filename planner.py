from point_correspondences import *

import numpy as np
from scipy.spatial import KDTree
from enum import Enum

class SegmentType(Enum):
    SEGMENT = 0
    BEND = 1
    LOOP = 2

def segment_shoelace(points, curvature_threshold=0.2, loop_threshold=0.05):
    """
    Segments an ordered shoelace point cloud into sections based on bends and loops.

    Args:
        points (np.ndarray): Ordered Nx3 array of 3D points representing the shoelace.
        curvature_threshold (float): Threshold for detecting bends.
        loop_threshold (float): Distance threshold to detect loops.

    Returns:
        List[List[int]]: List of segments, each containing indices of points in that segment.
    """

    def compute_curvature(pts):
        """ Compute curvature as angle change at each point. """
        vectors = np.diff(pts, axis=0)  # Compute segment vectors
        norms = np.linalg.norm(vectors, axis=1, keepdims=True)
        unit_vectors = vectors / (norms + 1e-8)  # Normalize, avoid division by zero
        dot_products = np.einsum('ij,ij->i', unit_vectors[:-1], unit_vectors[1:])
        angles = np.arccos(np.clip(dot_products, -1.0, 1.0))  # Compute angles
        return np.insert(angles, 0, 0)  # First point has no curvature

    def detect_loops(pts, threshold):
        """ Detect possible loops by looking at prox. """
        tree = KDTree(pts)
        loop_indices = []
        for i, point in enumerate(pts):
            neighbors = tree.query_ball_point(point, threshold)
            for j in neighbors:
                if abs(i-j) > 3:  # Ensure it's a non-trivial loop
                    loop_indices.append(i)
                    break
        return set(loop_indices)

    # Compute curvature and find bend points
    curvatures = compute_curvature(points)
    bend_indices = set(np.where(curvatures > curvature_threshold)[0])

    # Detect loops
    loop_indices = detect_loops(points, loop_threshold)

    # Merge into segments
    segment_indices = sorted(bend_indices.union(loop_indices))
    segments = [] # [(start, end, type)]
    start_idx = 0
    for segment_idx in segment_indices[1:]:
        if segment_idx in bend_indices:
            segments.append((start_idx, segment_idx, SegmentType.SEGMENT)) # because the bend indices just tell you where the bends are..
        elif segment_idx in loop_indices:
            segments.append((start_idx, segment_idx, SegmentType.LOOP))
        else:
            segments.append((start_idx, segment_idx, SegmentType.SEGMENT))
        start_idx = segment_idx

    return segments

def visualize_segments(points, segments):
    labeled_points = {}
    for (start_idx, end_idx, segment_type) in segments:
        for i in range(start_idx, end_idx):
            labeled_points[i] = segment_type
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=labeled_points.values())
    plt.show()
    return ax

if __name__ == "__main__":
    pass