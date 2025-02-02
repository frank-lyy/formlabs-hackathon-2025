import numpy as np
import matplotlib.pyplot as plt

def order_points_by_angle(pointcloud, normed_pointcloud):
    # In the real setup, we will sort by y- or x- coordinate
    # For this example image, we can use angle about the origin since the loop is arranged in a rough circle
    angle_offset = np.pi * .54
    arctan_with_offset = np.arctan2(normed_pointcloud[:, 1], normed_pointcloud[:, 0]) + angle_offset
    # print(np.max(arctan_with_offset), np.min(arctan_with_offset))
    arctan_with_offset = np.mod(arctan_with_offset + np.pi, 2 * np.pi) - np.pi
    # print(np.max(arctan_with_offset), np.min(arctan_with_offset))
    new_pointcloud = pointcloud[np.argsort(arctan_with_offset)]
    colors = np.interp(arctan_with_offset, [-np.pi, np.pi], [0, 1])
    return new_pointcloud, colors

def order_points_by_y(pointcloud, normed_pointcloud):
    new_pointcloud = pointcloud[np.argsort(normed_pointcloud[:, 1])]
    colors = np.interp(normed_pointcloud[:, 1], [np.min(normed_pointcloud[:, 1]), np.max(normed_pointcloud[:, 1])], [0, 1])
    return new_pointcloud, colors