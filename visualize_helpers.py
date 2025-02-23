import numpy as np

def normalize_plot(points, ax, three_d=True):
    ax.set_xlabel('X')
    ax.set_ylabel('Y')

    # Ensure all axes have the same scale
    min_vals = np.min(points, axis=0)
    max_vals = np.max(points, axis=0)
    max_range = np.max(max_vals - min_vals) / 2.0

    mid_x = (max_vals[0] + min_vals[0]) / 2.0
    mid_y = (max_vals[1] + min_vals[1]) / 2.0

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    
    if three_d:
        ax.set_zlabel('Z')
        mid_z = (max_vals[2] + min_vals[2]) / 2.0
        ax.set_zlim(mid_z - max_range, mid_z + max_range)
    
    return ax