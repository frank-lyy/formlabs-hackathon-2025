from image_loader import *
from ..point_correspondences import *

def get_pointcloud_from_image(filename, width, height, remove_sides=False):
    depth_image = get_depth(filename+"_Depth.raw", width, height)
    mask = get_mask(filename+"_Color.png", width, height, remove_sides=remove_sides)
    
    pointcloud = []
    for i in range(depth_image.shape[0]):
        for j in range(depth_image.shape[1]):
            if mask[i, j]:
                pointcloud.append([i, j, depth_image[i, j]])
                
    return np.array(pointcloud)

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