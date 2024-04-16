import time

import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import cv2

def get_clusters(pcd):
    """ takes in a PCD and returns the clusters -rob"""
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
    coordinate_frame.rotate(np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]), center=[0, 0, 0])    # Rotate to match camera frame

    #downsampling 
    voxel_size = 0.001   # Meters
    down_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    print(f'Downsampled point cloud has {len(down_pcd.points)} points.')

    #remove bgrnd, filter
    max_distance = 0.5   # Meters

    np_down_pcd = np.asarray(down_pcd.points)
    within_dist_idx = np.where(np.abs(np_down_pcd[:, 2]) < max_distance)[0]
    filtered_pcd = down_pcd.select_by_index(within_dist_idx)

    #remove outliers
    nb_neighbors = 20
    std_ratio = 2
    filtered_pcd, idx = filtered_pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    print(f'Outlier removed point cloud has {len(filtered_pcd.points)} points.')


    # Use RANSAC to fit a plane and locate the table surface
    plane_model, inliers = filtered_pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f'Plane model: {a}x + {b}y + {c}z + {d} = 0')

    #get inliers and outliers and print
    inlier_pcd = filtered_pcd.select_by_index(inliers)
    inlier_pcd.paint_uniform_color([1, 0, 0])
    outlier_pcd = filtered_pcd.select_by_index(inliers, invert=True)
    print(f'Plane inliers point cloud has {len(inlier_pcd.points)} points.')
    print(f'Plane outliers point cloud has {len(outlier_pcd.points)} points.')

    # Basic segmentation of the outlier point cloud using DBSCAN clustering
    labels = outlier_pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True)
    labels = np.asarray(labels)
    print(f'Found {len(np.unique(labels))} clusters.')

    # returns everything
    return outlier_pcd, labels, coordinate_frame

def get_cluster_n(outlier_pcd, labels, n):
    """ takes in an outlier_pcd from get_clusters and returns the nth cluster """
    cluster_n = outlier_pcd.select_by_index(np.where(labels == n)[0])
    return cluster_n

def pca(cluster):
    """give it a block, get the principle components"""
    # Center the cluster points around the mean
    points = np.asarray(cluster.points)
    mean = np.mean(points, axis=0)
    centered_points = points - mean
    print(f'Mean: {mean}')
    print(f'Cluster shape: {centered_points.shape}')

    cov = np.cov(centered_points.T)
    print(f'Covariance matrix: \n{cov}')
    u, sigma, v_transpose = np.linalg.svd(cov)
    v = v_transpose.T   # Transpose to get the right singular vectors
    print(f'Singular values: {sigma}')
    print(f'Right singular vectors (columns of V): \n{v}')

    cluster_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05, origin=mean)
    cluster_axes.rotate(v, center=mean)

    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.003)
    sphere.translate(mean)

    return mean, cluster_axes

# TESTING THE ABOVE FUNCTIONS  -rob #########################################################
print("taking in PCD")
pcd = o3d.io.read_point_cloud("example_pcd.ply")
outlier_pcd, labels, coordinate_frame =  get_clusters(pcd)
#o3d.visualization.draw_geometries([outlier_pcd, coordinate_frame]) # to view all blocks

#visualize block 1
block_0 = get_cluster_n(outlier_pcd, labels, 0) # 0 is the index of the block we want
mean, cluster_axis = pca(block_0)
mean_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.008)
mean_sphere.translate(mean)
#o3d.visualization.draw_geometries([cluster_axis, block_0, coordinate_frame, mean_sphere])

#visualize block 2
block_1 = get_cluster_n(outlier_pcd, labels, 1) # 0 is the index of the block we want
mean, cluster_axis = pca(block_1)
mean_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.008)
mean_sphere.translate(mean)
o3d.visualization.draw_geometries([cluster_axis, block_1, coordinate_frame, mean_sphere])

# END OF TESTING THE FUNCTIONS #########################################################


# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)

# Start the pipeline
pipeline.start(config)

# Get the device and color sensor
profile = pipeline.get_active_profile()
device = profile.get_device()
color_sensor = device.first_color_sensor()

color_sensor.set_option(rs.option.enable_auto_exposure, 1)
color_sensor.set_option(rs.option.enable_auto_white_balance, 1)

# Wait for the auto exposure and white balance to stabilize
time.sleep(2)

try:
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    if not depth_frame or not color_frame:
        raise RuntimeError("Could not acquire depth or color frames.")

    # Convert images to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Create alignment primitive with color as its target stream
    align = rs.align(rs.stream.color)
    frames = align.process(frames)

    # Update frames after alignment
    aligned_depth_frame = frames.get_depth_frame()

    # Validate that both frames are valid
    if not aligned_depth_frame:
        raise RuntimeError("Could not align depth frame to color frame.")

    # Create pointcloud object and map to color
    pc = rs.pointcloud()
    pc.map_to(color_frame)
    pointcloud = pc.calculate(aligned_depth_frame)

    # Export the point cloud to a PLY file
    pointcloud.export_to_ply("output.ply", color_frame)
    print("Point cloud saved to 'output.ply'.")

    # we have the point cloud now... so we will do this: 
    #pcd = o3d.io.read_point_cloud("output.ply")
    #outlier_pcd, labels, coordinate_frame =  get_clusters(pcd)
    #block_0 = get_cluster_n(outlier_pcd, labels, 0) # 0 is the index of the block we want
    #mean, cluster_axis = pca(block_0)
    #o3d.visualization.draw_geometries([cluster_axis, block_0, coordinate_frame])

finally:
    # Stop pipeline
    pipeline.stop()
