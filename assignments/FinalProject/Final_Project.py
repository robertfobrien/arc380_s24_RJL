import time

import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import cv2
from cv2 import aruco

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

def get_all_blocks(pcd):
    outlier_pcd, labels, coordinate_frame =  get_clusters(pcd)
    blocks = []
    means = []
    cluster_axes_arr = []
    mean_spheres = []

    for i in range(len(labels)):
        #print("i:", i)
        block = get_cluster_n(outlier_pcd, labels, i)
        #print("block", block)

        # make sure our block is large enough
        if len(block.points) < 50:
            continue
        
        blocks.append(block)
        mean, cluster_axis = pca(block)
        #print("mean ", mean)
        #print("cluster_axis", cluster_axis)
        means.append(mean)
        cluster_axes_arr.append(cluster_axis)
        mean_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.008)
        mean_sphere.translate(mean)
        mean_spheres.append(mean_sphere)

    print("Found ", len(blocks), " Blocks!")
    print("    ", blocks)

    # used for visualization purposes    
    all_geometries = [coordinate_frame, coordinate_frame]
    for i in range(len(blocks)):
        all_geometries.append(cluster_axes_arr[i])
        all_geometries.append(blocks[i])
        all_geometries.append(mean_spheres[i])
    return blocks, means, cluster_axes_arr, mean_spheres, all_geometries

def visualize_shapes(path="example_pcd.ply"):
    print("Intaking point cloud: ")
    pcd = o3d.io.read_point_cloud("example_pcd.ply")
    blocks, means, block_axes_arr, mean_spheres, all_geometries = get_all_blocks(pcd)
    o3d.visualization.draw_geometries(all_geometries)



# Reduces shadows on white background and brightens the color of acrylic pieces 
# for better k-means clustering. Can be fine-tuned for better performance.
def process_for_kmeans(img_file):
    img = cv2.imread(img_file)
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    h = img_hsv[:,:,0] # Hue
    s = img_hsv[:,:,1] # Saturation
    v = img_hsv[:,:,2] # Value

    for i in range(len(v)):
        for j in range(len(v[i])):

            if s[i][j] < 150 and v[i][j] < 50:
                # Leave black as is (for ArUco markers)
                pass
            elif s[i][j] < 50 and v[i][j] > 50:
                # Minimize shadows on white background
                s[i][j] = 0
                v[i][j] = 255
            else:
                # Lighten/brighten the pixel
                v[i][j] = min(v[i][j] + 100, 255)

    merged = cv2.merge([h, s, v])
    processed = cv2.cvtColor(merged, cv2.COLOR_HSV2BGR)

    return processed

def get_mm_per_pixel(pixel_width, pixel_height):
    # get (mm)/(#pixel) 
    actual_width = 480 #mm 
    actual_height = 300 #mm 
    # average them 
    a = actual_width/pixel_width
    b = actual_height/pixel_height
    return (a+b)/(2.0)

def get_shapes_info(img_path):
    # Load an image from a file
    img = cv2.imread(img_path)
    # Convert the image from BGR to RGB and display using matplotlib
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # Load the predefined dictionary where our markers are printed from
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

    # Load the default detector parameters
    detector_params = aruco.DetectorParameters()

    # Create an ArucoDetector using the dictionary and detector parameters
    detector = aruco.ArucoDetector(dictionary, detector_params)
    corners, ids, rejected = detector.detectMarkers(img)
    markers_img = img_rgb.copy()
    aruco.drawDetectedMarkers(markers_img, corners, ids)

    # show the ardutags on the screen 
    #plt.figure(figsize=(16,9))
    #plt.imshow(markers_img)
    #plt.title('Detected ArUco markers')
    #plt.show()

    # Define the dimensions of the output image
    width = 10      # inches
    height = 7.5    # inches
    ppi = 45        # pixels per inch (standard resolution for most screens - can be any arbitrary value that still preserves information)

    _ids = ids.flatten()
    #print(_ids)
    #print(np.argsort(_ids))

    # Sort corners based on id
    ids = ids.flatten()
    #print(ids)

    # Sort the corners based on the ids
    corners = np.array([corners[i] for i in np.argsort(ids)])
    #print(corners.shape)

    # Remove dimensions of size 1
    corners = np.squeeze(corners)
    #print(corners)

    # Sort the ids
    ids = np.sort(ids)

    # Extract source points corresponding to the exterior bounding box corners of the 4 markers
    #src_pts = np.array([corners[0][0], corners[1][1], corners[2][2], corners[3][3]], dtype='float32' # this is the original (with all the)
    src_pts = np.array([corners[0][1], corners[1][2], corners[2][3], corners[3][0]], dtype='float32')
    #print("src points")
    #print(src_pts)

    # Compute the axis-aligned bounding box of the points
    x, y, w, h = cv2.boundingRect(src_pts)

    mm_per_pixel = get_mm_per_pixel(w,h)


    # Crop the image using the computed bounding box
    cropped_image = img.copy()[y:y+h, x:x+w]

    # display the cropped image to the id tags
    #plt.imshow(cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB))
    #plt.title('cropped image')
    #plt.show()

    # Run k-means clustering on the image

    # Reshape our image data to a flattened list of RGB values
    img_data = cropped_image.reshape((-1, 3))
    img_data = np.float32(img_data)

    # Define the number of clusters
    k = 8

    # Define the criteria for the k-means algorithm
    # This is a tuple with three elements: (type of termination criteria, maximum number of iterations, epsilon/required accuracy)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)

    # Run the k-means algorithm
    # Parameters: data, number of clusters, best labels, criteria, number of attempts, initial centers
    _, labels, centers = cv2.kmeans(img_data, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)

    #print(f'Labels shape: {labels.shape}')
    #print(f'Centers shape: {centers.shape}')
    #print(f'Centers: \n{centers}')

    # The output of the k-means algorithm gives the centers as floating point values
    # We need to convert these back to uint8 to be able to use them as pixel values
    centers = np.uint8(centers)

    # Rebuild the image using the labels and centers
    kmeans_data = centers[labels.flatten()]
    kmeans_img = kmeans_data.reshape(cropped_image.shape)
    labels = labels.reshape(cropped_image.shape[:2])

    output_image = kmeans_img.copy()
    k_means_copy = kmeans_img.copy()

    # shape dictionary we will use to store all the info about shapes
    shape_dict = []
    """
    Color
    Shape (i.e., circle or square)
    Size
    Position (in the world frame)
    Orientation (if it is a square)
    """
    num_circles = 0

    # Loop through each cluster
    for i in range(k):
        # Create a mask for the current cluster
        mask = (labels == i).astype(np.uint8) * 255

        # find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours: # this will run for each shape 
            object_info = {
                "color": None,
                "shape": None,
                "size": None,  
                "position": None,
                "orientation": None 
            }
            # make sure we're above our threshold
            if cv2.contourArea(cnt) > 340 and cv2.contourArea(cnt) <  140000:
                #print("area, " + str(cv2.contourArea(cnt)))

                # draws the outer contour
                #cv2.drawContours(output_image, [cnt], -1, (0, 255, 0), 2)

                # center
                M = cv2.moments(cnt)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    area = cv2.contourArea(cnt)
                    perimeter = cv2.arcLength(cnt, closed=True)

                    #determining shape type
                    roundness = (4 * np.pi * area) / perimeter**2
                    #print("roundness, ", roundness)

                    # determines if a shape is a circle or square, the only "good" ones
                    is_a_good_shape = False 


                    if roundness > 0.85: # testing for circle
                        object_info['shape'] = "circle"
                        num_circles = num_circles + 1
                        is_a_good_shape = True
                    elif np.abs(area - (perimeter/4)**2) < 900: # testing for square --> making sure (0.25*perim)^2 is close to the area of the object
                        object_info['shape'] = "square"
                        #print("area from perimeter: ", str((perimeter/4)**2))
                        #print("actual area: ", area)
                        is_a_good_shape = True
                        object_info['orientation'] = cv2.minAreaRect(cnt)[2]
                        #print(object_info['orientation'])

                    if np.abs(area - (perimeter/4)**2) < 1500:
                        print("squareness: :", np.abs(area - (perimeter/4)**2))
                        print("seen as a: ",  object_info['shape'])
                    
                    
                    #some attributes that we're adding about this particular shape
                    object_info['color'] = str(i) # adds the kmeans id as the 'color'
                    object_info['position'] = [cx*mm_per_pixel,cy*mm_per_pixel]
                    object_info['size'] = area

                    
                    #add this shape to the full array that tells us about all our shapes, but only if it's "good"
                    if is_a_good_shape:
                        shape_dict.append(object_info)
                        # center of cluster
                        cv2.circle(output_image, (cx, cy), 5, (0, 0, 255), -1)
                        # put id on cluster
                        cv2.putText(output_image, str(i), (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)


                        # For squares, draw orientation axes
                        if object_info['shape'] == 'square':
                            center = (cx, cy)       # Center of the blue square (in pixels)
                            angle_rad = np.deg2rad(int(object_info['orientation']))    # Orientation angle of the square (in radians)
                            length = 50         # The length of the frame axes

                            # Calculate the end points of each axis
                            # X-axis (you could change color later if needed)
                            end_x = (int(center[0] + length * np.cos(angle_rad)), int(center[1] + length * np.sin(angle_rad)))
                            # Y-axis (rotate the angle by 90 degrees or pi/2 radians for the perpendicular)
                            end_y = (int(center[0] + length * np.cos(angle_rad + np.pi/2)), int(center[1] + length * np.sin(angle_rad + np.pi/2)))

                            # Draw the axes
                            cv2.line(output_image, center, end_x, (0, 0, 255), 2)
                            cv2.line(output_image, center, end_y, (0, 255, 0), 2)
                
    cv2.imshow('Clusters with Centers (Excluding Small Clusters)', output_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return shape_dict

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
o3d.visualization.draw_geometries([cluster_axis, block_0, coordinate_frame, mean_sphere])

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
