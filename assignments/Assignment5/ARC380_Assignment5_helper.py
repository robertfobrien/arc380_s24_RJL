import time

import pyrealsense2 as rs
import numpy as np
import cv2
from matplotlib import pyplot as plt
import compas.geometry as cg
import compas_rrc as rrc
from cv2 import aruco



def capture_img(visualize: bool = False, save: bool = False, path: str = 'img.png') -> np.ndarray:
    # Create a pipeline
    pipeline = rs.pipeline()

    devices = rs.context().devices
    
    # Find the T265
    camera_name = None
    devices = rs.context().devices


    print("pipeline", pipeline)

    # Create a config and configure the pipeline to stream
    config = rs.config()
    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

    # Start streaming
    print("pipeline before start, stream is enabled")
    pipeline.start(config)
    print("pipeline started")

    # Get the device and color sensor
    profile = pipeline.get_active_profile()
    device = profile.get_device()
    color_sensor = device.first_color_sensor()

    color_sensor.set_option(rs.option.enable_auto_exposure, 1)
    color_sensor.set_option(rs.option.enable_auto_white_balance, 1)

    # Wait for the auto exposure and white balance to stabilize
    time.sleep(2)

    try:
        # Wait for a coherent color frame
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            print("No color frame captured.")
        else:
            # Convert image to numpy array
            color_image = np.asanyarray(color_frame.get_data())

            if save:
                cv2.imwrite(path, color_image)

            if visualize:
                # Display the image
                plt.imshow(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
                plt.axis('off')  # Turn off axis numbers and ticks
                plt.show()
    finally:
        # Stop streaming
        pipeline.stop()

    return color_image


def create_frame_from_points(point1: cg.Point, point2: cg.Point, point3: cg.Point) -> cg.Frame:
    """Create a frame from three points.

    Args:
        point1 (cg.Point): The first point (origin).
        point2 (cg.Point): The second point (along x axis).
        point3 (cg.Point): The third point (within xy-plane).

    Returns:
        cg.Frame: The frame that fits the given 3 points.
    """
    frame = None
    # ================================== YOUR CODE HERE ==================================

    # Part 1.c.
    frame = cg.Frame.from_points(point1, point2, point3)
    print(frame.point.z)
    # ====================================================================================
    return frame


def transform_task_to_world_frame(ee_frame_t: cg.Frame, task_frame: cg.Frame) -> cg.Frame:
    """Transform a task frame to the world frame.

    Args:
        ee_frame_t (cg.Frame): The end-effector frame defined in task space.
        task_frame (cg.Frame): The task frame.

    Returns:
        cg.Frame: The task frame in the world frame.
    """
    ee_frame_w = None
    # ================================== YOUR CODE HERE ==================================

    world_frame = cg.Frame.worldXY()
    T = cg.Transformation.from_frame_to_frame(world_frame, task_frame) 
    ee_frame_t.transform(T)
    ee_frame_w = ee_frame_t
    # ====================================================================================
    return ee_frame_w


def goto_task_point(task_frame, x, y, abb_rrc ):
    """Goes to the point x, y on the task frame (on the paper). x and y are in millimeters"""
    f1 = cg.Frame([x,y,0], [1,0,0], [0,1,0]) #[1,0,0] and [0,1,0] define the x and y planes
    f1_p = transform_task_to_world_frame(f1, task_frame)
    #print("f1_p:", f1_p)
    next = abb_rrc.send_and_wait(rrc.MoveToFrame(f1_p, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))


def goto_robot_home():
    """Go to home position (linear joint move)"""
    home = rrc.RobotJoints([0, 0, 0, 0, 90, 0])
    done = abb_rrc.send_and_wait(rrc.MoveToJoints(home, [], speed, rrc.Zone.FINE))


def goto_task_f_origin(task_frame):
    """Go to the origin of the taskframe"""
    done = abb_rrc.send_and_wait(rrc.MoveToFrame(task_frame, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))


def execute_tf_points(task_frame, points, abb_rrc):
    """Executes a series of points in the task frame"""
    for p in points:
        goto_task_point(task_frame, p[0], p[1], abb_rrc)


if __name__ == '__main__':
    # Make sure markers are in correct order for aruco vs plane formation
    """
    aruco_1 = cg.Point(271.02,483.59, 30)
    aruco_2 = cg.Point(257.31,335.69,28)
    aruco_3 = cg.Point(69.25, 485.12, 30)
    aruco_4 = cg.Point(87.88,339.13,30)
    
    task_frame = create_frame_from_points(aruco_1, aruco_2, aruco_3)

    # Create Ros Client
    ros = rrc.RosClient()
    ros.run()

    # Create ABB Client
    abb_rrc = rrc.AbbClient(ros, '/rob1-rw6')
    print('Connected.')

    # End of Code
    print('Finished')

    # Close client
    ros.close()
    ros.terminate()
    """

    #code to test the image proccessing

    # Load an image from a file
    img_path = 'test_frame.jpeg'
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
    src_pts = np.array([corners[0][0], corners[1][1], corners[2][2], corners[3][3]], dtype='float32')
    #print("src points")
    #print(src_pts)

    # Compute the axis-aligned bounding box of the points
    x, y, w, h = cv2.boundingRect(src_pts)

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
    k = 11

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

    # Display the k-means image
    #plt.imshow(cv2.cvtColor(kmeans_img, cv2.COLOR_BGR2RGB))
    #plt.title(f'Image classification using k-means clustering (k = {k})')
    #plt.gca().invert_yaxis()
    #plt.show()

    # Identify the cluster that is closest to the dark green color
    dark_red = np.array([40, 9, 3])
    distances = np.linalg.norm(centers - dark_red, axis=1)
    red_cluster_label = np.argmin(distances)

    print(f'Label of the cluster closest to dark red: {red_cluster_label}')

    # Create a mask image for this label
    # All pixels that belong to this cluster will be white, and all others will be black
    mask_img = np.zeros(kmeans_img.shape[:2], dtype='uint8')
    mask_img[labels == red_cluster_label] = 255

    

    plt.imshow(mask_img, cmap='gray')
    plt.title(f'Mask image for cluster {red_cluster_label} corresponding to dark red')
    plt.gca().invert_yaxis()
    plt.show()


