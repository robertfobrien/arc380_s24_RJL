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

    # Create a config and configure the pipeline to stream
    config = rs.config()
    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

    # Start streaming
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
    aruco_1 = cg.Point(271.02,483.59, 30)
    aruco_2 = cg.Point(257.31,335.69,28)
    aruco_3 = cg.Point(69.25, 485.12, 30)
    aruco_4 = cg.Point(87.88,339.13,30)
    
    task_frame = create_frame_from_points(aruco_1, aruco_2, aruco_3)
    #task_frame = create_frame_from_points(aruco_1, aruco_2, aruco_3, aruco_4)
    #print("task frame: ", task_frame)

    #varying_thickness(task_frame,0,0,1,1,1.2)

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