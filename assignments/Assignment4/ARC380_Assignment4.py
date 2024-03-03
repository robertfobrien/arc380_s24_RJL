import numpy as np
import compas.geometry as cg
import compas_rrc as rrc

# Define any additional imports here if needed

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


# ====================== Drawing effects and helper functions ============================

# Part 2

# Just made this one after class.
def goto_task_point(task_frame, x, y):
    """Goes to the point x, y on the task frame (on the paper). x and y are in millimeters"""
    f1 = cg.Frame([x,y,0], [1,0,0], [0,1,0])
    f1_p = transform_task_to_world_frame(f1, task_frame)
    print("f1_p:", f1_p)
    next = abb_rrc.send_and_wait(rrc.MoveToFrame(f1_p, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))

def goto_robot_home():
    """Go to home position (linear joint move)"""
    home = rrc.RobotJoints([0, 0, 0, 0, 90, 0])
    done = abb_rrc.send_and_wait(rrc.MoveToJoints(home, [], speed, rrc.Zone.FINE))

def goto_task_f_origin(task_frame):
    done = abb_rrc.send_and_wait(rrc.MoveToFrame(task_frame, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))


# ========================================================================================


if __name__ == '__main__':

    # points in the world frame of parts of the paper
    # adjust z if we need to...
    task_point_in_wf_1 = cg.Point(271.02,483.59, 40) 
    task_point_in_wf_2 = cg.Point(257.31,335.69,40) 
    task_point_in_wf_3 = cg.Point(69.25, 485.12, 40) 
    task_point_in_wf_4 = cg.Point(87.88,339.13,40)
    

    task_frame = create_frame_from_points(task_point_in_wf_1, task_point_in_wf_2, task_point_in_wf_3)
    print("task frame: ", task_frame)

    # Create Ros Client
    ros = rrc.RosClient()
    ros.run()

    # Create ABB Client
    abb_rrc = rrc.AbbClient(ros, '/rob1-rw6')
    print('Connected.')

    # ================================== YOUR CODE HERE ==================================

    # Parts 1.e. and 2
    # set the speed
    speed = 20

    # Move robot to the task frame origin
    goto_task_point(task_frame, 0 , 0 )
    
    #moves to point 0mm, 50mm
    goto_task_point(task_frame, 0 , 50 )

    # ====================================================================================

    # End of Code
    print('Finished')

    # Close client
    ros.close()
    ros.terminate()
