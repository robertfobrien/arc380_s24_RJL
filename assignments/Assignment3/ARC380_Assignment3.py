from __future__ import annotations
from typing import List, Tuple

import time
import math
import numpy as np
import roslibpy
import roslibpy.actionlib

from numpy.testing import assert_almost_equal

# ========================================================================================
# ====================================== PART 1 ==========================================
# ========================================================================================


def quaternion_to_rotation_matrix(quaternion: np.ndarray) -> np.ndarray:
    """
    Convert a quaternion to a 3x3 rotation matrix
    :param quaternion: Quaternion as a NumPy array, in the format (x, y, z, w)
    :return: 3x3 rotation matrix as a NumPy array
    """
    matrix = None

    # ================================== YOUR CODE HERE ==================================



    # ====================================================================================

    return matrix


def rotation_matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    """
    Convert a 3x3 rotation matrix to a quaternion
    :param matrix: 3x3 rotation matrix as a NumPy array
    :return: Quaternion as a NumPy array, in the format (x, y, z, w)
    """
    quaternion = None

    # ================================== YOUR CODE HERE ==================================



    # ====================================================================================

    return quaternion


# ========================================================================================
# ====================================== PART 2 ==========================================
# ========================================================================================


# ================================= HELPER FUNCTIONS =====================================

# Define any helper functions you need here



# ========================================================================================


def get_block_positions(client: roslibpy.Ros) -> List[np.ndarray]:
    """
    Get the positions of the blocks in the Gazebo simulation.
    :param client: ROS client
    :return: List of block positions, each as a NumPy array of shape (3,)
    """

    block_positions = None

    def message_callback(message):
        nonlocal block_positions
        block_positions = []

        # =============================== YOUR CODE HERE =================================



        # ================================================================================

        # Unsubscribe from the topic after receiving a message, so we only receive one
        subscriber.unsubscribe()

    subscriber = roslibpy.Topic(client, '/gazebo/model_states', 'gazebo_msgs/ModelStates')
    subscriber.subscribe(message_callback)

    # Wait for the message to be received
    while block_positions is None:
        pass

    return block_positions


def get_robot_pose(client: roslibpy.Ros) -> Tuple[np.ndarray, np.ndarray]:
    """
    Get the position and orientation of the robot in the Gazebo simulation.
    :param client: ROS client
    :return: Tuple of the robot position and orientation (in quaternions)
             as NumPy arrays of shape (3,) and (4,)
    """

    robot_position = None
    robot_orientation = None

    # ================================== YOUR CODE HERE ==================================



    # ====================================================================================

    return robot_position, robot_orientation


def coordinate_xform_world_to_robot_frame(client: roslibpy.Ros, p_world: np.ndarray) -> np.ndarray:
    """
    Transform a point from the world frame to the robot frame.
    :param client: ROS client
    :param p_world: Point in the world frame as a NumPy array of shape (3,)
    :return: Point in the robot frame as a NumPy array of shape (3,)
    """

    p_robot = None

    robot_position, robot_orientation = get_robot_pose(client)

    # ================================== YOUR CODE HERE ==================================



    # ====================================================================================

    return p_robot


def form_cartesian_path_msg(waypoints: List[np.ndarray]) -> dict:
    """
    Form a message for a Cartesian path from a list of waypoints.
    :param waypoints: List of waypoints, each as a NumPy array of shape (3,)
    :return: Message for a Cartesian path as a dictionary
    """

    msg = {
        'header': {
            'stamp': {
                'secs': 0,
                'nsecs': 0
            },
            'frame_id': 'base_link'
        },
        'start_state': {
            'joint_state': {
                'name': ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
                'position': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            }
        },
        'group_name': 'manipulator',
        'waypoints': [],
        'max_step': 0.01,
        'jump_threshold': 0.0,
        'avoid_collisions': False
    }

    msg['waypoints'] = []

    # ================================== YOUR CODE HERE ==================================



    # ====================================================================================

    return msg


def move_robot_to_waypoints(client: roslibpy.Ros, waypoints: List[np.ndarray]) -> None:
    """
    Move the robot to a list of waypoints using a Cartesian path.
    :param client: ROS client
    :param waypoints: List of waypoints, each as a NumPy array of shape (3,)
    :return: None
    """

    service = roslibpy.Service(client, '/compute_cartesian_path', 'moveit_msgs/GetCartesianPath')
    request = roslibpy.ServiceRequest(form_cartesian_path_msg(waypoints))

    print('Calling service...')
    trajectory = service.call(request)['solution']['joint_trajectory']

    print('Solution received')

    action_client = roslibpy.actionlib.ActionClient(client, '/joint_trajectory_action', 'control_msgs/FollowJointTrajectoryAction')
    goal = roslibpy.actionlib.Goal(action_client,
                                   {'trajectory': trajectory,
                                    'goal_time_tolerance': 0.0})
    goal.send()
    result = goal.wait(10)
    action_client.dispose()


def knock_over_blocks() -> None:
    """
    Knock over the blocks using the robot arm
    :return: None
    """
    client = roslibpy.Ros(host='localhost', port=9090)
    client.on_ready(lambda: print('Connected to ROS'))
    client.run()

    block_positions = get_block_positions(client)

    # ================================== YOUR CODE HERE ==================================



    # ====================================================================================

    client.terminate()


def reset_gazebo_world() -> None:

    client = roslibpy.Ros(host='localhost', port=9090)
    client.on_ready(lambda: print('Connected to ROS'))
    client.run()

    # Joint move robot back to 0 position
    action_client = roslibpy.actionlib.ActionClient(client, '/joint_trajectory_action', 'control_msgs/FollowJointTrajectoryAction')
    goal = roslibpy.actionlib.Goal(action_client,
                                   {'trajectory': {'joint_names': ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
                                                   'points': [{'positions': [0, 0, 0, 0, 0, 0],
                                                               'time_from_start': {'secs': 0, 'nsecs': 1}}]},
                                    'goal_time_tolerance': 0.0})
    goal.send()
    action_client.dispose()

    time.sleep(1)

    # Reset the Gazebo simulation
    reset_service = roslibpy.Service(client, '/gazebo/reset_simulation', 'std_srvs/Empty')
    reset_service.call(roslibpy.ServiceRequest({}))

    client.terminate()


if __name__ == "__main__":

    # Test part 1
    # assert_almost_equal(quaternion_to_rotation_matrix(np.array([-0.7071068, 0.0, 0.7071068, 0.0])), np.array([[0.0, 0.0, -1.0],[0.0, -1.0, 0.0],[-1.0, 0.0, 0.0]]))
    # assert_almost_equal(rotation_matrix_to_quaternion(np.array([[0.0, 0.0, -1.0],[0.0, -1.0, 0.0],[-1.0, 0.0, 0.0]])), np.array([-0.7071068, 0.0, 0.7071068, 0.0]))

    # Test part 2
    # knock_over_blocks()

    # Uncomment to reset the Gazebo world
    # reset_gazebo_world()
