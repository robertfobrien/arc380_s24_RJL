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


# ====================== Drawing effects and helper functions ============================

# Part 2

# Just made this one after class.
def goto_task_point(task_frame, x, y):
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

def execute_tf_points(task_frame, points):
    """Executes a series of points in the task frame"""
    for p in points:
        goto_task_point(task_frame, p[0], p[1])

def polygon_waypoints(r, x, y, n):
    """Returns the points of an sided polygon with radius r """
    points_in_tf=[]

    angle=(2 * np.pi)/(n)

    #print(n)

    x_1 = x + r*np.cos(0)
    y_1 = y + r*np.sin(0)

    for i in range(n):
        x_temp = x + r*np.cos(i*angle)
        y_temp = y + r*np.sin(i*angle)
        points_in_tf.append((x_temp, y_temp))
    points_in_tf.append([x_1, y_1])

    return np.round(points_in_tf)

def square_waypoints(r, x, y, rotation):
    """returns the points of a square at x,y with lengths "r" and rotation"""
    p1 = [x,y]
    p2 = [x+np.cos(rotation)*r, y+np.sin(rotation)*r]
    p3 = [x-np.sin(rotation)*r, y+np.cos(rotation)*r]

    diag_size = np.sqrt(r**2 + r**2)
    #print(diag_size)
    p4 = [x+np.cos(rotation+np.pi/4)*diag_size, y+np.sin(rotation+np.pi/4)*diag_size]

    arr = np.round([p1,p2,p3,p4],3)

    return arr 

def draw_cirle(x,y,r):
    """Returns the points of circle"""
    #this is basically the same as the polygon but it just has a high enough n
    n = 75
    points_in_tf=[]

    angle=(2 * np.pi)/(n)

    for i in range(n):
        x_temp = x + r*np.cos(i*angle)
        y_temp = y + r*np.sin(i*angle)
        points_in_tf.append((x_temp, y_temp))

    return points_in_tf

def varying_thickness(task_frame,x1,y1,x2,y2,dz):
    """Moves along a line from x1,y1 to x2,y2 with varying thickness"""
    n = 5 # can be any number that shows the varying of thickness

    # frame just above the paper so its less thickness when it draws
    above_frame = task_frame
    above_frame.point.z = above_frame.point.z + dz
    
    goto_task_point(task_frame,x1,y1)
    goto_task_point(above_frame,x1,y1)
    """ # Uncomment this stuff if we want to switch between thick and thin
    #xys gives a points along a line in x,y
    xys = []
    dx = (x2-x1)/n
    dy = (y2-y1)/n
    for i in range(n):
        xys.append([dx*i,dy*i])
    
    print(xys)

    for i in range(n): #alternates between the above and the 
        if(i%2 == 0): #even ones
            goto_task_point(task_frame ,xys[i][0],xys[i][1])
        else: #odds
            goto_task_point(above_frame ,xys[i][0],xys[i][1])"""
    return 1

def move_pen_above_paper(task_frame):
    """moves the pen to above the paper"""
    dz = 100 # mm

    # frame just above the paper so its less thickness when it draws
    above_frame = task_frame
    above_frame.point.z = above_frame.point.z + dz
    goto_task_point(above_frame, 0,0)



def dashed_lines(task_frame,x1,y1,x2,y2,n):
    """Moves along a line from x1,y1 to x2,y2 with varying thickness"""
    dz = 10

    # frame just above the paper so its less thickness when it draws
    above_frame = task_frame
    above_frame.point.z = above_frame.point.z + dz
    
    goto_task_point(task_frame,x1,y1)
    goto_task_point(above_frame,x1,y1)

    #xys gives a points along a line in x,y
    xys = []
    dx = (x2-x1)/n
    dy = (y2-y1)/n
    for i in range(n):
        xys.append([dx*i,dy*i])
    
    print(xys)

    for i in range(0,n,2): #alternates between the above and the 
        if((i/2)%2 == 0):
            goto_task_point(task_frame ,xys[i][0],xys[i][1])
            goto_task_point(task_frame ,xys[i+1][0],xys[i+1][1])
        else: #odds
            goto_task_point(above_frame ,xys[i][0],xys[i][1])
            goto_task_point(above_frame ,xys[i+1][0],xys[i+1][1])
    return 1

def jittery_waypoints(length):
    "draws a line with random jitters"
    n = 100
    xys = []
    x = 0
    y = 0
    for i in range(n):
        x = x + np.random.uniform(0,5)
        y = y + np.random.uniform(0,5)
        xys.append([x,y])


    """xys = []
    dx = (length)/n
    dy = (length)/n
    for i in range(n):
        xys.append([np.random.uniform(-5,5),dy*i])"""
    
    #print(xys)
    
    return xys

def random_polygon_waypoints(n):
    #shape size
    min_size = 10
    max_size = 30

    max_coord = 500
    min_coord = 0

    min_n = 3
    max_n = 8


    r = np.random.uniform(min_size, max_size)
    x = np.random.uniform(min_coord, max_coord)
    y = np.random.uniform(min_coord, max_coord)
    sides = int(np.random.uniform(min_n, max_n))
    shape_points = polygon_waypoints(r,x,y,sides)
    #print(shape_points)

    return shape_points

def draw_random_shapes(n):
    """ draws an assortment of n random shapes """
    for i in range(n):
        shape_points = random_polygon_waypoints()
        print("Drawing shape: , i")
        execute_tf_points(task_frame, shape_points)
        move_pen_above_paper(task_frame)
    return 1
    

    


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

    varying_thickness(task_frame,0,0,1,1,1.2)

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
