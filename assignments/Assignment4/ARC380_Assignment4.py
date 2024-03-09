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

def execute_points_varying(task_frame, points, abb_rrc):
    """Executes a series of points in the task frame"""
    for p in points:
        goto_task_point(task_frame, p[0], p[1], abb_rrc)

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

    arr = np.round([p1,p2,p4,p3,p1],3)

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
    points_in_tf.append(points_in_tf[0])
    return points_in_tf

def varying_thickness(task_frame,x1,y1,x2,y2,dz, abb_rrc):
    """Moves along a line from x1,y1 to x2,y2 with varying thickness"""
    n = 5 # can be any number that shows the varying of thickness

    # frame just above the paper so its less thickness when it draws
    task_frame.point.z = task_frame.point.z + 1
    goto_task_point(task_frame,x1,y1, abb_rrc)
    
    below = task_frame
    below.point.z = below.point.z - dz
    goto_task_point(below,x2,y2, abb_rrc)
    below.point.z = below.point.z + dz - 1

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


def varying_thickness_to_point(task_frame,x1,y1,x2,y2,dz, abb_rrc):
    """Moves along a line from x1,y1 to x2,y2 with varying thickness"""
    n = 5 # can be any number that shows the varying of thickness

    # frame just above the paper so its less thickness when it draws
    task_frame.point.z = task_frame.point.z + 1
    goto_task_point(task_frame,x1,y1, abb_rrc)
    
    below = task_frame
    below.point.z = below.point.z - dz
    goto_task_point(below,x2,y2, abb_rrc)
    below.point.z = below.point.z + dz - 1

    goto_task_point(task_frame, x2, y2, abb_rrc)


def lift_up_pen(task_frame,mm_above=100, x=0, y=0, abb_rrc=None):

    # frame just above the paper so its less thickness when it draws
    above_frame = task_frame
    above_frame.point.z = above_frame.point.z + mm_above
    goto_task_point(above_frame, x,y, abb_rrc)
    above_frame.point.z = above_frame.point.z - mm_above



def dashed_lines(task_frame,n,x1,y1, x2,y2, gap_ratio,abb_rrc):
    """Moves along a line from x1,y1 to x2,y2 with varying thickness"""
    dz = 10

    x = x1
    y = x1
    dx = (x2-x1)/n
    dy = (y2-y1)/n
    for i in range(n):
        goto_task_point(task_frame,x,y, abb_rrc)

        for i in range(gap_ratio):
            x = x+dx
            y = y+dy

        goto_task_point(task_frame,x,y, abb_rrc)

        # frame just above the paper so its less thickness when it draws
        task_frame = task_frame
        task_frame.point.z = task_frame.point.z + dz

        x = x+dx
        y = y+dy

        goto_task_point(task_frame,x,y, abb_rrc)

        task_frame.point.z = task_frame.point.z - dz

def jittery_waypoints(length):
    "draws a line with random jitters"
    n = 100
    xys = []
    x = 0
    y = 0
    for i in range(n):
        x = x + np.random.uniform(0,7)
        y = y + np.random.uniform(0,7)
        xys.append([x,y])


    """xys = []
    dx = (length)/n
    dy = (length)/n
    for i in range(n):
        xys.append([np.random.uniform(-5,5),dy*i])"""
    
    #print(xys)
    
    return xys

def random_polygon_waypoints():
    #shape size
    min_size = 5
    max_size = 20

    max_coord = 150
    min_coord = 50

    min_n = 3
    max_n = 8


    r = np.random.uniform(min_size, max_size)
    x = np.random.uniform(min_coord, max_coord)
    y = np.random.uniform(min_coord, max_coord)
    sides = int(np.random.uniform(min_n, max_n))
    shape_points = polygon_waypoints(r,x,y,sides)
    #print(shape_points)

    return shape_points

def draw_random_shapes(task_frame, n, abb_rrc):
    """ draws an assortment of n random shapes """
    for i in range(n):
        shape_points = random_polygon_waypoints()
        print("Drawing shape: ", i)
        execute_tf_points(task_frame, shape_points, abb_rrc)
        x = shape_points[0][0]
        y = shape_points[0][1]
        lift_up_pen(task_frame,20, x, y, abb_rrc)
    return 1
    
# ========================================================================================


if __name__ == '__main__':


    # points in the world frame of parts of the paper
    # adjust z if we need to...
    task_point_in_wf_1 = cg.Point(271.02,483.59, 30) 
    task_point_in_wf_2 = cg.Point(257.31,335.69,28) 
    task_point_in_wf_3 = cg.Point(69.25, 485.12, 30) 
    task_point_in_wf_4 = cg.Point(87.88,339.13,30)
    

    task_frame = create_frame_from_points(task_point_in_wf_1, task_point_in_wf_2, task_point_in_wf_3)
    #print("task frame: ", task_frame)

    #varying_thickness(task_frame,0,0,1,1,1.2)

    # Create Ros Client
    ros = rrc.RosClient()
    ros.run()

    # Create ABB Client
    abb_rrc = rrc.AbbClient(ros, '/rob1-rw6')
    print('Connected.')

    # ================================== YOUR CODE HERE ==================================

    # Parts 1.e. and 2
    # set the speed
    speed = 100

    # Move robot to the task frame origin
    #goto_task_point(task_frame, 0 , 0 , abb_rrc)

    #wp = polygon_waypoints(40, 100,100, 5)
    #execute_tf_points(task_frame, wp, abb_rrc)

    """
    wp = square_waypoints(10, 200,200, np.pi/4)
    execute_tf_points(task_frame, wp, abb_rrc)

    lift_up_pen(task_frame, 40, 150, 150, abb_rrc)

    wp = square_waypoints(20, 150, 150, np.pi/7)
    execute_tf_points(task_frame, wp, abb_rrc)

    lift_up_pen(task_frame, 40, 100, 150, abb_rrc)

    wp = square_waypoints(30, 100,150, np.pi*(3/2))
    execute_tf_points(task_frame, wp, abb_rrc)

    lift_up_pen(task_frame, 40, 150, 100, abb_rrc)

    wp = square_waypoints(40, 150,100, np.pi/6)
    execute_tf_points(task_frame, wp, abb_rrc)

    lift_up_pen(task_frame, 40, 0, 250, abb_rrc) """

    """ lift_up_pen(task_frame, 50, 100, 100, abb_rrc)
    wp = draw_cirle(100, 100,100)
    execute_tf_points(task_frame, wp, abb_rrc)

    lift_up_pen(task_frame, 50, 100, 100, abb_rrc)
    wp = draw_cirle(100, 150,50)
    execute_tf_points(task_frame, wp, abb_rrc)

    lift_up_pen(task_frame, 50, 100, 100, abb_rrc)
    wp = draw_cirle(100, 25,50)
    execute_tf_points(task_frame, wp, abb_rrc)
    
    #moves to point 0mm, 50mm
    #goto_task_point(task_frame, 0 , 50 , abb_rrc) """
    
    """ lift_up_pen(task_frame, 50, 100, 100, abb_rrc)
    draw_random_shapes(task_frame, 5, abb_rrc) """


    """ wp = jittery_waypoints(40)
    execute_tf_points(task_frame, wp, abb_rrc)
    lift_up_pen(task_frame, 50, 100, 100, abb_rrc)"""

    ### MAKING THE LOGO 

    # make a circle 
    lift_up_pen(task_frame, 50, 100, 100, abb_rrc)
    wp = draw_cirle(100, 100, 20)
    execute_tf_points(task_frame, wp, abb_rrc)

    #varying thickness
    varying_thickness(task_frame,50,100,100,50,3,abb_rrc)

    # dashed lines
    dashed_lines(task_frame, 5, 50 ,50, 100, 100 , 1,abb_rrc)
    lift_up_pen(task_frame, 50, 100, 100, abb_rrc)

    #square
    wp = square_waypoints(20, 60, 60, np.pi/4)
    execute_tf_points(task_frame, wp, abb_rrc)
    lift_up_pen(task_frame, 50, 100, 100, abb_rrc)

    # polygon
    wp = polygon_waypoints(10, 50, 50, 5)
    execute_tf_points(task_frame, wp, abb_rrc)
    lift_up_pen(task_frame, 50, 100, 100, abb_rrc)

    # random polygons
    wp = draw_random_shapes(task_frame, 5, abb_rrc)
    execute_tf_points(task_frame, wp, abb_rrc)
    lift_up_pen(task_frame, 50, 100, 100, abb_rrc)

    #jittery
    wp = jittery_waypoints(40)
    execute_tf_points(task_frame, wp, abb_rrc)
    lift_up_pen(task_frame, 50, 100, 100, abb_rrc)

    lift_up_pen(task_frame, 50, 100, 100, abb_rrc)



    # ====================================================================================

    # End of Code
    print('Finished')

    # Close client
    ros.close()
    ros.terminate()
