import numpy as np
import matplotlib.pyplot as plt

def plot_points(points):
    # Unzip the list of points into two lists, x and y coordinates
    x_values, y_values = zip(*points)
    
    # Create a scatter plot of the points
    plt.plot(x_values, y_values, '-o')
    
    # Optional: Define the aspect ratio to be equal, so the scale is the same on both axes
    plt.gca().set_aspect('equal', adjustable='box')
    
    # Show the plot
    
    plt.show()

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

def jittery_waypoints(length):
    "draws a line with random jitters"
    n = 100
    xys = []
    x = 0
    y = 0
    for i in range(n):
        x = x + np.random.uniform(0,10)
        y = y + np.random.uniform(0,10)
        xys.append([x,y])


    """xys = []
    dx = (length)/n
    dy = (length)/n
    for i in range(n):
        xys.append([np.random.uniform(-5,5),dy*i])"""
    
    #print(xys)
    
    return xys

def random_polygon_waypoints():
    """returns the waypoints of a random shape"""
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


    




    


randomshape = random_polygon_waypoints()
plot_points(randomshape)