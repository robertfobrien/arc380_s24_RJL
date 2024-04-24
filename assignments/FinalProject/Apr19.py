import time
import json
import numpy

#import pyrealsense2 as rs
import numpy as np
import cv2
from matplotlib import pyplot as plt
import compas.geometry as cg
import compas_rrc as rrc
from cv2 import aruco


speed=75



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

def change_end_effector_orientation(task_frame, orientation ,abb_rrc):
    """get all the joint states and only adjust end effector angle, send as a command"""
    if orientation == None:
        return
    joints, external_axes = abb_rrc.send_and_wait(rrc.GetJoints())
    print("joints:", joints)
    joints.rax_6 = joints.rax_6 + orientation # only change the end effector value
    done = abb_rrc.send_and_wait(rrc.MoveToJoints(joints, [], speed, rrc.Zone.FINE)) # move the end effector

def goto_task_point(task_frame, x, y, abb_rrc):
    """ goe to a point in the task frame"""
    f1 = cg.Frame([x,y,0], [1,0,0], [0,1,0]) #[1,0,0] and [0,1,0] define the x and y planes
    f1_p = transform_task_to_world_frame(f1, task_frame)
    next = abb_rrc.send_and_wait(rrc.MoveToFrame(f1_p, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))

def move_linear_z(z, abb_rrc):
    """ Move linearly by a relative amount in the z direction according to the robot"""
    frame = abb_rrc.send_and_wait(rrc.GetFrame())
    frame.point.z += z
    abb_rrc.send_and_wait(rrc.MoveToFrame(frame, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))
    frame.point.z -= z

# Mock function to convert yaw angle to a rotation matrix.
def yaw_to_matrix(yaw):
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    return [[cos_yaw, -sin_yaw, 0], [sin_yaw, cos_yaw, 0], [0, 0, 1]]   

def goto_above_task_point(task_frame, x, y, z, abb_rrc):
    """Go to an x,y,z point in the tf"""""""""
    task_frame.point.z = task_frame.point.z + z
    task = goto_task_point(task_frame, x, y, abb_rrc)
    task_frame.point.z = task_frame.point.z - z
    return 1

def goto_robot_home(abb_rrc):
    """Go to home position (linear joint move)"""
    home = rrc.RobotJoints([0, 0, 0, 0, 90, 0])
    done = abb_rrc.send_and_wait(rrc.MoveToJoints(home, [], speed, rrc.Zone.FINE))


def goto_task_f_origin(task_frame,abb_rrc):
    """Go to the origin of the taskframe"""
    done = abb_rrc.send_and_wait(rrc.MoveToFrame(task_frame, speed, rrc.Zone.FINE, rrc.Motion.LINEAR))


def execute_tf_points(task_frame, points, abb_rrc):
    """Executes a series of points in the task frame"""
    for p in points:
        goto_task_point(task_frame, p[0], p[1], abb_rrc)

def get_mm_per_pixel(pixel_width, pixel_height):
    # get (mm)/(#pixel) 
    actual_width = 480 #mm 
    actual_height = 300 #mm 
    # average them 
    a = actual_width/pixel_width
    b = actual_height/pixel_height
    return (a+b)/(2.0)


def get_shapes_info(img_path, expected_k):
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
    cropped_copy = cropped_image.copy()

    gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0) 
    ret,block_thresh = cv2.threshold(blurred,90,255,cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(block_thresh,cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    block_contours = []
    block_centers = []
    block_orientations = []
    for contour in contours: 
        shape_area = cv2.contourArea(contour)
        if shape_area > 3000 and shape_area < 4000:
            #print("Blocks found with area: ", shape_area)
            block_contours.append(contour)
            # center
            M = cv2.moments(contour)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                block_centers.append((cx,cy))
                cv2.circle(cropped_copy, (cx, cy), 5, (0, 0, 255), -1)

            rect = cv2.minAreaRect(contour)

            #fixing orientation problems 
            orientation = rect[2]
            if orientation > 45:
                orientation = orientation - 90
            
            block_orientations.append(orientation)
            angle_rad = np.deg2rad(int(orientation))    # Orientation angle of the rectangle (in radians)
            length = 20         # The length of the frame axes
            # Calculate the end points of each axis
            # X-axis (you could change color later if needed)
            #end_x = (int(cx + length * np.cos(angle_rad)), int(cy + length * np.sin(angle_rad)))
            # Y-axis (rotate the angle by 90 degrees or pi/2 radians for the perpendicular)
            end_y = (int(cx + length * np.cos(angle_rad + np.pi/2)), int(cy + length * np.sin(angle_rad + np.pi/2)))
            #cv2.line(cropped_copy, (cx,cy), end_x, (0, 0, 255), 2)
            cv2.line(cropped_copy, (cx,cy), end_y, (0, 255, 0), 2)
            cv2.putText(cropped_copy, str(" B"), (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.drawContours(block_thresh, block_contours, -1, (0,255,0), 3)
    cv2.imshow('Circles and blocks', block_thresh)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    #print("block centers = ", block_centers)
    #print("block orientations = ", block_orientations)
    # display the cropped image to the id tags
    #plt.imshow(cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB))
    #plt.title('cropped image')
    #plt.show()

    # Run k-means clustering on the image

    # Reshape our image data to a flattened list of RGB values
    img_data = cropped_image.reshape((-1, 3))
    img_data = np.float32(img_data)

    # Define the number of clusters
    k = 17# expected_k 
    # Used to be 8

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

    # add the blocks to the shape dict
    for i in range(len(block_centers)):
        object_info = {
                    "color": None,
                    "shape": None,
                    "size": None,  
                    "position": None,
                    "orientation": None 
                }
        object_info['shape'] = "block"
        object_info['orientation'] = block_orientations[i]
        object_info['position'] = [block_centers[i][0]*mm_per_pixel,block_centers[i][1]*mm_per_pixel]
        object_info['color'] = "9"
        object_info['size'] = 3000
        shape_dict.append(object_info)

        
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


                    if roundness > 0.8: # testing for circle
                        object_info['shape'] = "disk"
                        cv2.circle(cropped_copy, (cx, cy), 5, (0, 0, 255), -1)
                        cv2.putText(cropped_copy, str(" C"), (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                        num_circles = num_circles + 1
                        is_a_good_shape = True
                    elif np.abs(area - (perimeter/4)**2) < 900: # testing for square --> making sure (0.25*perim)^2 is close to the area of the object
                        object_info['shape'] = "square"
                        #print("area from perimeter: ", str((perimeter/4)**2))
                        #print("actual area: ", area)
                        is_a_good_shape = True
                        object_info['orientation'] = cv2.minAreaRect(cnt)[2]
                        #print(object_info['orientation'])

                    #if np.abs(area - (perimeter/4)**2) < 1500:
                        #print("squareness: :", np.abs(area - (perimeter/4)**2))
                        #print("seen as a: ",  object_info['shape'])

                    
                    #some attributes that we're adding about this particular shape
                    object_info['color'] = str(i) # adds the kmeans id as the 'color'
                    object_info['position'] = [cx*mm_per_pixel,cy*mm_per_pixel]
                    object_info['size'] = area

                    # ----------------------------------

                    #add this shape to the full array that tells us about all our shapes, but only if it's "good"
                    if is_a_good_shape:
                        
                        already_a_shape_drawn_there = False
                        for shape in shape_dict: #make sure there isn't already a
                            x_dist = shape['position'][0] - object_info['position'][0]
                            y_dist = shape['position'][1] - object_info['position'][1]
                            total_dist = np.sqrt(x_dist**2 + y_dist**2)
                            if total_dist < 3: # if there's already a shape there within 3mm
                                already_a_shape_drawn_there = True
                                #print("Already a shape there with a distance of", total_dist)
                        
                        if not already_a_shape_drawn_there: 
                            shape_dict.append(object_info)

                            # center of cluster
                            cv2.circle(output_image, (cx, cy), 5, (0, 0, 255), -1)
                            # put id on cluster
                            cv2.putText(output_image, str(i), (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                            if object_info['shape'] != 'square':
                                cv2.putText(output_image, "  circ", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

                            if object_info['shape'] != 'square' and object_info['shape'] != 'circle':
                                cv2.putText(output_image, "  NA", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)


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

    cv2.imshow('Circles and blocks', cropped_copy)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    return shape_dict

if __name__ == '__main__':
    BLOCK_HEIGHT = 15 #mm
    ACRYLIC_HEIGHT = 3 #mm
    TRAVEL_BUFFER = 50 #mm
    PLACEMENT_BUFFER = 2 #mm

    # Make sure markers are in correct order for aruco vs plane formation
        # commented out this part just to test the image processing. 
    aruco_1 = cg.Point(244.67, 487.16, 23.51) # Origin, bottom left
    aruco_2 = cg.Point(235.93, 134.19, 19.29) # x-axis, top left
    aruco_3 = cg.Point(-236.45, 496.64, 23.49) # xy-plane, bottom right
    aruco_4 = cg.Point(-236.01, 194.54, 20.05) # Top right

    pt_1 =  cg.Point(-236.41, 494.42, 24.13)
    pt_2 =  cg.Point(250.35, 491.66, 23.13)
    pt_3 = cg.Point(-235.71, 190.92, 23.26)
    task_frame = create_frame_from_points(pt_1, pt_2, pt_3)

    print("TASK FRAME")
    print(task_frame)

    # Create Ros Client
    ros = rrc.RosClient()
    ros.run()

    # Create ABB Client
    abb_rrc = rrc.AbbClient(ros, '/rob1-rw6')
    print('Connected.')


    #goto_above_task_point(x = 250, y=250, z=10, task_frame=task_frame, abb_rrc=abb_rrc)
    #change_end_effector_orientation(task_frame=task_frame, orientation=75, abb_rrc=abb_rrc)
    #goto_above_task_point(x = 250, y=250, z=20, task_frame=task_frame, abb_rrc=abb_rrc)
    

    # Load the design
    f = open('tower.json')
    data = json.load(f)
    f.close()

    objects = []

    for object in data:
        objects.append(data[object])

    objects = sorted(objects, key = lambda z: z["position"][2])

    expected_k = len(objects)

    # takes the image and saves it to "test_frame.jpg" using opencv 
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_EXPOSURE, -4)
    ret, frame = cap.read()
    time.sleep(2)
    ret, frame = cap.read()

    cv2.imshow('raw frame', frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    cv2.imwrite('test_frame.jpg', frame)

    print("Photo taken: ", str(ret))
    cap.release()


    #get all shape info. An array of object infos
    shapes = get_shapes_info('test_frame.jpg', expected_k)


    # remap the unique shape values to 0,1,2,...
    unique_colors = sorted(set(item['color'] for item in shapes))
    color_to_group = {shape: str(i) for i, shape in enumerate(unique_colors)}
    for shape in shapes:
        shape['color'] = color_to_group[shape['color']]

    # sorts the shape by negative size
    shapes = sorted(shapes, key=lambda x: -x['size'])

    # for debugging
    print(shapes)
    print("Just got all shape info ^ sorted by area")

    current_height = 0
    print("     ")
    print("Now looking for objects that match our criteria:")

    building_x = objects[0]['position'][0] # this will let us center the building area at the task frame origin
    building_y = objects[0]['position'][1]
    
    for object in objects:
        # this will let us center the building area at the task frame origin
        object['position'][0] = object['position'][0] - building_x
        object['position'][1] = object['position'][1] - building_y

        print("Looking for an object with this criteria: ", object)
        
        match = None

        for i, shape in enumerate(shapes):

            # right now we're just making sure the shape is the same as the shame we want
            if object['shape'] == shape['shape']:
                print("Got it. Here is its information: ", shape)
                print()
                match = shape
                shapes[i]['shape'] = "used" # mark the shape used so we dont use the same thing again
                break

        if match is None:
            print("Oops, no good shapes were found.")
            print()
        else:
            # Go to the shape
            x = match['position'][0]
            y = match['position'][1]

            print("going above the shape")
            goto_above_task_point(task_frame, x, y, current_height + TRAVEL_BUFFER, abb_rrc)

            # go down to the actual shape
            print("going down to the shape")
            if object['shape'] == 'block':
                print("Moving to shape, Block")
                goto_above_task_point(x=x, y=y, z=BLOCK_HEIGHT, task_frame=task_frame, abb_rrc=abb_rrc)
            else:
                print("Moving to shape, Not block")
                goto_above_task_point(task_frame=task_frame, x=x, y=y, z=ACRYLIC_HEIGHT, abb_rrc=abb_rrc)

            # Changes the end effector to match the shape we're gonna pick up
            change_end_effector_orientation(task_frame=task_frame, orientation=shape['orientation'], abb_rrc=abb_rrc)
            # move down 1mm to the block or shape
            move_linear_z(-1, abb_rrc) # -1mm

            #  activate the suction
            low = 0
            high = 1
            done = abb_rrc.send_and_wait(rrc.SetDigital('DO00',high))

            # add wait time for suction to engage
            t = 1.0 # Unit [s]
            done = abb_rrc.send_and_wait(rrc.WaitTime(t))

             #lift it above 
            goto_above_task_point(task_frame, x, y, current_height + TRAVEL_BUFFER, abb_rrc) #10mm

            # Go to the object target position
            x = object['position'][0]
            y = object['position'][1]
            z = object['position'][2]

            # Move above target
            goto_above_task_point(task_frame, x, y, current_height + TRAVEL_BUFFER, abb_rrc) #10mm

            
            # move to 5mm above where we want to place the block
            goto_above_task_point(task_frame, x, y, z + PLACEMENT_BUFFER + 5, abb_rrc) #10mm
            #change orientation to final orientation
            change_end_effector_orientation(task_frame=task_frame, orientation=object['orientation'], abb_rrc=abb_rrc)
            #move down 5mm 
            move_linear_z(-5, abb_rrc)

            # turn off suction
            done = abb_rrc.send_and_wait(rrc.SetDigital('DO00',low))

            # wait time for suction to de engage
            t = 1.0 # Unit [s]
            done = abb_rrc.send_and_wait(rrc.WaitTime(t))

            #raise above the pile so it doesnt knock it over 
            goto_above_task_point(task_frame, x, y, current_height + TRAVEL_BUFFER, abb_rrc)
        
    # End of Code
    print('Finished')

    # Close client
    ros.close()
    ros.terminate()
        
        



