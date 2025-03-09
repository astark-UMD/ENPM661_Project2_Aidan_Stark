import numpy as np
import cv2
from queue import Queue
import copy

# Create a Node class that contains the node's index, its parent's index, its cost-to-come, and it's position in x and y.
class Node:
    def __init__(self, Node_Index, Parent_Node_Index, Node_Cost, Node_x, Node_y):
        self.Node_Index = int(Node_Index)
        self.Parent_Node_Index = int(Parent_Node_Index)
        self.Node_Cost = Node_Cost
        self.Node_x = int(Node_x)
        self.Node_y = int(Node_y)

# Each of the eight move functions takes in a node, copies its information
# to generate the basis of the new node as a result of movement, 
# updates the cost of the new node to execute that movement from the 
# parent node, and updates the position of the new node.
def move_up(given_Node):
    newNode = copy.deepcopy(given_Node)
    newNode.Parent_Node_Index = newNode.Node_Index
    newNode.Node_Index = -1
    newNode.Node_Cost = newNode.Node_Cost + 1
    # newNode.Node_x = newNode.Node_x
    newNode.Node_y = newNode.Node_y + 1
    return newNode

def move_down(given_Node):
    newNode = copy.deepcopy(given_Node)
    newNode.Parent_Node_Index = newNode.Node_Index
    newNode.Node_Index = -1
    newNode.Node_Cost = newNode.Node_Cost + 1
    # newNode.Node_x = newNode.Node_x
    newNode.Node_y = newNode.Node_y -1
    return newNode

def move_left(given_Node):
    newNode = copy.deepcopy(given_Node)
    newNode.Parent_Node_Index = newNode.Node_Index
    newNode.Node_Index = -1
    newNode.Node_Cost = newNode.Node_Cost + 1
    newNode.Node_x = newNode.Node_x - 1
    # newNode.Node_y = newNode.Node_y
    return newNode

def move_right(given_Node):
    newNode = copy.deepcopy(given_Node)
    newNode.Parent_Node_Index = newNode.Node_Index
    newNode.Node_Index = -1
    newNode.Node_Cost = newNode.Node_Cost + 1
    newNode.Node_x = newNode.Node_x + 1
    # newNode.Node_y = newNode.Node_y 
    return newNode

def move_up_left(given_Node):
    newNode = copy.deepcopy(given_Node)
    newNode.Parent_Node_Index = newNode.Node_Index
    newNode.Node_Index = -1
    newNode.Node_Cost = newNode.Node_Cost + 1
    newNode.Node_x = newNode.Node_x - 1
    newNode.Node_y = newNode.Node_y + 1
    return newNode

def move_up_right(given_Node):
    newNode = copy.deepcopy(given_Node)
    newNode.Parent_Node_Index = newNode.Node_Index
    newNode.Node_Index = -1
    newNode.Node_Cost = newNode.Node_Cost + 1
    newNode.Node_x = newNode.Node_x + 1
    newNode.Node_y = newNode.Node_y + 1
    return newNode

def move_down_left(given_Node):
    newNode = copy.deepcopy(given_Node)
    newNode.Parent_Node_Index = newNode.Node_Index
    newNode.Node_Index = -1
    newNode.Node_Cost = newNode.Node_Cost + 1
    newNode.Node_x = newNode.Node_x - 1
    newNode.Node_y = newNode.Node_y - 1
    return newNode

def move_down_right(given_Node):
    newNode = copy.deepcopy(given_Node)
    newNode.Parent_Node_Index = newNode.Node_Index
    newNode.Node_Index = -1
    newNode.Node_Cost = newNode.Node_Cost + 1
    newNode.Node_x = newNode.Node_x + 1
    newNode.Node_y = newNode.Node_y - 1
    return newNode

# gen_obstacle_map generates the map and its obstacles using half planes 
# and semi-algebraic models. Each obstacle is composed of a union of convex
# polygons that define it. It then constructs an in image in BGR and sets
# obstacle pixels as red in the image. Additionally, the entire obstacle map
# can be configured for a certain resolution by the given scale factor, sf.
# When sf = 1, each pixel represents 1 mm. sf = 10, each pixel represents .1 mm.
# The robot navigates on a per pixel basis, this larger scale factors will result in
# more optimal paths due to increased resolution, but slower processing time. 
def gen_obstacle_map(sf=10):
    # Set the height and width of the image in pixels.
    height = 50*sf
    width = 180*sf
    # Create blank canvas.
    obstacle_map = np.zeros((height,width,3), dtype=np.uint8 )
    
    # Define polygons for E obstacle.
    def E_obstacle1(x,y):
        return (10*sf <= x <= 15*sf) and (10*sf <= y <= 35*sf)
    
    def E_obstacle2(x,y):
        return (15*sf <= x <= 23*sf) and (10*sf <= y <= 15*sf)
    
    def E_obstacle3(x,y):
        return (15*sf <= x <= 23*sf) and (20*sf <= y <= 25*sf)
    
    def E_obstacle4(x,y):
        return (15*sf <= x <= 23*sf) and (30*sf <= y <= 35*sf)
    
    # Define polygons for N obstacle.
    def N_obstacle1(x,y):
        return (30*sf <= x <= 35*sf) and (10*sf <= y <= 35*sf)
    
    def N_obstacle2(x,y):
        return (40*sf <= x <= 45*sf) and (10*sf <= y <= 35*sf)
    
    def N_obstacle3(x,y):
        return (35*sf <= x <= 40*sf) and (-3*x+130*sf <= y <= -3*x+140*sf)
    
    # Define polygons for P obstacle.
    def P_obstacle1(x,y):
        return (53*sf <= x <= 58*sf) and (10*sf <= y <= 35*sf)
    
    def P_obstacle2(x,y):
        return (58*sf <= x <= 64*sf) and ((x-58*sf)**2 + (y-29*sf)**2 <= (6*sf)**2)
    
    # Define polygons for M obstacle.
    def M_obstacle1(x,y):
        return (70*sf <= x <= 75*sf) and (10*sf <= y <= 35*sf)
    
    def M_obstacle2(x,y):
        return (88*sf <= x <= 93*sf) and (10*sf <= y <= 35*sf)
    
    def M_obstacle3(x,y):
        return (79*sf <= x <= 84*sf) and (10*sf <= y <= 15*sf)
    
    def M_obstacle4(x,y):
        return (75*sf <= x <= 79*sf) and (-5*x+400*sf <= y <= -5*x+410*sf) and (10*sf <= y) 
    
    def M_obstacle5(x,y):
        return (84*sf <= x <= 88*sf) and (5*x-415*sf <= y <= 5*x-405*sf) and (10*sf <= y )
    
    # Define polygons for first Six obstacle.
    def Six1_obstacle1(x,y):
        return ((x-109*sf)**2 + (y-19*sf)**2 <= (9*sf)**2)
    
    def Six1_obstacle2(x,y):
        return ((x-121.5*sf)**2 + (y-19*sf)**2 <= (21.50*sf)**2) and ((x-121.5*sf)**2 + (y-19*sf)**2 >= (16.50*sf)**2) and (19*sf <= y <= -1.732*x+229.438*sf)
    
    def Six1_obstacle3(x,y):
        return ((x-112*sf)**2 + (y-35.454*sf)**2 <= (2.5*sf)**2)
    
    # Define polygons for second Six obstacle.
    def Six2_obstacle1(x,y):
        return ((x-132*sf)**2 + (y-19*sf)**2 <= (9*sf)**2)
    
    def Six2_obstacle2(x,y):
        return ((x-144.5*sf)**2 + (y-19*sf)**2 <= (21.50*sf)**2) and ((x-144.5*sf)**2 + (y-19*sf)**2 >= (16.50*sf)**2) and (19*sf <= y <= -1.732*x+269.274*sf)
    
    def Six2_obstacle3(x,y):
        return ((x-135*sf)**2 + (y-35.454*sf)**2 <= (2.5*sf)**2)
    
    # Define polygon for One obstacle.
    def One_obstacle1(x,y):
        return (148*sf <= x <= 153*sf) and (10*sf <= y <= 38*sf)

    # For every pixel in the image, check if it is within the bounds of any obstacle.
    # If it is, set it's color to red.
    for y in range(height):
        for x in range(width):
            if (E_obstacle1(x, y) or E_obstacle2(x,y) or E_obstacle3(x,y) or E_obstacle4(x,y) 
                or N_obstacle1(x,y) or N_obstacle2(x,y) or N_obstacle3(x,y)
                or P_obstacle1(x,y) or P_obstacle2(x,y)
                or M_obstacle1(x,y) or M_obstacle2(x,y) or M_obstacle3(x,y) or M_obstacle4(x,y) or M_obstacle5(x,y)
                or Six1_obstacle1(x,y) or Six1_obstacle2(x,y) or Six1_obstacle3(x,y)
                or Six2_obstacle1(x,y) or Six2_obstacle2(x,y) or Six2_obstacle3(x,y)
                or One_obstacle1(x,y)):
                obstacle_map[y, x] = (0, 0, 255) 
            

    # The math used assumed the origin was in the bottom left.
    # The image must be vertically flipped to satisy cv2 convention. 
    return np.flipud(obstacle_map)

# expand_obstacles takes the obstacle map given by gen_obstacle_map as an image, along with
# the scale factor sf, and generates two images. The first output_image, is a BGR image
# to draw on used for visual display only. expanded_mask is a grayscale image with white
# pixels as either obstacles or clearance space around obstacles. This function will take 
# the given obstacle image and apply a 2 mm radius circular kernel to the image. This ensures
# an accurate 2 mm clearance around every obstacle.
def expand_obstacles(image, scale_factor):

    # Convert image to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # Define color mask for red and create grayscale image.
    lower_red = np.array([0, 200, 200])
    upper_red = np.array([10, 255, 255])
    obstacle_mask = cv2.inRange(hsv, lower_red, upper_red)
    
    # Create circular structuring element for expansion
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * scale_factor + 1, 2 * scale_factor + 1))
    # Apply kernel to get 2 mm dilation around all elements.
    expanded_mask = cv2.dilate(obstacle_mask, kernel, iterations=1)

    # Apply 2 mm dilation to all of the borders.
    h, w = expanded_mask.shape
    expanded_mask[:scale_factor+1, :] = 255  # Top border
    expanded_mask[h-scale_factor:, :] = 255  # Bottom border
    expanded_mask[:, :scale_factor+1] = 255  # Left border
    expanded_mask[:, w-scale_factor:] = 255  # Right border
    
    # Create the output image and apply color orange to all obstacle and clearance
    # pixels.
    output_image = image.copy()
    output_image[np.where(expanded_mask == 255)] = [0, 165, 255]  # Color orange
    
    # Restore original red pixels. This creates an image with red obstacles,
    # and orange clearance zones. 
    output_image[np.where(obstacle_mask == 255)] = [0, 0, 255]  
    
    return output_image, expanded_mask

# prompt the user for a point. prompt is text that specifies what
# type of point is be given. prompt is solely used for terminal text output.
# sf is the scale factor to ensure the user's input is scaled correctly for the map. 
# image is passed to ensure the point is within the image bounds. obstacles is passed
# to ensure the user's point does not lie in an obstacle. The function returns the user's
# points as integers.
def get_point(prompt, sf, image, obstacles):

    valid_input = False
    # Repeat prompting the user until a valid input is given.
    while not valid_input:
        # Get x and y input and adjust by scale factor sf.
        x = int(input(f"Enter the x-coordinate for {prompt}: ")) * sf
        y = int(input(f"Enter the y-coordinate for {prompt}: ")) * sf

        # Ensure the point is valid. Break if it is. Prompt again if not.
        if valid_move(x, y, image.shape, obstacles):
            valid_input = True
        else:
            print("Invalid Input. Within Obstacle. Please try again.")

    return int(x), int(y)

# valid_move checks if a given point lies within the map bounds and
# if it is located within an obstacle. If the point is in the image and NOT in an obstacle,
# it returns True, meaning the position is valid/Free/open space.
def valid_move(x, y, map_shape, obstacles):
    return 0 <= x < map_shape[1] and 0 <= y < map_shape[0] and obstacles[y, x] == 0

# Check if we are at the goal position. 
def goal_check(x, y, end):
    return (x, y) == end

def BFS_search(map, obstacles, start, end, sf):

    # Convert y coordinates from origin bottom left (user input) to origin top left (cv2 convention).
    height, width, _ = map.shape
    start_x, start_y = start
    end_x, end_y = end
    start_y = height - start_y
    end_y = height - end_y
    start = (start_x, start_y)
    end = (end_x, end_y)
    
    # Open video file to write path planning images to.
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_filename = "BFS_Aidan_Stark.mp4"
    fps = 60
    video_out = cv2.VideoWriter(video_filename, fourcc, fps, (width, height))

    # Create the start node and initialize the index counter to keep track of nodes.
    index_counter = 0
    start_node = Node(index_counter, 0, 0, start[0], start[1])
    index_counter += 1

    # Create the end node.
    end_node = Node(0, 0, 0, end[0], end[1])
    
    # Create the open_set. The open_set is where we create a FIFO system for examining nodes.
    open_set = Queue()
    open_set.put(start_node)

    # Create the seen set. The seen set ensures we do not reexamine nodes that have been seen,
    # whether they have already been placed in the open_set of placed in visited (the closed set).
    seen = set()
    seen.add(start)

    # Visited is the closed set. It is a dictionary to keep track of each node's information by connecting
    # it to its index. This allows for easy information backtracking when finding the final path.
    visited = {}
    
    # Create a list of functions of all the types of moves we can execute.
    directions = [move_up, move_down, move_left, move_right,
                    move_up_left, move_up_right, move_down_left, move_down_right]
    
    # Used to store only every 10th frame to the video. Otherwise the video is hours long.
    # Additionally, writing frames takes the most computation for every loop.
    video_frame_counter = 0

    # Draw the start and end points as a magenta and cyan circle, respectively.
    cv2.circle(map, start, sf,(255,0,255), -1)
    cv2.circle(map, end, sf, (255,255,0), -1)
    
    # While the open_set has nodes to explore, keep exploring to find the goal.
    while not open_set.empty():
       # Attempt to get the next node from the queue. If the queue is empty, alert and break. 
        try:
            current_node = open_set.get_nowait()
        except:
            print("Queue is empty!")
            print(open_set.qsize())
            break

        # Add the current node to the nodes visited (closed set).
        visited.update({current_node.Node_Index: current_node})
        
        # Extract the node's position.
        current_x = current_node.Node_x
        current_y = current_node.Node_y

        # Draw this visited/explored node as green on the map.
        map[current_y, current_x] = [0, 100, 0]  # Dark green for explored nodes

        # Increment the video_frame_counter and save a frame if it is the 10th frame.
        video_frame_counter += 1
        if video_frame_counter == 10:
            # Redraw start and end circles.
            cv2.circle(map, start, sf, (255, 0, 255), -1)
            cv2.circle(map, end, sf, (255, 255, 0), -1)
            # Save current map state as a frame in the final video.
            video_out.write(map)
            # Reset the frame counter.
            video_frame_counter = 0
        
        # If the goal has been reached, set the end_node and the current_node and
        # get the final path.
        if goal_check(current_x, current_y, end):
            end_node = current_node
            path = get_final_path(visited, end_node)

            # For each pixel in the path, draw it as white and save a video frame.
            for x, y in path:
                map[y, x] = [255, 255, 255]  # White path overlay
                video_out.write(map)
            # Release the video file.
            video_out.release()
            # Terminate search and return the final map with the path and area explored.
            return map
        
        # For the current node, apply each of the eight move functions and examine
        # the newNode generated from moving in each direction.
        for move in directions:
            # Get newNode from current move.
            newNode = move(current_node)
            # Apply correct index to newNode.
            newNode.Node_Index = index_counter
            
            # If the move is valid (in image and not in obstacle) AND this position has not been seen before (not in open
            #  OR closed list), then add this node to the open_set and the seen set.
            if valid_move(newNode.Node_x, newNode.Node_y, map.shape, obstacles) and (newNode.Node_x, newNode.Node_y) not in seen:
                index_counter += 1
                open_set.put(newNode)
                seen.add((newNode.Node_x, newNode.Node_y))

    # Close the video and alert that no path was found.
    video_out.release()
    print("Path not found!")
    return map

# Backtrack to find the final path
def get_final_path(visited, end_node):
    # Create a list of all indexes in the path.
    path_indexes = []
    current_index = end_node.Node_Index

    # Iterate recursively to find the path by finding each node's parent index whilst
    # appending to the path.
    while current_index != 0:
        path_indexes.append(current_index)
        current_index = visited[current_index].Parent_Node_Index

    path_indexes.append(0)  # Add the initial state index.
    path_indexes.reverse()  # Reverse to get the correct order.
    path_indexes = tuple(path_indexes)
    # Get each x and y position in the path from the indexes.
    path = [(visited[idx].Node_x, visited[idx].Node_y) for idx in path_indexes]

    return path

def main():
    print("Program Start")
    print("Please enter the start and end coordinates.")
    print("Coordinates should be given as integers in units of mm from the bottom left origin.")

    # The scale factor is the resolution of the image for pathing. A scale factor of 5
    # means that every pixel is .2 mm in size. Increase for greater resolution.
    sf = 5

    # Generate and expand the obstacle map.
    obstacle_map = gen_obstacle_map(sf=sf)
    expanded_obstacle_map, obs_map_gray = expand_obstacles(obstacle_map, 2*sf)

    # Prompt the user for the start and end points for planning.
    start_x, start_y = get_point(prompt="start", sf=sf, image=expanded_obstacle_map, obstacles=obs_map_gray)
    end_x, end_y = get_point(prompt="end", sf=sf, image=expanded_obstacle_map, obstacles=obs_map_gray)

    print("Planning Path...")

    # Apply BFS search.
    final_path_image = BFS_search(map=expanded_obstacle_map, obstacles=obs_map_gray, start=(start_x, start_y), end=(end_x, end_y), sf=sf)

    print("Program Finished.")

    # Show the solution.
    cv2.imshow("Map", final_path_image)
    cv2.waitKey(0)
    return

if __name__ == "__main__":
    main()
