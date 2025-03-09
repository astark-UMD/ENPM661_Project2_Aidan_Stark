Libraries / Dependencies Used:
numpy, cv2, heapq, copy, and queue

Video Output Explanation:
The video shows the node exploration and the generation of the optimal path. As green pixels
turn into black pixels, that means those are nodes being explored. Once the goal position is within the green,
the goal state has been found. The program then draws a white line to show the path from the start to the goal.

    Color Key:
        Black = Unexplored Free Space
        Dark Green = Explored Free Space
        Red = Obstacle Space
        Orange = Clearance Space Around Obstacles and Walls
        Magenta = Start Point
        Cyan = End Point
        White = Final Path

How To Operate Program:
1. Run the program via the terminal.
2. The terminal will prompt the user for the start and end coordinates. 
    It will ask for the coordinates as intengers, in units of mm, and with respect
    to the bottom left corner as the origin. Please note that the box is 180 long
    in x and and 50 long in y. If the point given is invalid, it will reprompt the user.
    Suggested Test Point: start_x = 5, start_y = 5 and end_x = 170, end_y = 45
    This will provide a valid path that tests the entire map. 
3. The program will then output "Planning Path". Wait until the program outputs "Program Finished."
4. When "Program Finished." is output to the terminal a window should appear that shows the total
    region explored and the final path. If the user hits any key on the keyboard, the window will close.
    Additionally, a video file should be generated in the same
    directory that the python file is run. It will be named either BFS_Aidan_Stark.mp4 or
    Dijkstra_Aidan_Stark.mp4 depending upon which path planner was run. 

Expected Time to Execute Program:
BFS: ~10-20 seconds for full map paths.
Dijkstra: ~12-22 seconds for full map paths.

NOTE: The resolution of the maps generated is .2 mm per pixel. This is dictated by a value called
sf (scale factor). If desired this can be changed to an
integer value between 1-10 with 1 = 1 mm per pixel and 10 = .1 mm per pixels.
By default, sf = 5. If desired, it can be changed for faster path finding or better resolution maps.
In BFS_Aidan_Stark its value is located on line 409 in main().
In dijkstra_Aidan_Stark its value is located on line 378 in main().
