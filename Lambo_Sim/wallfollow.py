#Tutorial to use the program:
# 1. Run the program
# 2. Press 'w', 'a', 's', 'd' to move the robot up, left, down, and right respectively
# 3. Press 'z' and 'c' to rotate the robot left and right respectively
# 4. Press 'r' to start the robot algorithm to follow the wall
# 5. Press 'q' to exit the program

import dudraw
import math
import matplotlib.pyplot as plt

def draw_hexagon(center_x, center_y, side_length, rotation):
    x_coords = []
    y_coords = []
    for i in range(6):
        angle = i * math.pi / 3 + math.radians(rotation + 90)  # Rotate the angle by 90 degrees
        x_coords.append(center_x + side_length * math.cos(angle))
        y_coords.append(center_y + side_length * math.sin(angle))
    dudraw.set_pen_color(dudraw.RED)
    dudraw.filled_polygon(x_coords, y_coords)

    #add a perpendicular line to the top, left, and right of the hexagon
    line_length = 0.5
    line_x0 = center_x + line_length * math.cos(math.radians(rotation))
    line_y0 = center_y + line_length * math.sin(math.radians(rotation))
    dudraw.set_pen_color(dudraw.BLUE)
    dudraw.set_pen_width(0.01)
    dudraw.line(center_x, center_y, line_x0, line_y0)

def draw_maze():
    dudraw.set_pen_color(dudraw.BLACK)
    dudraw.set_pen_width(0.025)
    dudraw.line(0, 0.25, 0.25, 0.25)
    dudraw.line(0.5, 0.25, 1, 0.25)
    dudraw.line(0, 0.5, 0.5, 0.5)
    dudraw.line(0.75, 0.5, 1, 0.5)
    dudraw.line(0.25, 0.75, 1, 0.75)

def point_line_distance(x, y, x1, y1, x2, y2):
    # Calculate the distance between a point (x, y) and a line segment defined by (x1, y1) and (x2, y2)
    dx = x2 - x1
    dy = y2 - y1
    if dx == 0 and dy == 0:
        # The line segment is a point, return the distance between the point and the circle center
        return math.sqrt((x - x1) ** 2 + (y - y1) ** 2)
    t = ((x - x1) * dx + (y - y1) * dy) / (dx ** 2 + dy ** 2)
    t = max(0, min(1, t))  # Clamp t to the range [0, 1]
    closest_x = x1 + t * dx
    closest_y = y1 + t * dy
    return math.sqrt((x - closest_x) ** 2 + (y - closest_y) ** 2)

def collision(x, y, lines):
    for line in lines:
        x1, y1, x2, y2 = line
        # Calculate the distance between the circle center and the line segment
        distance = point_line_distance(x, y, x1, y1, x2, y2)
        # If the distance is less than the circle radius, there is a collision
        if distance < 0.01:
            return True
    return False

    

dudraw.set_canvas_size(800, 800)
dudraw.clear()

overlap = 0
lines = [(0, 0.25, 0.25, 0.25), (0.5, 0.25, 1, 0.25), (0, 0.5, 0.5, 0.5), (0.75, 0.5, 1, 0.5), (0.25, 0.75, 1, 0.75)]
reversed_lines = lines[::-1]
rotation = 0
x_pos = 0.1
y_pos = 0.1
size = 0.07
vel = 0.005
key = ''
shift_value = 0
front_collision = 0
direction = 0
setup_phase = 0
x_bound = [0,0]
x_positions = []

while True:
    #get the line data for the ultrasonic sensor. "the data where the green circle is"
    line_length = 0.1
    line_x0 = x_pos + line_length * math.cos(math.radians(rotation+90))
    line_y0 = y_pos + line_length * math.sin(math.radians(rotation+90))
    line_ends = [(line_x0, line_y0)]

    dudraw.clear()
    draw_maze()
    draw_hexagon(x_pos, y_pos, size, rotation+90)
    dudraw.set_pen_color(dudraw.GREEN)
    dudraw.filled_circle(line_ends[0][0], line_ends[0][1], 0.01)



    if dudraw.has_next_key_typed():
        key = dudraw.next_key_typed()
    
    #check for collision with the green circle and the maze lines
    for line in lines:
        if collision(line_ends[0][0], line_ends[0][1], [line]):
            front_collision = 1

    # Drive algotithm for the robot to follow the wall
    # to start, the robot will move forward until it detects a wall
    # once it detects a wall, it will move right or left depending on which half of the maze it is in
        # i.e. if x_pos<0.5, the robot will move right, else it will move left
    # if the robot does not detect a wall, it will continue moving left or right before moving forward to adjust its delay
        # this is to prevent the robot from moving forward immediately after not detecting a wall
    # if the robot reaches a bound before detecting a hole, it will invert its direction to go the other way
        # this fixes the issue of keeping the robot in the maze if it reaches a bound before detecting a hole
    if key == 'r':
        #algortim for the robot to follow the wall
        x_positions.append(x_pos)
        if front_collision == 1 or shift_value>0:
            # function to delay the robot from going forward immediately after not detecting a wall
            if front_collision == 1 and shift_value<20:
                shift_value+=1
            else:
                shift_value-=1
            
            # move the robot right or left depending on the direction
            if direction == 1:
                x_pos += vel * math.cos(math.radians(rotation)) 
                y_pos += vel * math.sin(math.radians(rotation))
            if direction == -1:
                x_pos -= vel * math.cos(math.radians(rotation)) 
                y_pos -= vel * math.sin(math.radians(rotation))

            # reset the front collision variable
            front_collision = 0
        else:
            # set the direction of the robot depending on which half of the maze it is in
            if x_pos<0.5:
                direction = 1 # move right if the robot is in the left half of the maze
            else:
                direction = -1 # move left if the robot is in the right half of the maze
            
            # move the robot forward
            y_pos += vel * math.sin(math.radians(rotation+90)) 
            
            
    # drive the robot
    if key == 'w':
        y_pos += vel * math.sin(math.radians(rotation+90)) # move the robot forward
    if key == 's':
        y_pos -= vel * math.sin(math.radians(rotation+90)) # move the robot backward
    if key == 'a':
        x_pos -= vel * math.cos(math.radians(rotation)) # move the robot left
    if key == 'd':
        x_pos += vel * math.cos(math.radians(rotation)) # move the robot right
    if key == 'z':
        rotation +=2 # rotate the robot left
    if key == 'c':
        rotation -=2 # rotate the robot right


    # Exit the loop if the user presses 'q' or the robot reaches the end of the maze
    if key == 'q' or y_pos>0.9:
        break

    dudraw.show(20)

# Plot the x positions
plt.plot(x_positions)
plt.xlabel('Time')
plt.ylabel('X Position')
plt.show()