#Tutorial to use the program:
# 1. Run the program
# 2. Press 'r' to start the robot algorithm to follow the wall
# 3. Press 'q' to exit the program

import dudraw
import math
import matplotlib.pyplot as plt

dudraw.set_canvas_size(800, 800)
dudraw.clear()

def draw_hexagon(center_x, center_y, side_length, rotation):
    x_coords = []
    y_coords = []
    for i in range(6):
        angle = i * math.pi / 3 + math.radians(rotation + 90)  # Rotate the angle by 90 degrees
        x_coords.append(center_x + side_length * math.cos(angle))
        y_coords.append(center_y + side_length * math.sin(angle))
    
    dudraw.set_pen_color(dudraw.RED)
    dudraw.filled_polygon(x_coords, y_coords)

    # Add circles to the left and right of the hexagon
    circle_radius = 0.015
    left_circle_x = center_x + (side_length - 0.02) * math.cos(math.radians(rotation + 90))
    left_circle_y = center_y + (side_length - 0.02) * math.sin(math.radians(rotation + 90))
    right_circle_x = center_x + (side_length - 0.02) * math.cos(math.radians(rotation - 90))
    right_circle_y = center_y + (side_length - 0.02) * math.sin(math.radians(rotation - 90))
    top_circle_x = center_x + (side_length - 0.03) * math.cos(math.radians(rotation))
    top_circle_y = center_y + (side_length - 0.03) * math.sin(math.radians(rotation))
    
    dudraw.set_pen_color(dudraw.BLUE)
    dudraw
    dudraw.filled_circle(left_circle_x, left_circle_y, circle_radius)
    dudraw.filled_circle(right_circle_x, right_circle_y, circle_radius)
    dudraw.filled_circle(top_circle_x, top_circle_y, circle_radius)
    
def draw_maze():
    dudraw.set_pen_color(dudraw.BLACK)
    dudraw.set_pen_width(0.025)
    overlap = 0.01
    dudraw.line(0, 0.1, 0.25 + overlap, 0.1)
    dudraw.line(0.25, 0.1, 0.25, 0.75 + overlap)
    dudraw.line(0.25, 0.75, 0.5 + overlap, 0.5)
    dudraw.line(0.5, 0.5, 0.75 + overlap, 0.5)
    dudraw.line(0.75, 0.5, 0.75, 0.9 + overlap)
    dudraw.line(0.75, 0.9, 1 + overlap, 0.9)
    #add a perpendicular line to the first line segment and the last
    dudraw.line(0.01, 0, 0.01, 0.25)
    dudraw.line(0.99, 0.8, 0.99, 1)


def collision(circle_x, circle_y, lines):
    for line in lines:
        x1, y1, x2, y2 = line
        # Calculate the distance between the circle center and the line segment
        distance = point_line_distance(circle_x, circle_y, x1, y1, x2, y2)
        # If the distance is less than the circle radius, there is a collision
        if distance < 0.01:
            return True
    return False

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


    
overlap = 0
lines = [(0, 0.1, 0.25 + overlap, 0.1), (0.25, 0.1, 0.25, 0.75 + overlap), (0.25, 0.75, 0.5 + overlap, 0.5), (0.5, 0.5, 0.75 + overlap, 0.5), (0.75, 0.5, 0.75, 0.9 + overlap), (0.75, 0.9, 1 + overlap, 0.9)]
goal_lines = [(0.01, 0, 0.01, 0.25), (0.99, 0.8, 0.99, 1)]
rotation = 0
rotation_vel = 2
x_pos = 0.1
y_pos = 0.1
size = 0.07
vel = 0.005
key = ''
move_val = 0
turn_condition= 0
filter = 0
test_values = [0]
time_values = [0]
collision_count = 0

while True:
    dudraw.clear()
    draw_hexagon(x_pos, y_pos, size, rotation) # Draw the robot
    draw_maze() # Draw the maze

    # Check if the user has pressed a key
    if dudraw.has_next_key_typed():
        key = dudraw.next_key_typed()

    # calculate the position of the sensors
    left_circle_x = x_pos + (size - 0.02) * math.cos(math.radians(rotation + 90))
    left_circle_y = y_pos + (size - 0.02) * math.sin(math.radians(rotation + 90))
    right_circle_x = x_pos + (size - 0.02) * math.cos(math.radians(rotation - 90))
    right_circle_y = y_pos + (size - 0.02) * math.sin(math.radians(rotation - 90))
    top_circle_x = x_pos + (size - 0.03) * math.cos(math.radians(rotation))
    top_circle_y = y_pos + (size - 0.03) * math.sin(math.radians(rotation))

    # Set the Sensor conditions for the robot 
    if turn_condition== 0:
        if collision(left_circle_x, left_circle_y, goal_lines) and collision(right_circle_x, right_circle_y, goal_lines):
            print('Collision both')
            rotation += 180
            x_pos += 0.01 * math.cos(math.radians(rotation))
            collision_count+=1
        else:
            if collision(left_circle_x, left_circle_y, lines):
                print('Collision left')
                move_val = 1
            if collision(right_circle_x, right_circle_y, lines):
                print('Collision right')
                move_val = -1
    # End of sensor conditions


    # Drive algorithm for the robot to follow the lines
    # if the front sensor is colliding with a line, the robot moves forward
    # if either side sensor collides with a line, the robot turns in that direction
        # i.e. the turn_condition increases
    # After the robot initiates a turn, it will continue to turn in that direction to straiten itself if the front sensor is not colliding with a line
        # i.e. the turn_condition decrieses
    # if the robot turn condition is increasing after decreasing, the robot will invert the direction of the turn
        # i.e. the move_val will be inverted if the turn_condition is increasing after decreasing
        # this corrects the robot from getting stuck in a loop
        # spesifically occours for obtuse angles in the maze
    if key == 'r':
        if collision(top_circle_x, top_circle_y, lines):
            x_pos += vel * math.cos(math.radians(rotation))
            y_pos += vel * math.sin(math.radians(rotation))
            if turn_condition> 0:
                turn_condition-= 2
            if turn_condition< 0:
                turn_condition= 0
                move_val = 0
        else:
            turn_condition+= 1
            if turn_condition> 100:
                turn_condition= 0
            if move_val == 1:
                rotation += rotation_vel
            elif move_val == -1:
                rotation -= rotation_vel
            else:
                x_pos += vel * math.cos(math.radians(rotation))
                y_pos += vel * math.sin(math.radians(rotation))
                turn_condition= 0
        
        
        # Store the turn_condition value in a list to plot it later
        # Also filter the values for the move condition, 
        # i.e. if the turn_condition is increasing after decreasing then invert the move_val
        filter += 1 # Filter the turn_condition value to avoid noise
        if filter == 5:
            filter = 0
            test_values.append(turn_condition)
            time_values.append(len(test_values))
            print(f"Turn Condition: {test_values[-1]}, move_val: {move_val}, time {time_values[-1]}")
            # Change the Turn condition if turn_condition is increasing again after decreasing
            if len(test_values)>10 and test_values[-3] > test_values[-2] and test_values[-2] < test_values[-1]:
                move_val *= -1
        
        # End of drive algorithm

    
    # Exit the simulation if the user presses 'q'
    # or if the robot collides with the goal n times 
    if key == 'q' or collision_count > 1:
        break

    dudraw.show(10)


plt.figure(figsize=(10, 6))
plt.plot(time_values, test_values)
plt.xlabel('Time')
plt.ylabel('turn_condition Value')
plt.title('turn_condition Value Over Time')
plt.show() 