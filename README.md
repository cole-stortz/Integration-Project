# Lambo Rambo Robot Algorithms

## Overview

This project implements autonomous navigation and object manipulation algorithms for the Lambo Rambo robot. The codebase includes sophisticated line-following algorithms, wall-following navigation, and real-time simulation environments for algorithm development and testing.

## Project Contributors

- **Cole Stortz** (Main Developer)
- **Duncan Wood** (Secondary Developer)
- **Yannis Cassis**, **Kian Roseborough**, **Ala Assaf**

**Development Period:** April 8, 2024 - June 5, 2024

## Core Algorithms

### 1. State Machine Control Algorithm

The robot operates using a finite state machine with 4 primary states:

```
State 0: Object Detection & Pickup
State 1: Line Following (Forward)
State 2: Object Placement
State 3: Line Following (Reverse/Return)
```

#### State Transition Logic
```cpp
void loop() {
  if(robot_case == 0) {
    pickUpBox();           // Object detection and pickup
  } else if(robot_case == 1) {
    followLine();          // Forward line following
  } else if(robot_case == 2) {
    placeBox();            // Object placement with routing
  } else if(robot_case == 3) {
    followLineReverse();   // Return journey
  }
}
```

### 2. Advanced Line Following Algorithm

#### Differential Steering with Multi-Sensor Input
The line following algorithm uses a 4-sensor array for precise path tracking:

```cpp
void followLine() {
  if (colorsensorValues[2] == LINE_COLOR && 
      colorsensorValues[3] == BLACK && 
      colorsensorValues[0] == BLACK) {
    moveMotorsTurnLeft();   // Left sensor detects line
  } 
  else if (colorsensorValues[0] == LINE_COLOR && 
           colorsensorValues[3] == BLACK && 
           colorsensorValues[2] == BLACK) {
    moveMotorsTurnRight();  // Right sensor detects line
  } 
  else if (colorsensorValues[3] == LINE_COLOR) {
    moveMotorsForward();    // Center sensor on line
  } 
  else if (colorsensorValues[2] == LINE_COLOR && 
           colorsensorValues[0] == LINE_COLOR) {
    robot_case = 2;         // Junction detected
  }
}
```

#### Key Algorithm Features:
- **Predictive Steering:** Uses outer sensors to detect line deviation before center sensor loses contact
- **Junction Detection:** Simultaneous activation of both outer sensors indicates a fork
- **Centering Behavior:** Continues turning until center sensor reacquires the line
- **Color-Adaptive:** Follows different colored lines based on object color

### 3. Wall-Following Algorithm (Alternative Navigation)

#### Coordinate-Based Navigation System
The wall-following algorithm uses a virtual coordinate system:

```python
# Virtual maze dimensions: 35 units width × 140 units height
if distance > 20:  # No wall detected
    if delayCondition == 1:  # Delay after finding opening
        delay(500)
        # Adjust position based on previous movement direction
        if move_val == 1:
            x_floating = x_floating + 5
        elif move_val == -1:
            x_floating = x_floating - 5
    y_pos += 1  # Move forward
    x_pos = x_floating  # Store stable position
else:  # Wall detected
    if x_pos < 18:  # Left half of maze
        if x_floating < 34:  # Not at right boundary
            moveMotorsRight()
            x_floating += 1
    else:  # Right half of maze
        if x_floating > 0:  # Not at left boundary
            moveMotorsLeft()
            x_floating -= 1
```

#### Algorithm Logic:
1. **Forward Movement:** Continue until wall detection
2. **Lateral Scanning:** Move left/right based on maze position
3. **Boundary Detection:** Prevent infinite scanning at maze edges
4. **Gap Detection:** Delay before forward movement after finding opening
5. **Position Tracking:** Maintain virtual coordinates for decision making

### 4. Object Manipulation Algorithm

#### Adaptive Gripper Control with Force Feedback
```cpp
void closeGripper() {
  if (gripperPosition > minGripperPosition && forceValue < forceThreshold) {
    gripperPosition--;
    Gripper.write(gripperPosition);
    
    // Size classification based on grip distance
    if (gripperPosition < 75) {
      BOXSIZE = 0;  // Small object
    } else {
      BOXSIZE = 1;  // Large object
    }
  } else {
    box_detected = 1;  // Trigger next phase
    moveMotorsForward();
  }
}
```

#### Decision Tree for Object Routing
```cpp
void placeBox() {
  if (BOXSIZE == 1) {        // Large object
    moveMotorsLeft();         // Go to left fork
    delay(650);
    // ... placement sequence
  }
  if (BOXSIZE == 0) {        // Small object
    moveMotorsRight();        // Go to right fork
    delay(650);
    // ... placement sequence
  }
}
```

## Simulation Algorithms

### 1. Line Following Simulator

#### Collision-Based Navigation
```python
def collision(circle_x, circle_y, lines):
    for line in lines:
        distance = point_line_distance(circle_x, circle_y, x1, y1, x2, y2)
        if distance < 0.01:
            return True
    return False
```

#### Adaptive Turn Control Algorithm
```python
if collision(top_circle_x, top_circle_y, lines):
    # Move forward when front sensor detects line
    x_pos += vel * math.cos(math.radians(rotation))
    y_pos += vel * math.sin(math.radians(rotation))
    
    # Decrease turn condition (centering behavior)
    if turn_condition > 0:
        turn_condition -= 2
else:
    # Turn based on side sensor detection
    turn_condition += 1
    if move_val == 1:
        rotation += rotation_vel  # Turn right
    elif move_val == -1:
        rotation -= rotation_vel  # Turn left
```

#### Self-Correction Algorithm
```python
# Prevent oscillation at obtuse angles
if len(test_values) > 10 and test_values[-3] > test_values[-2] and test_values[-2] < test_values[-1]:
    move_val *= -1  # Invert turn direction
```

### 2. Wall Following Simulator

#### Boundary-Aware Navigation
```python
if front_collision == 1 or shift_value > 0:
    # Delay mechanism to prevent immediate forward movement
    if front_collision == 1 and shift_value < 20:
        shift_value += 1
    else:
        shift_value -= 1
    
    # Move based on determined direction
    if direction == 1:
        x_pos += vel * math.cos(math.radians(rotation))
    if direction == -1:
        x_pos -= vel * math.cos(math.radians(rotation))
else:
    # Set movement direction based on position
    if x_pos < 0.5:
        direction = 1  # Move right in left half
    else:
        direction = -1  # Move left in right half
    
    y_pos += vel * math.sin(math.radians(rotation + 90))
```

## Algorithm Performance Features

### Real-Time Adaptability
- **Dynamic Speed Control:** PWM values adjust based on turning requirements
- **Sensor Fusion:** Multiple sensor inputs provide robust navigation
- **Error Recovery:** Self-correction mechanisms prevent infinite loops

### Optimization Techniques
- **Predictive Control:** Uses leading sensors to anticipate path changes
- **State Minimization:** Reduces computational overhead with efficient state transitions
- **Calibration Independence:** Algorithms adapt to sensor variations through mapping functions

### Robustness Features
- **Boundary Checking:** Prevents navigation outside defined areas
- **Timeout Mechanisms:** Prevents infinite loops in edge cases
- **Graceful Degradation:** Continues operation with partial sensor failures

## Algorithm Testing & Validation

### Simulation-First Development
1. **Algorithm Prototyping:** Python simulators allow rapid iteration
2. **Parameter Tuning:** Visual feedback enables real-time optimization
3. **Edge Case Testing:** Simulated environments test boundary conditions
4. **Performance Metrics:** Data logging tracks algorithm effectiveness

### Hardware Validation
- **Serial Debugging:** Real-time algorithm state monitoring
- **Sensor Visualization:** Color and distance readings for algorithm verification
- **Performance Logging:** Turn conditions and movement decisions tracked

## Key Algorithmic Innovations

1. **Multi-Phase State Machine:** Seamless transitions between complex behaviors
2. **Adaptive Line Following:** Color-agnostic navigation with dynamic line switching
3. **Force-Feedback Object Sizing:** Hardware integration for intelligent decision making
4. **Virtual Coordinate Navigation:** Position-aware maze solving without external localization
5. **Predictive Path Correction:** Prevents oscillation through historical analysis

## File Structure

```
├── Lambo_LineFollow.ino          # Main state machine & line following
├── Lambo_LineFollow_v2.ino       # Wall-following navigation algorithm
├── line_follow_simulator.py      # Collision-based navigation simulation
├── manual_control_simulator.py   # Interactive algorithm testing
└── README.md                     # Algorithm documentation
```

---

*This project demonstrates advanced autonomous navigation algorithms including predictive control, sensor fusion, state machine design, and real-time adaptation mechanisms.*
