from controller import Robot, DistanceSensor, Motor, GPS, Compass
import math

robot = Robot()

timestep = int(robot.getBasicTimeStep())

# Initialise GPS and Compass sensors for precise navigation
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# Initialise distance sensors for behaviour-based navigation
ds_names = ["ds1", "ds2", "ds3", "ds4"]
distance_sensors = []
for name in ds_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    distance_sensors.append(sensor)

# Initialise motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Robot physical parameters
MAX_SPEED = 6.28
WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.052

# Navigation states
STATE_GO_STRAIGHT = 0
STATE_TURN_RIGHT = 1
STATE_TURN_LEFT = 2
STATE_TURN_AROUND = 3
STATE_STOP = 4

# Target position
TARGET_POSITION = [0.23, -0.22, 0]
GOAL_TOLERANCE = 0.025

# Robot state variables
current_state = STATE_GO_STRAIGHT

# GPS-based position tracking
current_gps_position = [0, 0, 0]
compass_heading = 0

# Turn timing
turn_start_time = 0
is_turning = False
turn_direction = 0
target_turn_angle = 0

# Flags
goal_reached = False

# Performance tracking
navigation_start_time = robot.getTime()

# Sensor indices
SENSOR_FRONT = 0
SENSOR_BACK = 1 
SENSOR_LEFT = 2
SENSOR_RIGHT = 3

# SIMPLE THRESHOLDS
WALL_CLOSE = 1000   # Close to wall
WALL_FAR = 300      # Far from wall

def get_sensor_values():
    return [sensor.getValue() for sensor in distance_sensors]

def update_gps_compass():
    global current_gps_position, compass_heading
    gps_values = gps.getValues()
    current_gps_position = gps_values.copy()
    compass_values = compass.getValues()
    compass_heading = math.atan2(compass_values[0], compass_values[1])

def calculate_target_heading():
    """Calculate the heading angle toward the target"""
    dx = TARGET_POSITION[0] - current_gps_position[0]
    dy = TARGET_POSITION[1] - current_gps_position[1]
    return math.atan2(dy, dx)

def get_heading_error():
    """Calculate the difference between current heading and target heading"""
    target_heading = calculate_target_heading()
    error = target_heading - compass_heading
    
    # Normalise to [-π, π]
    while error > math.pi:
        error -= 2 * math.pi
    while error < -math.pi:
        error += 2 * math.pi
    
    return error

def simple_decision(ds_values):
    """SIMPLE RULE: Go toward target, turn to most open space if blocked"""
    global current_state, is_turning, turn_start_time, turn_direction, target_turn_angle
    
    if is_turning:
        return
    
    # Get sensor values
    front = ds_values[SENSOR_FRONT]
    left = ds_values[SENSOR_LEFT]
    right = ds_values[SENSOR_RIGHT]
    
    # Get heading to target
    heading_error = get_heading_error()
    heading_error_deg = math.degrees(heading_error)
    
    # Calculate distance to target
    distance_to_target = math.sqrt(
        (current_gps_position[0] - TARGET_POSITION[0])**2 + 
        (current_gps_position[1] - TARGET_POSITION[1])**2
    )
    
    # RULE 1: If very close to target, go straight
    if distance_to_target < 0.1:
        if front < WALL_CLOSE:
            current_state = STATE_GO_STRAIGHT
            is_turning = False
            return
    
    # RULE 2: If aligned with target (±20°) and front is open, go straight
    if abs(heading_error_deg) < 20 and front < WALL_CLOSE:
        current_state = STATE_GO_STRAIGHT
        is_turning = False
        return
    
    # RULE 3: If front is blocked, turn to side with MOST OPEN SPACE
    if front > WALL_CLOSE:
        # Choose direction with lowest sensor value (most open)
        if left < right:  # More open on left
            current_state = STATE_TURN_LEFT
            turn_direction = -1
            target_turn_angle = 90
        else:  # More open on right
            current_state = STATE_TURN_RIGHT
            turn_direction = 1
            target_turn_angle = 90
        
        is_turning = True
        turn_start_time = robot.getTime()
        return
    
    # RULE 4: Not aligned with target, turn toward it using most open space
    # Check which direction has more open space
    left_more_open = left < right
    
    if heading_error_deg > 0:  # Target is to the left
        if left_more_open and left < WALL_CLOSE:
            # Turn left toward target
            current_state = STATE_TURN_LEFT
            turn_direction = -1
            target_turn_angle = min(90, abs(heading_error_deg))
        elif right < WALL_CLOSE:
            # Can't turn left, turn right instead
            current_state = STATE_TURN_RIGHT
            turn_direction = 1
            target_turn_angle = min(90, abs(heading_error_deg))
        else:
            # Both sides blocked, go straight if possible
            if front < WALL_CLOSE:
                current_state = STATE_GO_STRAIGHT
                is_turning = False
            else:
                # Turn around
                current_state = STATE_TURN_AROUND
                turn_direction = 1
                target_turn_angle = 180
    else:  # Target is to the right
        if not left_more_open and right < WALL_CLOSE:
            # Turn right toward target
            current_state = STATE_TURN_RIGHT
            turn_direction = 1
            target_turn_angle = min(90, abs(heading_error_deg))
        elif left < WALL_CLOSE:
            # Can't turn right, turn left instead
            current_state = STATE_TURN_LEFT
            turn_direction = -1
            target_turn_angle = min(90, abs(heading_error_deg))
        else:
            # Both sides blocked, go straight if possible
            if front < WALL_CLOSE:
                current_state = STATE_GO_STRAIGHT
                is_turning = False
            else:
                # Turn around
                current_state = STATE_TURN_AROUND
                turn_direction = 1
                target_turn_angle = 180
    
    if current_state in [STATE_TURN_LEFT, STATE_TURN_RIGHT, STATE_TURN_AROUND]:
        is_turning = True
        turn_start_time = robot.getTime()

def execute_current_state():
    global is_turning, current_state
    
    if current_state == STATE_GO_STRAIGHT:
        # Fast forward movement
        speed = MAX_SPEED * 0.9
        left_motor.setVelocity(speed)
        right_motor.setVelocity(speed)
        
    elif current_state in [STATE_TURN_RIGHT, STATE_TURN_LEFT, STATE_TURN_AROUND]:
        if is_turning:
            turn_duration = calculate_turn_duration(target_turn_angle)
            
            current_time = robot.getTime()
            if current_time - turn_start_time < turn_duration:
                turn_speed = MAX_SPEED * 0.7
                if turn_direction > 0:
                    left_motor.setVelocity(turn_speed)
                    right_motor.setVelocity(-turn_speed)
                else:
                    left_motor.setVelocity(-turn_speed)
                    right_motor.setVelocity(turn_speed)
            else:
                is_turning = False
                left_motor.setVelocity(0)
                right_motor.setVelocity(0)
                # Brief pause
                robot.step(int(timestep * 2))
                current_state = STATE_GO_STRAIGHT
                
    elif current_state == STATE_STOP:
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)

def calculate_turn_duration(angle_degrees):
    angle_rad = math.radians(angle_degrees)
    wheel_distance = angle_rad * (AXLE_LENGTH / 2)
    return wheel_distance / (MAX_SPEED * 0.7 * WHEEL_RADIUS)

def check_target_reached():
    global goal_reached
    
    distance_to_target = math.sqrt(
        (current_gps_position[0] - TARGET_POSITION[0])**2 + 
        (current_gps_position[1] - TARGET_POSITION[1])**2
    )
    
    if distance_to_target <= GOAL_TOLERANCE:
        goal_reached = True
        print(f"*** TARGET REACHED! ***")
        print(f"Final position: ({current_gps_position[0]:.3f}, {current_gps_position[1]:.3f})")
        print(f"Target position: ({TARGET_POSITION[0]:.3f}, {TARGET_POSITION[1]:.3f})")
        print(f"Distance to target: {distance_to_target:.3f} meters")
        return True
    
    return False

# Main control loop
print("Starting SIMPLE Maze Navigation...")
print(f"Target: {TARGET_POSITION}")
print("Strategy: Go toward target, turn to most open space if blocked")

# Initial movement
print("Starting initial movement...")
left_motor.setVelocity(MAX_SPEED * 0.8)
right_motor.setVelocity(MAX_SPEED * 0.8)
for i in range(25):
    robot.step(timestep)

last_status_time = 0

while robot.step(timestep) != -1 and not goal_reached:
    ds_values = get_sensor_values()
    update_gps_compass()
    
    if check_target_reached():
        break
    
    # Check timeout
    if robot.getTime() - navigation_start_time > 500:
        print("TIME LIMIT EXCEEDED - 1 minute")
        break
    
    # Make simple decision
    simple_decision(ds_values)
    execute_current_state()
    
    # Status updates
    current_time = robot.getTime()
    if current_time - last_status_time > 0.5:
        distance_to_target = math.sqrt(
            (current_gps_position[0] - TARGET_POSITION[0])**2 + 
            (current_gps_position[1] - TARGET_POSITION[1])**2
        )
        heading_error = math.degrees(get_heading_error())
        elapsed_time = current_time - navigation_start_time
        
        # Get sensor values for display
        front = ds_values[SENSOR_FRONT]
        left = ds_values[SENSOR_LEFT]
        right = ds_values[SENSOR_RIGHT]
        
        state_names = ["STRAIGHT", "TURN_RIGHT", "TURN_LEFT", "TURN_AROUND", "STOP"]
        
        print(f"T:{elapsed_time:4.1f}s | Pos:({current_gps_position[0]:5.2f},{current_gps_position[1]:5.2f}) | "
              f"Target:{distance_to_target:5.3f}m | Err:{heading_error:5.1f}° | "
              f"Sensors: F{front:4.0f} L{left:4.0f} R{right:4.0f} | "
              f"State:{state_names[current_state]}")
        
        last_status_time = current_time

# Final output
left_motor.setVelocity(0)
right_motor.setVelocity(0)
total_navigation_time = robot.getTime() - navigation_start_time

print("\n" + "="*50)
if goal_reached:
    print("*** TARGET REACHED! ***")
    if total_navigation_time < 30:
        print("*** EXCELLENT TIME! Under 30 seconds! ***")
    elif total_navigation_time < 45:
        print("*** GOOD TIME! Under 45 seconds! ***")
else:
    print("*** NAVIGATION STOPPED ***")

print(f"Total time: {total_navigation_time:.2f}s")
print(f"Final position: ({current_gps_position[0]:.3f}, {current_gps_position[1]:.3f})")
print(f"Target position: ({TARGET_POSITION[0]:.3f}, {TARGET_POSITION[1]:.3f})")
final_distance = math.sqrt((current_gps_position[0] - TARGET_POSITION[0])**2 + 
                          (current_gps_position[1] - TARGET_POSITION[1])**2)
print(f"Final distance to target: {final_distance:.3f}m")

if goal_reached:
    print("✅ SUCCESS!")
else:
    print("❌ FAILED - Target not reached")
    
print("="*50)