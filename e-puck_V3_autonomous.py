from controller import Robot, DistanceSensor, Motor, GPS, Compass
import math

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialise sensors
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# Initialise distance sensors
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

# Robot parameters
MAX_SPEED = 6.28

# Target position
TARGET_POSITION = [0.15, -0.24, 0]
GOAL_TOLERANCE = 0.05
MAX_TIME = 900

# Sensor indices
SENSOR_FRONT = 0
SENSOR_LEFT = 2
SENSOR_RIGHT = 3

# Thresholds based on sensor readings (60-70 = open)
FRONT_BLOCKED = 100   # If front > 60, can't go forward
OPEN_SPACE = 50      # If side < 50, it's open

# Start time
start_time = robot.getTime()

def get_sensor_values():
    return [sensor.getValue() for sensor in distance_sensors]

def update_position():
    gps_values = gps.getValues()
    current_position = [gps_values[0], gps_values[1]]
    
    compass_values = compass.getValues()
    current_heading = math.atan2(compass_values[0], compass_values[1])
    if current_heading < 0:
        current_heading += 2 * math.pi
    
    return current_position, current_heading

def calculate_target_info(current_position):
    dx = TARGET_POSITION[0] - current_position[0]
    dy = TARGET_POSITION[1] - current_position[1]
    
    distance = math.sqrt(dx*dx + dy*dy)
    target_angle = math.atan2(dy, dx)
    if target_angle < 0:
        target_angle += 2 * math.pi
    
    return distance, target_angle

def get_heading_error(current_heading, target_angle):
    error = target_angle - current_heading
    while error > math.pi:
        error -= 2 * math.pi
    while error < -math.pi:
        error += 2 * math.pi
    return math.degrees(error)

print("Starting SIMPLE Navigation...")
print(f"Target: ({TARGET_POSITION[0]:.3f}, {TARGET_POSITION[1]:.3f})")
print("Rule: Always move forward, turn only when absolutely necessary")

# Initial movement 
left_motor.setVelocity(MAX_SPEED * 0.8)
right_motor.setVelocity(MAX_SPEED * 0.8)
for i in range(30):
    robot.step(timestep)

last_print_time = 0
last_turn_time = 0
turn_in_progress = False
turn_direction = ""

while robot.step(timestep) != -1:
    # Check time
    elapsed = robot.getTime() - start_time
    if elapsed > MAX_TIME:
        print(f"\n⏰ Time limit exceeded ({MAX_TIME}s)")
        break
    
    # Update position
    current_position, current_heading = update_position()
    sensor_values = get_sensor_values()
    
    front = sensor_values[SENSOR_FRONT]
    left = sensor_values[SENSOR_LEFT]
    right = sensor_values[SENSOR_RIGHT]
    
    # Calculate target info
    target_distance, target_angle = calculate_target_info(current_position)
    heading_error = get_heading_error(current_heading, target_angle)
    
    # Check if target reached
    if target_distance < GOAL_TOLERANCE:
        print(f"\n TARGET REACHED SUCCESSFULLY. Distance covered: {target_distance:.3f}metres")
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        break
    
    # Print status
    if robot.getTime() - last_print_time > 0.5:
        print(f"Time elapsed:{elapsed:4.1f}s | Current position:({current_position[0]:5.2f},{current_position[1]:5.2f}) | "
              f"Target Proximity:{target_distance:5.3f}m | Err:{heading_error:5.1f}° | "
              f"Front Sensor{front:3.0f} Left Sensor{left:3.0f} Right Sensor{right:3.0f}")
        last_print_time = robot.getTime()
    
    # If turning, continue for fixed time
    if turn_in_progress:
        current_time = robot.getTime()
        if current_time - last_turn_time < 0.4:  # Turn for 0.4 seconds
            turn_speed = MAX_SPEED * 0.6
            if turn_direction == "RIGHT":
                left_motor.setVelocity(turn_speed)
                right_motor.setVelocity(-turn_speed)
            else:  # LEFT
                left_motor.setVelocity(-turn_speed)
                right_motor.setVelocity(turn_speed)
            continue
        else:
            # Turn complete
            turn_in_progress = False
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            robot.step(int(timestep * 2))
    
    # Rule 1: If front is blocked, MUST turn
    if front > FRONT_BLOCKED:
        print(f"Front blocked ({front}), turning...")
        
        # Choose direction with more open space
        if left < right:  # Left is more open
            turn_in_progress = True
            turn_direction = "LEFT"
            last_turn_time = robot.getTime()
            print(f"  → Turning LEFT (L:{left} vs R:{right})")
        else:  # Right is more open or equal
            turn_in_progress = True
            turn_direction = "RIGHT"
            last_turn_time = robot.getTime()
            print(f"  → Turning RIGHT (L:{left} vs R:{right})")
        continue
    
    # Rule 2: DEFAULT - Always go forward unless there's a good reason not to
    # Just go forward at good speed
    base_speed = MAX_SPEED * 0.8
    left_motor.setVelocity(base_speed)
    right_motor.setVelocity(base_speed)

# Final output
left_motor.setVelocity(0)
right_motor.setVelocity(0)
total_time = robot.getTime() - start_time

print("\n" + "="*60)
print("MAZE NAVIGATION COMPLETE")
print(f"Total time: {total_time:.2f}s")
print(f"Final position: ({current_position[0]:.3f}, {current_position[1]:.3f})")
print(f"Target position: ({TARGET_POSITION[0]:.3f}, {TARGET_POSITION[1]:.3f})")
print(f"Final distance to target: {target_distance:.3f}m")

if target_distance < GOAL_TOLERANCE:
    print("\n✅ SUCCESS! Target reached")
else:
    print("\n❌ Target not reached")

print("="*60)