from controller import Robot, DistanceSensor, Motor, GPS, Compass
import math
import heapq

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
WHEEL_RADIUS = 0.0205
AXLE_LENGTH = 0.052

# Target position
TARGET_POSITION = [0.75, -0.75, 0]
GOAL_TOLERANCE = 0.15

# Maze mapping parameters
GRID_SIZE = 9  # 9x9 grid for detailed mapping
CELL_SIZE = 0.25  # 25cm per cell for 2m x 2m maze
MAZE_ORIGIN = [-1.0, 1.0]  # Top-left corner of maze

# Initialise maze map (9x9 grid)
maze_map = [[1 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
visited = [[False for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]

# Navigation states
STATE_EXPLORE = 0
STATE_MOVE_TO_TARGET = 1
STATE_AVOID_OBSTACLE = 2
STATE_STOP = 3

# Robot state
current_state = STATE_EXPLORE
robot_grid_x, robot_grid_y = 4, 4  # Start in center
path_to_target = []
current_path_index = 0
exploration_complete = False

# Performance optimisation
last_map_update = 0
MAP_UPDATE_INTERVAL = 0.2  # Update map every 200ms
navigation_start_time = robot.getTime()

# Sensor indices
SENSOR_FRONT = 0
SENSOR_RIGHT = 3
SENSOR_LEFT = 2
SENSOR_BACK = 1

# Sensor thresholds
WALL_THRESHOLD = 80
EMERGENCY_THRESHOLD = 200

def get_sensor_values():
    return [sensor.getValue() for sensor in distance_sensors]

def update_gps_compass():
    gps_values = gps.getValues()
    compass_values = compass.getValues()
    return gps_values, math.atan2(compass_values[0], compass_values[1])

def gps_to_grid(x, y):
    """Convert GPS coordinates to grid coordinates"""
    grid_x = int((x - MAZE_ORIGIN[0]) / CELL_SIZE)
    grid_y = int((MAZE_ORIGIN[1] - y) / CELL_SIZE)
    return max(0, min(GRID_SIZE-1, grid_x)), max(0, min(GRID_SIZE-1, grid_y))

def grid_to_gps(grid_x, grid_y):
    """Convert grid coordinates to GPS coordinates"""
    x = MAZE_ORIGIN[0] + grid_x * CELL_SIZE + CELL_SIZE/2
    y = MAZE_ORIGIN[1] - grid_y * CELL_SIZE - CELL_SIZE/2
    return x, y

def update_maze_map(ds_values, current_x, current_y, heading):
    """Update 4-point map around robot based on sensor readings"""
    global maze_map, visited
    
    # Mark current cell as visited and open
    if 0 <= current_x < GRID_SIZE and 0 <= current_y < GRID_SIZE:
        maze_map[current_y][current_x] = 0
        visited[current_y][current_x] = True
    
    # Update adjacent cells based on sensor readings and heading
    front_wall = ds_values[SENSOR_FRONT] > WALL_THRESHOLD
    right_wall = ds_values[SENSOR_RIGHT] > WALL_THRESHOLD
    left_wall = ds_values[SENSOR_LEFT] > WALL_THRESHOLD
    back_wall = ds_values[SENSOR_BACK] > WALL_THRESHOLD
    
    # Convert heading to cardinal direction (simplified)
    if -0.785 <= heading <= 0.785:  # East
        if front_wall and current_x + 1 < GRID_SIZE: maze_map[current_y][current_x + 1] = 1
        if right_wall and current_y + 1 < GRID_SIZE: maze_map[current_y + 1][current_x] = 1
        if left_wall and current_y - 1 >= 0: maze_map[current_y - 1][current_x] = 1
        if back_wall and current_x - 1 >= 0: maze_map[current_y][current_x - 1] = 1
    elif 0.785 < heading <= 2.356:  # North
        if front_wall and current_y - 1 >= 0: maze_map[current_y - 1][current_x] = 1
        if right_wall and current_x - 1 >= 0: maze_map[current_y][current_x - 1] = 1
        if left_wall and current_x + 1 < GRID_SIZE: maze_map[current_y][current_x + 1] = 1
        if back_wall and current_y + 1 < GRID_SIZE: maze_map[current_y + 1][current_x] = 1
    elif 2.356 < heading <= 3.927:  # West
        if front_wall and current_x - 1 >= 0: maze_map[current_y][current_x - 1] = 1
        if right_wall and current_y - 1 >= 0: maze_map[current_y - 1][current_x] = 1
        if left_wall and current_y + 1 < GRID_SIZE: maze_map[current_y + 1][current_x] = 1
        if back_wall and current_x + 1 < GRID_SIZE: maze_map[current_y][current_x + 1] = 1
    else:  # South
        if front_wall and current_y + 1 < GRID_SIZE: maze_map[current_y + 1][current_x] = 1
        if right_wall and current_x + 1 < GRID_SIZE: maze_map[current_y][current_x + 1] = 1
        if left_wall and current_x - 1 >= 0: maze_map[current_y][current_x - 1] = 1
        if back_wall and current_y - 1 >= 0: maze_map[current_y - 1][current_x] = 1

def heuristic(a, b):
    """Manhattan distance heuristic for A* pathfinding"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_pathfinding(start, goal, maze):
    """A* pathfinding algorithm to find shortest path through zeros"""
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    
    while open_set:
        current = heapq.heappop(open_set)[1]
        
        if current == goal:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path
        
        # Check all 4 neighbors
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            neighbor = (current[0] + dx, current[1] + dy)
            
            # Check if neighbor is within bounds and is open space (0)
            if (0 <= neighbor[0] < GRID_SIZE and 0 <= neighbor[1] < GRID_SIZE and 
                maze[neighbor[1]][neighbor[0]] == 0):
                
                tentative_g_score = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return []  # No path found

def find_nearest_unvisited(current_pos, maze, visited_map):
    """Find nearest unvisited cell that's accessible"""
    queue = [current_pos]
    visited_temp = [[False for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    visited_temp[current_pos[1]][current_pos[0]] = True
    
    while queue:
        x, y = queue.pop(0)
        
        if not visited_map[y][x] and maze[y][x] == 0:
            return (x, y)
        
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            nx, ny = x + dx, y + dy
            if (0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and 
                not visited_temp[ny][nx] and maze[ny][nx] == 0):
                visited_temp[ny][nx] = True
                queue.append((nx, ny))
    
    return None  # All accessible cells visited

def get_move_direction(current_pos, next_pos, current_heading):
    """Calculate required turn to face next position"""
    dx = next_pos[0] - current_pos[0]
    dy = next_pos[1] - current_pos[1]
    
    # Convert grid movement to target heading
    if dx == 1: target_heading = 0  # East
    elif dx == -1: target_heading = math.pi  # West
    elif dy == 1: target_heading = -math.pi/2  # South
    else: target_heading = math.pi/2  # North
    
    # Calculate heading error
    error = target_heading - current_heading
    while error > math.pi: error -= 2 * math.pi
    while error < -math.pi: error += 2 * math.pi
    
    return error

def move_to_position(target_gps, current_gps, current_heading):
    """Move toward target GPS position"""
    dx = target_gps[0] - current_gps[0]
    dy = target_gps[1] - current_gps[1]
    distance = math.sqrt(dx*dx + dy*dy)
    
    if distance < 0.1:  # Reached position
        return 0, 0, True
    
    # Calculate target heading
    target_heading = math.atan2(dy, dx)
    heading_error = target_heading - current_heading
    
    # Normalise heading error
    while heading_error > math.pi: heading_error -= 2 * math.pi
    while heading_error < -math.pi: heading_error += 2 * math.pi
    
    # Adjust speed based on alignment
    if abs(heading_error) > 0.3:  # Need to turn
        if heading_error > 0:
            return -MAX_SPEED * 0.6, MAX_SPEED * 0.6, False
        else:
            return MAX_SPEED * 0.6, -MAX_SPEED * 0.6, False
    else:  # Move forward
        speed = MAX_SPEED * min(1.0, distance / 0.3)
        return speed, speed, False

def print_maze_map():
    """Print the current maze map for debugging"""
    print("Current Maze Map (1=wall, 0=open, X=robot):")
    for y in range(GRID_SIZE):
        row = ""
        for x in range(GRID_SIZE):
            if x == robot_grid_x and y == robot_grid_y:
                row += "X "
            else:
                row += f"{maze_map[y][x]} "
        print(row)
    print()

# Main control loop
print("Starting Optimized Maze Navigation with 4-Point Mapping...")
print("Target: (0.75, -0.75)")
print(f"Grid Size: {GRID_SIZE}x{GRID_SIZE}, Cell Size: {CELL_SIZE}m")

# Convert target to grid coordinates
target_grid_x, target_grid_y = gps_to_grid(TARGET_POSITION[0], TARGET_POSITION[1])
print(f"Target Grid: ({target_grid_x}, {target_grid_y})")

for i in range(10):
    robot.step(timestep)

while robot.step(timestep) != -1:
    current_time = robot.getTime()
    
    # Check timeout (2 minutes)
    if current_time - navigation_start_time > 120:
        print("TIME LIMIT EXCEEDED (2 minutes)")
        break
    
    # Get sensor data
    ds_values = get_sensor_values()
    gps_position, compass_heading = update_gps_compass()
    
    # Update grid position
    robot_grid_x, robot_grid_y = gps_to_grid(gps_position[0], gps_position[1])
    
    # Update maze map periodically
    if current_time - last_map_update > MAP_UPDATE_INTERVAL:
        update_maze_map(ds_values, robot_grid_x, robot_grid_y, compass_heading)
        last_map_update = current_time
    
    # Check if target reached
    distance_to_target = math.sqrt(
        (gps_position[0] - TARGET_POSITION[0])**2 + 
        (gps_position[1] - TARGET_POSITION[1])**2
    )
    
    if distance_to_target <= GOAL_TOLERANCE:
        print("*** TARGET REACHED! ***")
        left_motor.setVelocity(0)
        right_motor.setVelocity(0)
        break
    
    # Emergency stop for close obstacles
    if ds_values[SENSOR_FRONT] > EMERGENCY_THRESHOLD:
        left_motor.setVelocity(-MAX_SPEED * 0.3)
        right_motor.setVelocity(-MAX_SPEED * 0.3)
        robot.step(timestep)
        continue
    
    # State machine for navigation
    if current_state == STATE_EXPLORE:
        if not path_to_target or current_path_index >= len(path_to_target):
            # Find path to target or explore
            path_to_target = a_star_pathfinding(
                (robot_grid_x, robot_grid_y), 
                (target_grid_x, target_grid_y), 
                maze_map
            )
            
            if path_to_target:
                print(f"Path to target found! Length: {len(path_to_target)}")
                current_state = STATE_MOVE_TO_TARGET
                current_path_index = 0
            else:
                # Explore nearest unvisited area
                next_explore = find_nearest_unvisited(
                    (robot_grid_x, robot_grid_y), maze_map, visited
                )
                if next_explore:
                    path_to_target = a_star_pathfinding(
                        (robot_grid_x, robot_grid_y), next_explore, maze_map
                    )
                    if path_to_target:
                        current_path_index = 0
                        print(f"Exploring to {next_explore}")
                    else:
                        # Random movement if stuck
                        left_motor.setVelocity(MAX_SPEED * 0.7)
                        right_motor.setVelocity(MAX_SPEED * 0.3)
                else:
                    exploration_complete = True
                    current_state = STATE_MOVE_TO_TARGET
    
    if current_state == STATE_MOVE_TO_TARGET and path_to_target:
        if current_path_index < len(path_to_target):
            next_grid_pos = path_to_target[current_path_index]
            next_gps_pos = grid_to_gps(next_grid_pos[0], next_grid_pos[1])
            
            left_speed, right_speed, reached = move_to_position(
                next_gps_pos, gps_position, compass_heading
            )
            
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)
            
            if reached:
                current_path_index += 1
                # Check if we reached the final target
                if (next_grid_pos[0] == target_grid_x and 
                    next_grid_pos[1] == target_grid_y):
                    current_state = STATE_STOP
        else:
            current_state = STATE_EXPLORE
    
    # Print status every 5 seconds
    if current_time % 5.0 < timestep/1000.0:
        visited_count = sum(sum(row) for row in visited)
        print(f"Time: {current_time:.1f}s | Pos: ({gps_position[0]:.2f}, {gps_position[1]:.2f})")
        print(f"Grid: ({robot_grid_x}, {robot_grid_y}) | Visited: {visited_count}/{GRID_SIZE*GRID_SIZE}")
        print_maze_map()

# Final results
total_time = robot.getTime() - navigation_start_time
print(f"\n=== NAVIGATION COMPLETED ===")
print(f"Total time: {total_time:.2f} seconds")
print(f"Success: {distance_to_target <= GOAL_TOLERANCE}")

left_motor.setVelocity(0)
right_motor.setVelocity(0)