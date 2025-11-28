from typing import Optional
from controller import Robot

from maze.maze import Maze, Direction, PassageState, Cell


"""
High-level controller for the maze-solving robot.

This class connects the Webots Robot API with the internal Maze
representation and (later) the movement, sensor, and pathfinding
components.

Responsibilities:
- Own the Webots Robot instance.
- Maintain the robot's belief about its current cell and orientation.
- Drive the main control loop: sense → update map → choose action → move.
"""
class MazeController:

    """
    Initialise the maze controller.

    This constructor:
    - Creates the Webots Robot instance.
    - Stores the basic timestep.
    - Creates the Maze belief structure.
    - Initialises the robot's believed cell and heading.

    @param rows Number of maze rows in the belief model.
    @param cols Number of maze columns in the belief model.
    @param startCell Starting cell (row, col) for the robot.
    @param goalCell Goal cell (row, col) for the robot.
    """
    def __init__(self, rows: int, cols: int, startCell: Cell, goalCell: Cell) -> None:
        # Webots robot
        self._robot = Robot()
        basicStep = int(self._robot.getBasicTimeStep())
        self._timeStep = basicStep if basicStep > 0 else 32

        # Maze belief
        self._maze = Maze(rows, cols, startCell, goalCell)

        # Robot belief about its pose in maze coordinates
        self._currentCell: Cell = startCell
        self._currentDirection: Direction = Direction.NORTH  # assumed initial heading

        # Movement / action state
        self._actionInProgress = False
        self._currentAction = None  # later: probably an enum

        # TODO: initialise devices (wheels, distance sensors, GPS, compass)
        # self._leftMotor = ...
        # self._rightMotor = ...
        # self._distanceSensors = [...]
        # self._gps = ...
        # self._compass = ...

    """
    Main control loop.

    At a high level, the loop alternates between:

    - Executing the current movement action (if one is in progress).
    - When idle:
        1) Check if the robot is at the goal.
        2) Sense the environment and update the map.
        3) Decide the next action using the maze belief.
        4) Start executing that action.

    Movement actions may take multiple Webots timesteps to complete,
    so the controller only performs sensing and planning when no action
    is currently in progress.

    @return None
    """
    def run(self) -> None:
        while self._robot.step(self._timeStep) != -1:

            # 0. If an action is in progress, update it and skip planning
            if self._actionInProgress:
                self._updateOngoingAction()
                continue

            # 1. Check goal condition (only when robot is idle)
            if self._currentCell == self._maze.getGoal():
                print("\n==============================")
                print("  GOAL REACHED!  ")
                print("==============================\n")
                # Optional: victory dance / spin / LED flash
                self._victoryCelebration()
                # Ensure motors are stopped
                self._stopMotors()
                # Exit the loop cleanly
                break
            # 2. Sense environment and 3. update maze belief
            self._senseAndUpdateMap()

            # 4. Decide next action based on the updated belief
            action = self._decideNextAction()

            # Deal with edge cases, for example pathFinder doesn't have a path
            if action is None:
                print("No action decided; stopping.")
                self._stopMotors()
                break
            
            # 5. Start executing the chosen action (async movement)
            self._startAction(action)

        print("Final belief map:")
        self._maze.printAsciiMap()

    """
    Read sensors and update the maze belief.

    This method is responsible for:
    - Polling the robot's distance sensors (or other sensors).
    - Interpreting the readings as OPEN/BLOCKED passages around the
      current cell.
    - Calling maze.markPassageState(...) for each direction where the
      belief should be updated.
    - Marking the current cell as visited.

    Currently implemented as a stub to be filled when the sensor
    configuration is finalised.

    @return None
    """
    def _senseAndUpdateMap(self) -> None:
        # TODO: read distance sensors and infer passage states.
        # Example structure (pseudo-code):
        #
        #   hasWallNorth = ...
        #   if hasWallNorth:
        #       self._maze.markPassageState(self._currentCell, Direction.NORTH, PassageState.BLOCKED)
        #   else:
        #       self._maze.markPassageState(self._currentCell, Direction.NORTH, PassageState.OPEN)
        #
        # Repeat for EAST, SOUTH, WEST depending on sensor layout.
        #
        self._maze.markVisited(self._currentCell)

    """
    Decide the next action for the robot.

    For now, this method is a placeholder. In the final system, it will:
    - Use the Maze belief (passages and visited cells).
    - Call a pathfinding or behaviour-based algorithm.
    - Return a symbolic action such as:
        "MOVE_FORWARD_ONE_CELL", "TURN_LEFT", "TURN_RIGHT"
      or a more structured type.

    @return A value representing the chosen action.
    """
    def _decideNextAction(self):
        # TODO: integrate pathfinding / behaviour module here.
        # For now, always return None or a hard-coded action.
        return None

    """
    Execute one atomic action and update the belief pose.

    This method is responsible for:
    - Sending motor commands to perform the action (in Webots world).
    - Waiting for the movement to complete (possibly over multiple
      timesteps, depending on how you implement movement).
    - Updating _currentCell and/or _currentDirection in the belief.

    @param action The action to execute (as chosen by _decideNextAction).
    @return None
    """
    def _executeAction(self, action) -> None:
        # TODO: implement movement primitives (turn left/right, move one cell).
        # For now, this is a stub so the controller compiles and runs.
        pass

    """Stop all wheel motors."""
    def _stopMotors(self) -> None:
        # TODO: set motor velocities to 0
        pass

        """
    Optional fun routine for the end of the maze.
    Could spin the robot in place, flash LEDs, etc.
    """
    def _victoryCelebration(self) -> None:
        # TODO: implement a small spin or LED pattern if desired.
        # Completely optional, but looks great in presentation video.
        pass

"""
Entry point for the controller.

Creates a MazeController instance with a chosen maze size, start and
goal cells, and starts the main control loop.

Adjust the rows, cols, startCell, and goalCell values to match the
Webots world you construct.

@return None
"""
def main() -> None:
    rows = 4
    cols = 4
    startCell: Cell = (0, 0)
    goalCell: Cell = (3, 3)

    controller = MazeController(rows, cols, startCell, goalCell)
    controller.run()


# Webots uses the file as a module; this guard allows running it
# directly with Python for basic import checks without starting Webots.
if __name__ == "__main__":
    main()
