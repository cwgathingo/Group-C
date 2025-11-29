from typing import Optional, Tuple
from controller import Robot

from maze.maze import Maze, Direction, PassageState, Cell
from robots.robot_interface import RobotFacade, MotionAction, ActionResult
from robots.epuck_facade import EPuckFacade


# Debug path
pathIndex = 0
pathList = [
    MotionAction.MOVE_FORWARD_ONE_CELL,
    MotionAction.TURN_LEFT_90,
    MotionAction.MOVE_FORWARD_ONE_CELL,
]

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
    @param startDirection the direction of the robot at the start.
    @param goalCell Goal cell (row, col) for the robot.
    @param cellSizeMeters Size of one maze cell edge in meters.
    @param mazeOriginWorld World (x, y) position of cell (0, 0) centr
    """
    def __init__(
            self, 
            rows: int, 
            cols: int, 
            startCell: Cell,
            startDirection: Direction,
            goalCell: Cell,
            cellSizeMeters: float,
            mazeOriginWorld: Tuple[float, float],
            ) -> None:
        # Webots robot
        self._robot = Robot()
        basicStep = int(self._robot.getBasicTimeStep())
        self._timeStep = basicStep if basicStep > 0 else 32

        # Maze belief
        self._maze = Maze(rows, cols, startCell, goalCell)

        # Robot belief about its pose in maze coordinates
        self._currentCell: Cell = startCell
        self._currentDirection: Direction = startDirection

        # Robot motion facade (e-puck implementation)
        self._robotFacade: RobotFacade = EPuckFacade(
            robot=self._robot,
            cellSizeMeters=cellSizeMeters,
            mazeOriginWorld=mazeOriginWorld,
            startCell=startCell,
            startDirection=self._currentDirection,
        )

        # Track the high-level action we asked the robot to execute.
        # None = no pending action (idle from planning perspective).
        self._pendingAction: Optional[MotionAction] = None

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

            # for now it's basically a stub. 
            self._robotFacade.update(self._timeStep / 1000.0)

            # 0. If an action is in progress, update it and skip planning
            if self._robotFacade.isBusy():
                continue

            # If we had asked for an action previously and the robot is now idle,
            # handle the completion here.
            if self._pendingAction is not None:
                self._handleCompletedAction()
            
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
            self._executeAction(action)

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
    def _decideNextAction(self) -> Optional[MotionAction]:
        # TODO: integrate pathfinding / behaviour module here.
        # For now, always return None or a hard-coded action.
        # return None

        # going through hard coded path
        global pathIndex
        global pathList
        if (pathIndex > len(pathList) - 1):
            return None
        action = pathList[pathIndex]
        pathIndex += 1
        return action

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
        if action == MotionAction.MOVE_FORWARD_ONE_CELL:
            print("Executing Action: MOVE_FORWARD_ONE_CELL")
            self._pendingAction = MotionAction.MOVE_FORWARD_ONE_CELL
            self._robotFacade.requestMoveForwardOneCell()
        elif action == MotionAction.TURN_LEFT_90:
            self._robotFacade.requestTurnLeft90()
        elif action == MotionAction.TURN_RIGHT_90:
            self._robotFacade.requestTurnRight90()
        else:
            print("Warning: trying to execute unrecognized action: ", action)

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
    Handle the completion of the last pending action.

    - Check the facade's last action result.
    - If not successful, stop and exit (for now).
    - If MOVE_FORWARD_ONE_CELL succeeded:
        1) Sync currentCell from the robot facade.
        2) Mark that cell as visited in the maze belief.
        3) Mark the passage back to the previous cell as OPEN.
        4) Print the updated ASCII map.
    """
    def _handleCompletedAction(self) -> None:
        if self._pendingAction is None:
            return

        result = self._robotFacade.getLastActionResult()

        if result != ActionResult.SUCCESS:
            print("Action failed:", self._pendingAction, "result:", result)
            self._stopMotors()
            # For now, exit the controller on any failure.
            # (You can add smarter recovery later.)
            self._pendingAction = None
            # Force termination by exiting run() main loop:
            # simplest is to raise SystemExit.
            raise SystemExit

        # At this point: result == SUCCESS
        if self._pendingAction == MotionAction.MOVE_FORWARD_ONE_CELL:
            # 1) Sync current cell belief from robot facade
            self._currentCell = self._robotFacade.getCurrentCell()

            # 2) Mark the passage back to the previous cell as OPEN.
            #    We arrived here from the opposite of the current heading.
            heading = self._robotFacade.getHeadingDirection()
            opposite = self._maze.getOppositeDirection(heading)
            self._maze.markPassageState(
                self._currentCell,
                opposite,
                PassageState.OPEN
            )

            # 4) Print a map for debugging
            print("Map after MOVE_FORWARD_ONE_CELL:")
            self._maze.printAsciiMap()

        # Clear the pending action now that we've processed it
        self._pendingAction = None

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
    startCell: Cell = (3, 3)
    startDirection = Direction.NORTH
    goalCell: Cell = (3, 0)

    # TODO: set these to match your actual world
    cellSizeMeters = 0.15          # placeholder
    mazeOriginWorld = (-0.225, 0.225)  # placeholder (x, y of cell (0, 0) centre)

    controller = MazeController(
        rows,
        cols,
        startCell,
        startDirection,
        goalCell,
        cellSizeMeters,
        mazeOriginWorld,
    )

    controller.run()


# Webots uses the file as a module; this guard allows running it
# directly with Python for basic import checks without starting Webots.
if __name__ == "__main__":
    main()
