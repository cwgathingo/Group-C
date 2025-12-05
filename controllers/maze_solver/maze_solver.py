import os
import sys

# Add the parent 'controllers' directory to sys.path so we can import maze_shared
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from collections import deque
from math import inf
from typing import Dict, List, Optional, Tuple
from controller import Robot

from maze.maze import Maze, Direction, PassageState, Cell
from robots.robot_interface import RobotFacade, MotionAction, ActionResult
from robots.epuck_facade import EPuckFacade

from maze_shared.maze_config import (
    CELL_SIZE as DEFAULT_CELL_SIZE,
    COLS as DEFAULT_COLS,
    DEFAULT_GOAL,
    DEFAULT_START,
    MAZE_ORIGIN,
    ROWS as DEFAULT_ROWS,
    SEED as DEFAULT_SEED,
)


"""
High-level controller for the maze-solving robot.

This class connects the Webots Robot API with the internal Maze
representation and (later) the movement, sensor, and pathfinding
components.

Responsibilities:
- Own the Webots Robot instance.
- Maintain the robot's belief about its current cell and orientation.
- Drive the main control loop: sense -> update map -> choose action -> move.
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
    @param mazeOriginWorld World (x, y) position of cell (0, 0) centre.
    @return None
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

        self._mazeOriginWorld = mazeOriginWorld
        self._startDirection = startDirection
        self._defaultConfig = {
            "rows": rows,
            "cols": cols,
            "start": startCell,
            "goal": goalCell,
            "cell_size": cellSizeMeters,
            "seed": DEFAULT_SEED,
            "startDir": startDirection,
        }

        # Maze belief (populated after runtime config arrives)
        self._maze: Optional[Maze] = None

        # Robot belief about its pose in maze coordinates
        self._currentCell: Optional[Cell] = None
        self._currentDirection: Direction = startDirection

        # Robot motion facade (e-puck implementation)
        self._robotFacade: Optional[RobotFacade] = None

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
        runtimeConfig = self._waitForRuntimeConfig()
        self._initialiseRuntimeState(runtimeConfig)

        if self._robotFacade is None or self._maze is None or self._currentCell is None:
            print("[maze_solver] Runtime configuration failed; stopping controller.")
            return

        while self._robot.step(self._timeStep) != -1:

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
            print("Map after sensing: ")
            self._maze.printAsciiMap()

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
    Wait for the supervisor to signal that the world is ready and return the
    merged runtime configuration.

    This polls customData, stepping the simulation until a payload with
    world_ready=1 is received. Uses startDir when provided.

    @return Dictionary of runtime config values merged with defaults.
    """

    def _waitForRuntimeConfig(self) -> Dict[str, object]:
        print("[maze_solver] Waiting for world_ready==1 from supervisor...")

        parsed: Dict[str, object] = {}

        if hasattr(self._robot, "getCustomData"):
            while True:
                rawData = self._robot.getCustomData() or ""
                parsed = self._parseCustomData(rawData)
                if parsed.get("world_ready") == 1:
                    print("[maze_solver] world_ready==1, starting control loop.")
                    break
                if self._robot.step(self._timeStep) == -1:
                    return self._mergeWithDefaults(parsed)
        else:
            print(
                "[maze_solver] WARNING: Robot API missing getCustomData; "
                "using default configuration."
            )

        merged = self._mergeWithDefaults(parsed)
        print(f"[maze_solver] parsed customData: {parsed}")
        print(f"[maze_solver] runtime config: {merged}")
        return merged

    """
    Parse the supervisor-provided customData string into structured values.

    The expected format is key=value pairs separated by ';' with start/goal
    cells encoded as 'row,col'. Supports startDir for initial heading.
    """

    def _parseCustomData(self, customData: str) -> Dict[str, object]:
        parsed: Dict[str, object] = {}
        entries = [entry.strip() for entry in customData.split(";") if entry.strip()]

        for entry in entries:
            if "=" not in entry:
                continue
            key, rawValue = entry.split("=", 1)
            key = key.strip().lower()
            rawValue = rawValue.strip()

            if key in ("rows", "cols", "seed", "world_ready"):
                try:
                    parsed[key] = int(rawValue)
                except ValueError:
                    continue
            elif key == "startdir":
                parsed["startDir"] = rawValue.lower()
            elif key == "cell_size":
                try:
                    parsed[key] = float(rawValue)
                except ValueError:
                    continue
            elif key in ("start", "goal"):
                parts = [p.strip() for p in rawValue.split(",")]
                if len(parts) != 2:
                    continue
                try:
                    parsed[key] = (int(parts[0]), int(parts[1]))
                except ValueError:
                    continue

        return parsed

    """
    Combine parsed customData with design-time defaults to ensure all keys
    needed by the controller are present.
    """

    def _mergeWithDefaults(self, parsed: Dict[str, object]) -> Dict[str, object]:
        config = {
            "rows": parsed.get("rows", self._defaultConfig["rows"]),
            "cols": parsed.get("cols", self._defaultConfig["cols"]),
            "start": parsed.get("start", self._defaultConfig["start"]),
            "goal": parsed.get("goal", self._defaultConfig["goal"]),
            "cell_size": parsed.get("cell_size", self._defaultConfig["cell_size"]),
            "seed": parsed.get("seed", self._defaultConfig["seed"]),
            "world_ready": parsed.get("world_ready", 0),
            "startDir": parsed.get("startDir", self._defaultConfig["startDir"]),
        }
        return config

    """
    Convert a string into a Direction enum, defaulting to current start
    direction if the string is not recognised. Accepts north/east/south/west.
    """

    def _directionFromString(self, value: object) -> Direction:
        if isinstance(value, Direction):
            return value
        if isinstance(value, str):
            lowered = value.strip().lower()
            if lowered == "north":
                return Direction.NORTH
            if lowered == "east":
                return Direction.EAST
            if lowered == "south":
                return Direction.SOUTH
            if lowered == "west":
                return Direction.WEST
        return self._startDirection

    """
    Initialise maze belief and robot facade after receiving runtime config.
    Applies startDir so the internal heading matches supervisor placement.
    """

    def _initialiseRuntimeState(self, config: Dict[str, object]) -> None:
        rows = int(config["rows"])
        cols = int(config["cols"])
        startValue = config.get("start", self._defaultConfig["start"])
        goalValue = config.get("goal", self._defaultConfig["goal"])
        startCell = (
            startValue
            if isinstance(startValue, tuple)
            else self._defaultConfig["start"]
        )
        goalCell = (
            goalValue if isinstance(goalValue, tuple) else self._defaultConfig["goal"]
        )
        cellSizeMeters = float(config["cell_size"])
        seed = config.get("seed", None)
        startDirValue = config.get("startDir", self._startDirection)
        startDirection = self._directionFromString(startDirValue)

        self._maze = Maze(rows, cols, startCell, goalCell)
        self._currentCell = startCell
        self._currentDirection = startDirection

        self._robotFacade = EPuckFacade(
            robot=self._robot,
            cellSizeMeters=cellSizeMeters,
            mazeOriginWorld=self._mazeOriginWorld,
            startCell=startCell,
            startDirection=self._currentDirection,
        )

        print(
            "[maze_solver] runtime config applied: "
            f"rows={rows}, cols={cols}, start={startCell}, goal={goalCell}, "
            f"cell_size={cellSizeMeters}, seed={seed}, startDir={self._currentDirection}"
        )

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
        if not self._maze.isVisited(self._currentCell):
            self._maze.markVisited(self._currentCell)
        localPassages = self._robotFacade.senseLocalPassages()

        # Debug
        # print("SENSING: localPassages", localPassages)

        for direction in Direction:
            blocked = localPassages.get(direction)
            if blocked:
                self._maze.markPassageState(
                    self._currentCell,
                    direction,
                    PassageState.BLOCKED,
                )

    """
    Compute a wavefront (NF1/grassfire) distance transform from the goal,
    using a breadth-first expansion. Each traversable neighbour of a cell
    is assigned a cost one greater than its parent's, producing a discrete
    distance field over the maze.

    This corresponds to the wavefront planner described in:

    Siegwart, R., Nourbakhsh, I. & Scaramuzza, D. (2011).
    Introduction to Autonomous Mobile Robots (2nd ed.). MIT Press.
    See Section 6.3.1.2, discussion of breadth-first search and the
    wavefront expansion algorithm (NF1/grassfire). 
    """
    """
    Compute a wavefront distance matrix from the goal to every cell.

    @return Matrix of shortest path estimates (inf for unreachable).
    """

    def _computeWavefront(self) -> List[List[int]]:
        maze = self._maze
        (rows, cols) = maze.getShape()

        # Initialise wavefront with +INF everywhere.
        wfMatrix: List[List[int]] = [[inf for _ in range(cols)] for _ in range(rows)]

        goal = maze.getGoal()
        (gr, gc) = goal
        wfMatrix[gr][gc] = 0  # distance 0 at the goal

        # BFS frontier starting from the goal
        frontier = deque([goal])

        while frontier:
            cell = frontier.popleft()
            (row, col) = cell
            currentDistVal = wfMatrix[row][col]

            for direction in Direction:
                passage = maze.getPassage(cell, direction)

                # Skip hard walls; treat UNKNOWN as traversable
                if passage == PassageState.BLOCKED:
                    continue

                neighbourCell = maze.getNeighbour(cell, direction)
                if neighbourCell is None:
                    # Out of bounds (shouldn't happen if borders are BLOCKED,
                    # but this keeps things robust).
                    continue

                (nr, nc) = neighbourCell
                nextDist = currentDistVal + 1

                # Only update if we found a strictly shorter path
                if nextDist < wfMatrix[nr][nc]:
                    wfMatrix[nr][nc] = nextDist
                    frontier.append(neighbourCell)

        return wfMatrix

    """
    Pretty-print the wavefront matrix.
    - INF (unreachable) cells are shown as 'INF'
    - All columns are aligned

    @param wfMatrix Wavefront distance matrix to print.
    @return None
    """

    def _printWavefront(self, wfMatrix: List[List[int]]):

        rows = len(wfMatrix)
        cols = len(wfMatrix[0])

        # Convert values to strings first (INF for inf)
        rowToPrint = []
        for r in range(rows):
            rowStr = []
            for c in range(cols):
                v = wfMatrix[r][c]
                if v == inf:
                    rowStr.append("INF")
                else:
                    rowStr.append(str(v))
            rowToPrint.append(rowStr)

        # Compute max width per column for alignment
        col_widths = [0] * cols
        for c in range(cols):
            col_widths[c] = max(len(rowToPrint[r][c]) for r in range(rows))

        # Print formatted table
        print("\nWavefront Matrix:")
        print("=================")
        for r in range(rows):
            row_out = []
            for c in range(cols):
                s = rowToPrint[r][c]
                row_out.append(s.rjust(col_widths[c]))
            print("  ".join(row_out))
        print()

    """
    Decide the next high-level motion action based on the current
    wavefront distances and maze belief.

    The policy is:
      1) Recompute the wavefront distance matrix from the goal.
      2) At the current cell, consider all non-BLOCKED neighbours
         (UNKNOWN passages are treated as traversable).
      3) Select neighbours whose wavefront value is exactly
         currentDist - 1 (i.e. one step closer to the goal).
      4) If there are multiple candidates, choose the direction that
         best matches the robot’s current heading.
      5) Convert the chosen direction into a MotionAction.

    If no neighbour with a finite distance-1 value exists, the method
    returns None to signal that no progress toward the goal is possible
    under the current belief.

    @return A value representing the chosen action.
    """

    def _decideNextAction(self) -> Optional[MotionAction]:
        # 1. Recompute and print wavefront
        wfm = self._computeWavefront()
        self._printWavefront(wfm)

        # 2. Get current cell and its distance
        row, col = self._currentCell
        currentDist = wfm[row][col]

        if currentDist == inf:
            print("Current cell has no path to goal under current belief.")
            return None

        # 3. Find candidate directions that move to a cell with distance-1
        passages = self._maze.getAllPassages(self._currentCell)
        candidateDirs: List[Direction] = []

        for direction, pState in passages.items():
            # Skip hard walls; treat UNKNOWN as traversable
            if pState == PassageState.BLOCKED:
                continue

            neighbour = self._maze.getNeighbour(self._currentCell, direction)
            if neighbour is None:
                continue

            nr, nc = neighbour
            neighbourDist = wfm[nr][nc]

            if neighbourDist == currentDist - 1:
                candidateDirs.append(direction)

        if not candidateDirs:
            print(
                "No neighbour with wavefront value current-1; "
                "cannot step closer to goal."
            )
            return None

        # 4. Choose the best direction according to current heading
        nextDir = self._choosePreferredDirection(candidateDirs)

        # 5. Convert desired direction into a MotionAction
        return self._directionToAction(nextDir)

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
        self._pendingAction = action
        if action == MotionAction.MOVE_FORWARD_ONE_CELL:
            print("Executing Action: MOVE_FORWARD_ONE_CELL")
            self._robotFacade.requestMoveForwardOneCell()
        elif action == MotionAction.TURN_LEFT_90:
            print("Executing Action: TURN_LEFT_90")
            self._robotFacade.requestTurnLeft90()
        elif action == MotionAction.TURN_RIGHT_90:
            print("Executing Action: TURN_RIGHT_90")
            self._robotFacade.requestTurnRight90()
        else:
            print("Warning: trying to execute unrecognized action: ", action)

    """
    Stop all wheel motors.

    @return None
    """

    def _stopMotors(self) -> None:
        # TODO: set motor velocities to 0
        pass

    """
    Optional fun routine for the end of the maze.
    Could spin the robot in place, flash LEDs, etc.

    @return None
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

    @return None
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

        # Update shared heading belief after *any* action
        self._currentDirection = self._robotFacade.getHeadingDirection()

        if self._pendingAction == MotionAction.MOVE_FORWARD_ONE_CELL:
            self._currentCell = self._robotFacade.getCurrentCell()
            self._maze.markVisited(self._currentCell)

            heading = self._robotFacade.getHeadingDirection()
            opposite = self._maze.getOppositeDirection(heading)
            self._maze.markPassageState(self._currentCell, opposite, PassageState.OPEN)

        elif self._pendingAction in (
            MotionAction.TURN_LEFT_90,
            MotionAction.TURN_RIGHT_90,
        ):
            # Debug
            # print(f"Heading after turn: {self._currentDirection}")
            pass

        print(f"Map after action: {self._pendingAction}: ")
        self._maze.printAsciiMap()
        self._pendingAction = None

    """
    Given a list of candidate directions that all have wavefront distance-1,
    choose one based on current heading:

    1. straight ahead
    2. right turn
    3. left turn
    4. back (180 degrees)

    @param candidates Directions leading to cells with distance-1.
    @return Preferred direction according to heading.
    """

    def _choosePreferredDirection(self, candidates: List[Direction]) -> Direction:
        heading = self._currentDirection

        def priority(direction: Direction) -> int:
            delta = (int(direction) - int(heading)) % 4
            if delta == 0:
                return 0  # straight
            if delta == 1:
                return 1  # right
            if delta == 3:
                return 2  # left
            return 3  # back (delta == 2)

        return min(candidates, key=priority)

    """
    Convert the desired heading into a single MotionAction
    (one atomic step for the high-level controller).

    @param targetDir Desired maze heading.
    @return Corresponding MotionAction to move/turn toward it.
    """

    def _directionToAction(self, targetDir: Direction) -> MotionAction:
        heading = self._currentDirection
        delta = (int(targetDir) - int(heading)) % 4

        if delta == 0:
            return MotionAction.MOVE_FORWARD_ONE_CELL
        elif delta == 1:
            return MotionAction.TURN_RIGHT_90
        elif delta == 3:
            return MotionAction.TURN_LEFT_90
        else:
            # 180 degree turn: choose one direction (right here), the second turn
            # will be planned on the next call to _decideNextAction.
            return MotionAction.TURN_RIGHT_90


"""
Entry point for the controller.

Creates a MazeController instance with a chosen maze size, start and
goal cells, and starts the main control loop.

Adjust the rows, cols, startCell, and goalCell values to match the
Webots world you construct.

@return None
"""


def main() -> None:
    rows = DEFAULT_ROWS
    cols = DEFAULT_COLS
    startCell: Cell = DEFAULT_START
    startDirection = Direction.NORTH
    goalCell: Cell = DEFAULT_GOAL

    cellSizeMeters = DEFAULT_CELL_SIZE
    mazeOriginWorld = MAZE_ORIGIN  # (x, y) of cell (0, 0) centre

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
