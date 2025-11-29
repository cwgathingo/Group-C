from abc import ABC, abstractmethod
from enum import Enum, auto
from typing import Tuple, Optional

from maze.maze import Cell, Direction

Vec2 = Tuple[float, float]
Pose2D = Tuple[float, float, float]


"""
Represents the type of atomic motion the robot can execute.

These actions are defined in maze/maze coordinates, not raw Webots units.
"""


class MotionAction(Enum):
    MOVE_FORWARD_ONE_CELL = auto()
    TURN_LEFT_90 = auto()
    TURN_RIGHT_90 = auto()


"""
Represents the outcome of the most recently completed atomic action.
"""


class ActionResult(Enum):
    SUCCESS = auto()
    FAILED = auto()
    ABORTED = auto()
    TIMEOUT = auto()
    NONE = auto()  # no action has completed yet (initial state)


"""
High-level state of the robot facade.

This is a coarse summary used by MazeController to reason about what the
robot is currently doing.
"""


class RobotState(Enum):
    IDLE = auto()
    EXECUTING_ACTION = auto()
    ERROR = auto()


directionUnit = {
    Direction.NORTH: (0.0, +1.0),
    Direction.EAST: (+1.0, 0.0),
    Direction.SOUTH: (0.0, -1.0),
    Direction.WEST: (-1.0, 0.0),
}

"""
Abstract robot facade used by MazeController.

This interface is robot-agnostic: different robots (e-puck, others) can
implement the same methods to support the maze-solving controller.

Responsibilities:
- Expose the robot's pose in maze coordinates (cell + heading).
- Execute atomic movement actions (move forward one cell, turn 90°).
- Maintain and expose internal action state (busy/idle/error, last result).
- Run its own low-level control loop each timestep (update()).

Implementations are expected to:
- Use Webots-specific APIs (motors, encoders, GPS, compass, etc.).
- Handle veering and small errors via internal course correction.
- Respect tolerances when deciding that an action has completed.
- Stop the motors when no action is being executed.
"""


class RobotFacade(ABC):
    """
    Advance the internal control loop by one timestep.

    This method should:
    - Read sensors / encoders.
    - Update any ongoing motion action.
    - Apply course-correction as needed.
    - Update internal pose estimates.

    It must NOT block; MazeController will call this once per timestep.

    @param timeStepSeconds Duration of this timestep in seconds.
    """

    @abstractmethod
    def update(self, timeStepSeconds: float) -> None:
        raise NotImplementedError

    # ------------------------------------------------------------------
    # Pose / orientation
    # ------------------------------------------------------------------

    """
    Get the robot's current cell in maze coordinates.

    @return Current cell as (row, col).
    """

    @abstractmethod
    def getCurrentCell(self) -> Cell:
        raise NotImplementedError

    """
    Get the robot's current heading as a unit vector in world coordinates.

    The vector should be normalised to length 1 (within numerical tolerance)
    and represent the robot's facing direction in the world frame.

    @return Tuple (hx, hy) representing a unit heading vector.
    """

    @abstractmethod
    def getHeadingVector(self) -> Vec2:
        raise NotImplementedError

    """
    Get the robot's current heading snapped to the nearest cardinal direction.

    This method converts the raw heading vector (e.g. from the compass) into
    a maze Direction (NORTH, EAST, SOUTH, WEST), using whatever tolerance the
    implementation deems appropriate.

    @return Direction enum value representing the maze heading.
    """

    @abstractmethod
    def getHeadingDirection(self) -> Direction:
        raise NotImplementedError

    # ------------------------------------------------------------------
    # Atomic movement requests
    # ------------------------------------------------------------------

    """
    Request an atomic movement: move forward exactly one maze cell.

    This method should:
    - Start an asynchronous movement towards the next cell in the current
      heading direction.
    - Leave the robot in an EXECUTING_ACTION state until the movement
      completes or fails.
    - Ensure that when the action has finished (in any outcome), the motors
      are stopped and the robot is safe to receive the next command.
    - Not block; completion is observed via update() + status methods.

    If the robot is already busy when this is called, implementations may
    either ignore the request or treat it as an error; this behaviour should
    be documented in the concrete implementation.
    """

    @abstractmethod
    def requestMoveForwardOneCell(self) -> None:
        raise NotImplementedError

    """
    Request an atomic movement: turn 90 degrees to the left (counter-clockwise).

    This method starts an asynchronous rotation; the robot remains in-place
    (same cell) but its heading changes by +90 degrees in maze coordinates.

    When the action has finished (in any outcome), the implementation must
    leave the motors stopped.

    Completion and result are obtained via the status methods.
    """

    @abstractmethod
    def requestTurnLeft90(self) -> None:
        raise NotImplementedError

    """
    Request an atomic movement: turn 90 degrees to the right (clockwise).

    This method starts an asynchronous rotation; the robot remains in-place
    (same cell) but its heading changes by -90 degrees in maze coordinates.

    When the action has finished (in any outcome), the implementation must
    leave the motors stopped.

    Completion and result are obtained via the status methods.
    """

    @abstractmethod
    def requestTurnRight90(self) -> None:
        raise NotImplementedError

    # ------------------------------------------------------------------
    # Status / results
    # ------------------------------------------------------------------

    """
    Get the high-level state of the robot.

    @return RobotState.IDLE, RobotState.EXECUTING_ACTION, or RobotState.ERROR.
    """

    @abstractmethod
    def getState(self) -> RobotState:
        raise NotImplementedError

    """
    Check whether the robot is currently executing an action.

    This is equivalent to checking getState() == RobotState.EXECUTING_ACTION.

    @return True if an atomic action is in progress, False if the robot is idle
            or in an error state.
    """

    @abstractmethod
    def isBusy(self) -> bool:
        raise NotImplementedError

    """
    Get the currently executing motion action, if any.

    When the robot is idle or in an error state, this should return None.

    @return MotionAction for the ongoing action, or None if idle.
    """

    @abstractmethod
    def getCurrentAction(self) -> Optional[MotionAction]:
        raise NotImplementedError

    """
    Get the result of the most recently completed action.

    While an action is still in progress, implementations may:
    - Return ActionResult.NONE, or
    - Keep returning the previous completed result.
    This behaviour should be documented in the concrete implementation.

    Regardless of the specific outcome (SUCCESS, FAILED, TIMEOUT, ABORTED),
    when an action has finished the robot must have its motors stopped.

    @return ActionResult enum indicating the last completed action's outcome.
    """

    @abstractmethod
    def getLastActionResult(self) -> ActionResult:
        raise NotImplementedError

    """
    gets a mapping Direction -> has_wall (True/False) for
    the directions that sensors can see from the current pose.
    @return  Direction -> Optional[bool], where True=wall, None=unknown.
    """

    @abstractmethod
    def senseLocalPassages(self) -> dict[Direction, Optional[bool]]:
        raise NotImplementedError

    # ------------------------------------------------------------------
    # Control / emergency behaviour
    # ------------------------------------------------------------------

    """
    Cancel any ongoing action and stop the robot safely.

    Implementations should:
    - Stop the wheel motors (or equivalent).
    - Mark the current action as aborted, if appropriate.
    - Transition to an idle or error state, depending on the implementation.

    If no action is in progress, this should be a no-op.
    """

    @abstractmethod
    def cancelAction(self) -> None:
        raise NotImplementedError

    """
    Reset the robot's internal pose belief to a known cell and heading.

    This is useful when:
    - The simulation is reset.
    - The robot is placed at a known starting cell in the maze.
    - You want to realign internal odometry with the world frame.

    @param cell Starting cell as (row, col).
    @param direction Initial heading as a Direction.
    """

    @abstractmethod
    def resetPose(self, cell: Cell, direction: Direction) -> None:
        raise NotImplementedError

    # ------------------------------------------------------------------
    # Optional world-pose accessors (may be unimplemented)
    # ------------------------------------------------------------------

    """
    Optionally return the robot's precise world pose.

    This is intended for debugging, visualisation, or for future robots
    that provide high-quality localisation. Implementations that do not
    track a continuous pose may return None.

    The pose is (x, y, theta) where:
    - x, y are world coordinates in meters.
    - theta is orientation in radians in the same world frame used for
      getHeadingVector().

    MazeController must NOT depend on this being available.

    @return Tuple (x, y, theta) or None if unsupported.
    """

    def getWorldPose(self) -> Optional[Pose2D]:
        return None

    """
    Optionally return the robot's world position only (x, y).

    Provided as a convenience for robots without a reliable yaw estimate.
    Implementations may return None.

    MazeController must NOT depend on this being available.

    @return Tuple (x, y) or None if unsupported.
    """

    def getWorldPosition(self) -> Optional[Tuple[float, float]]:
        return None
