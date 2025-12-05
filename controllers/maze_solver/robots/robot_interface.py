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
    """Atomic motion primitives the robot can execute."""

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
    """Lifecycle state for the robot facade."""

    IDLE = auto()
    EXECUTING_ACTION = auto()
    ERROR = auto()


DIRECTION_UNIT = {
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
- Execute atomic movement actions (move forward one cell, turn 90 degrees).
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

    @param timeStepSeconds Duration of this timestep in seconds.
    @return None
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

    @return None
    """

    @abstractmethod
    def requestMoveForwardOneCell(self) -> None:
        raise NotImplementedError

    """
    Request an atomic movement: turn 90 degrees to the left (counter-clockwise).

    @return None
    """

    @abstractmethod
    def requestTurnLeft90(self) -> None:
        raise NotImplementedError

    """
    Request an atomic movement: turn 90 degrees to the right (clockwise).

    @return None
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

    @return True if an atomic action is in progress.
    """

    @abstractmethod
    def isBusy(self) -> bool:
        raise NotImplementedError

    """
    Get the currently executing motion action, if any.

    @return MotionAction for the ongoing action, or None if idle.
    """

    @abstractmethod
    def getCurrentAction(self) -> Optional[MotionAction]:
        raise NotImplementedError

    """
    Get the result of the most recently completed action.

    @return ActionResult enum indicating the last completed action's outcome.
    """

    @abstractmethod
    def getLastActionResult(self) -> ActionResult:
        raise NotImplementedError

    """
    Get a mapping Direction -> has_wall (True/False) for the current pose.

    @return Direction -> Optional[bool], where True=wall, None=unknown.
    """

    @abstractmethod
    def senseLocalPassages(self) -> dict[Direction, Optional[bool]]:
        raise NotImplementedError

    # ------------------------------------------------------------------
    # Control / emergency behaviour
    # ------------------------------------------------------------------

    """
    Cancel any ongoing action and stop the robot safely.

    @return None
    """

    @abstractmethod
    def cancelAction(self) -> None:
        raise NotImplementedError

    """
    Reset the robot's internal pose belief to a known cell and heading.

    @param cell Starting cell as (row, col).
    @param direction Initial heading as a Direction.
    @return None
    """

    @abstractmethod
    def resetPose(self, cell: Cell, direction: Direction) -> None:
        raise NotImplementedError

    # ------------------------------------------------------------------
    # Optional world-pose accessors (may be unimplemented)
    # ------------------------------------------------------------------

    """
    Optionally return the robot's precise world pose.

    @return Tuple (x, y, theta) or None if unsupported.
    """

    def getWorldPose(self) -> Optional[Pose2D]:
        return None

    """
    Optionally return the robot's world position only (x, y).

    @return Tuple (x, y) or None if unsupported.
    """

    def getWorldPosition(self) -> Optional[Tuple[float, float]]:
        return None
