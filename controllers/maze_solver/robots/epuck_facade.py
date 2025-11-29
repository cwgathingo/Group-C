# controllers/maze_solver/robots/epuck_facade.py

from typing import Tuple, Optional
import math

from controller import Robot as WebotsRobot

from .robot_interface import (
    RobotFacade,
    RobotState,
    MotionAction,
    ActionResult,
    Vec2,
    Pose2D,
    directionUnit
)
from maze.maze import Cell, Direction  # if Pylance complains, you can switch to relative later


WHEEL_RADIUS = 0.02 # meters

"""
Convert a maze Direction into a world-frame orientation angle.

The exact mapping depends on how the maze is aligned in the Webots world.
For example, if NORTH corresponds to +y in world coordinates and EAST to
+x, you might choose:

    EAST  → 0 rad
    NORTH → +π/2
    WEST  → π
    SOUTH → -π/2

You should adjust this mapping to match your actual world axes.
"""
def _directionToWorldTheta(direction: Direction) -> float:
    pi = math.pi
    if direction == Direction.EAST:
        return 0.0
    if direction == Direction.NORTH:
        return 0.5 * pi
    if direction == Direction.WEST:
        return pi
    if direction == Direction.SOUTH:
        return -0.5 * pi
    # Fallback; shouldn't happen
    return 0.0


"""
E-puck-specific implementation of the RobotFacade interface.

This class wraps the Webots e-puck API (motors, sensors, etc.) and exposes
a clean, maze-level motion interface to MazeController.
"""
class EPuckFacade(RobotFacade):

    """
    Initialise the e-puck facade and its pose in the maze.

    The constructor assumes that, at simulation start, the robot is placed
    exactly at the centre of a known maze cell with a known heading.

    Using this information and the maze geometry, the facade:
    - Sets the discrete pose (current cell + Direction).
    - Computes the corresponding world pose (x, y, theta) at the cell centre.
    - Prepares internal state for later movement primitives.

    @param robot           Webots Robot instance provided by the controller.
    @param cellSizeMeters  Size of one maze cell edge in meters.
    @param mazeOriginWorld World (x, y) position of cell (0, 0) centre.
    @param startCell       Starting cell as (row, col) in maze coordinates.
    @param startDirection  Initial maze heading (e.g. Direction.NORTH).
    """
    def __init__(
        self,
        robot: WebotsRobot,
        cellSizeMeters: float,
        mazeOriginWorld: Tuple[float, float],
        startCell: Cell,
        startDirection: Direction,
    ) -> None:

        self._robot = robot

        # Maze geometry
        self._cellSize = cellSizeMeters
        self._mazeOrigin = mazeOriginWorld

        # Basic timestep (ms → s) from Webots
        basicStepMs = int(self._robot.getBasicTimeStep())
        self._timeStepSeconds = (basicStepMs if basicStepMs > 0 else 32) / 1000.0

        # Devices (to be initialised by you)
        self._leftMotor = robot.getDevice('left wheel motor')
        self._rightMotor = robot.getDevice('right wheel motor')

        # Enable velocity mode
        self._leftMotor.setPosition(float("inf"))
        self._rightMotor.setPosition(float("inf"))
        self._leftMotor.setVelocity(0.0)
        self._rightMotor.setVelocity(0.0)

        # Query max speeds
        self._maxLeftSpeed = self._leftMotor.getMaxVelocity()
        self._maxRightSpeed = self._rightMotor.getMaxVelocity()

        # For simplicity, use min of both
        self._maxSpeed = min(self._maxLeftSpeed, self._maxRightSpeed)

        print("[EPuckFacade] Max motor speeds:", self._maxLeftSpeed, self._maxRightSpeed)


        self._compass = robot.getDevice('compass')
        self._compass.enable(basicStepMs)
        # self._encoders = ...
        self._gps = robot.getDevice('gps')
        self._gps.enable(basicStepMs)

        # --- Discrete pose (maze coordinates) ---
        self._currentCell: Cell = startCell
        self._currentDirection: Direction = startDirection

        self._setWorldPose()

        # GPS offset is unknown at start; we’ll compute it lazily
        self._gpsOffset: Optional[Tuple[float, float]] = None

        # --- Action / state tracking ---
        self._state: RobotState = RobotState.IDLE
        self._currentAction: Optional[MotionAction] = None
        self._lastActionResult: ActionResult = ActionResult.NONE
        self._targetPose = None

    # ------------------------------------------------------------------
    # Core update loop
    # ------------------------------------------------------------------

    """
    Advance the internal control loop by one timestep.

    For now this just stubs out the structure. You will:
    - Read sensors / encoders.
    - Update ongoing motion (if any).
    - Apply course correction.
    - Decide when an action has completed.
    """
    def update(self, timeStepSeconds: float) -> None:
        print("time: ", self._robot.getTime())
        print('_gpsOffset', self._gpsOffset)
        # First, if we haven't calibrated GPS offset yet, try to do it.
        if self._gpsOffset is None:
            gpsValues = self._gps.getValues()
            gpsX, gpsY, _ = gpsValues

            if not math.isnan(gpsX) and not math.isnan(gpsY):
                worldX, worldY, _theta = self._worldPose
                # gps = world + offset  → offset = gps - world
                offsetX = gpsX - worldX
                offsetY = gpsY - worldY
                self._gpsOffset = (offsetX, offsetY)
                # Optional debug:
                print("[EPuckFacade] Calibrated GPS offset:", self._gpsOffset)
            # If still NaN, just skip; we’ll try again on the next update.

        if self._state != RobotState.EXECUTING_ACTION:
            # Nothing to do; keep motors stopped.
            return

        # At the moment we only support one action type.
        if self._currentAction == MotionAction.MOVE_FORWARD_ONE_CELL:
            self._updateForwardAction()


        # TODO: implement control logic for the current action.
        # Something like:
        # - if self._currentAction == MotionAction.MOVE_FORWARD_ONE_CELL: ...
        # - if finished: self._finishAction(ActionResult.SUCCESS)
        pass

    # ------------------------------------------------------------------
    # Pose / orientation
    # ------------------------------------------------------------------

    """
    Get the robot's current cell in maze coordinates.

    This is the discrete pose used by the maze and planner. It is updated
    whenever an atomic move action (one cell forward) completes successfully.

    @return Current cell as (row, col).
    """
    def getCurrentCell(self) -> Cell:
        return self._currentCell

    """
    Get the robot's current heading as a unit vector in world coordinates.

    For now you can return a dummy value until compass is wired.
    """
    def getHeadingVector(self) -> Vec2:
        # TODO: read compass and convert to (hx, hy) unit vector.
        return (0.0, 1.0)

    """
    Get the robot's current heading snapped to the nearest cardinal direction.

    Initially you can just return self._currentDirection; later you can
    derive it from the heading vector with a tolerance.
    """
    def getHeadingDirection(self) -> Direction:
        return self._currentDirection
    
    """
    Return GPS-based (x, y) corrected by the calibrated offset.

    If we don't yet have valid GPS or offset, returns None.
    """
    def _getCleanWorldXY(self) -> Optional[Tuple[float, float]]:
        if self._gpsOffset is None:
            return None

        gpsX, gpsY, _ = self._gps.getValues()
        if math.isnan(gpsX) or math.isnan(gpsY):
            return None

        offsetX, offsetY = self._gpsOffset
        worldX = gpsX - offsetX
        worldY = gpsY - offsetY
        return (worldX, worldY)

    # ------------------------------------------------------------------
    # Atomic movement requests
    # ------------------------------------------------------------------

    """
    Request: move forward exactly one maze cell.
    """
    def requestMoveForwardOneCell(self) -> None:
        print("requestMoveForwardOneCell")
        if self._state == RobotState.EXECUTING_ACTION:
            # For now, ignore if already busy. Later you could raise or log.
            return

        self._state = RobotState.EXECUTING_ACTION
        self._currentAction = MotionAction.MOVE_FORWARD_ONE_CELL
        self._lastActionResult = ActionResult.NONE

        # TODO:
        print(self._gps.getValues())
        print(self._worldPose)
        # - compute target world pose based on currentCell + currentDirection
        dx, dy = directionUnit[self._currentDirection]
        print("x, dy = directionUnit[self._currentDirection]", dx, dy)
        currentX, currentY, _theta = self._worldPose
        targetX = currentX + dx * self._cellSize
        targetY = currentY + dy * self._cellSize
        targetTheta = self._worldPose[2]  # same heading
        self._targetPose = (targetX, targetY, targetTheta)
        print("self._targetPose", self._targetPose)
        # - start motors in the right direction
        forwardSpeed = 0.8 * self._maxSpeed
        self._leftMotor.setVelocity(forwardSpeed)
        self._rightMotor.setVelocity(forwardSpeed)

    """
    Request: turn 90 degrees left (counter-clockwise).
    """
    def requestTurnLeft90(self) -> None:
        if self._state == RobotState.EXECUTING_ACTION:
            return

        self._state = RobotState.EXECUTING_ACTION
        self._currentAction = MotionAction.TURN_LEFT_90
        self._lastActionResult = ActionResult.NONE

        # TODO:
        # - determine target heading angle
        # - start motors for in-place turn
        pass

    """
    Request: turn 90 degrees right (clockwise).
    """
    def requestTurnRight90(self) -> None:
        if self._state == RobotState.EXECUTING_ACTION:
            return

        self._state = RobotState.EXECUTING_ACTION
        self._currentAction = MotionAction.TURN_RIGHT_90
        self._lastActionResult = ActionResult.NONE

        # TODO: similar to requestTurnLeft90 but opposite direction.
        pass

    """
    Update logic for MOVE_FORWARD_ONE_CELL.

    Uses GPS (with offset) to estimate current (x, y), compares to
    target cell centre, and stops when within a small threshold.
    """
    def _updateForwardAction(self) -> None:
        cleanPos = self._getCleanWorldXY()
        print("clean pos", cleanPos)
        if cleanPos is None:
            # No reliable GPS yet; keep driving, try again next tick.
            return

        currentX, currentY = cleanPos
        targetX, targetY, targetTheta = self._targetPose

        # Update our internal world pose's x,y; keep theta as before for now.
        _oldX, _oldY, currentTheta = self._worldPose
        self._worldPose = (currentX, currentY, currentTheta)

        # Distance to target centre
        dist = math.hypot(targetX - currentX, targetY - currentY)
        # Tune this threshold; 0.005 = 5mm, 0.01 = 1cm, etc.
        positionTolerance = 0.001

        # Debug (optional)
        print(f"[EPuckFacade] forward dist={dist:.4f}, target=({targetX:.3f},{targetY:.3f}), "
              f"pos=({currentX:.3f},{currentY:.3f})")

        if dist > positionTolerance:
            # Not there yet; leave motors running.
            return

        # We consider the movement complete. Stop motors and finish.
        self._leftMotor.setVelocity(0.0)
        self._rightMotor.setVelocity(0.0)

        # Update discrete cell belief: we have moved one cell in currentDirection.
        row, col = self._currentCell
        if self._currentDirection == Direction.NORTH:
            row -= 1
        elif self._currentDirection == Direction.SOUTH:
            row += 1
        elif self._currentDirection == Direction.EAST:
            col += 1
        elif self._currentDirection == Direction.WEST:
            col -= 1
        self._currentCell = (row, col)

        # Snap worldPose to the exact target (optional but nice).
        self._worldPose = (targetX, targetY, targetTheta)

        self._finishAction(ActionResult.SUCCESS)
        print("Finished Action Forward")

    # ------------------------------------------------------------------
    # Status / results
    # ------------------------------------------------------------------

    """
    Get the high-level state of the robot.

    @return RobotState.IDLE, RobotState.EXECUTING_ACTION, or RobotState.ERROR.
    """
    def getState(self) -> RobotState:
        return self._state

    """
    Check whether the robot is currently executing an action.
    """
    def isBusy(self) -> bool:
        return self._state == RobotState.EXECUTING_ACTION

    """
    Get the currently executing motion action, if any.
    """
    def getCurrentAction(self) -> Optional[MotionAction]:
        return self._currentAction

    """
    Get the result of the most recently completed action.
    """
    def getLastActionResult(self) -> ActionResult:
        return self._lastActionResult

    # ------------------------------------------------------------------
    # Control / emergency behaviour
    # ------------------------------------------------------------------

    """
    Cancel any ongoing action and stop the robot safely.
    """
    def cancelAction(self) -> None:
        # TODO: stop motors
        # self._leftMotor.setVelocity(0.0)
        # self._rightMotor.setVelocity(0.0)

        if self._state == RobotState.EXECUTING_ACTION:
            self._lastActionResult = ActionResult.ABORTED

        self._state = RobotState.IDLE
        self._currentAction = None

    """
    Reset the robot's internal pose belief to a known cell and heading.

    This updates both:
    - The discrete pose (currentCell + currentDirection).
    - The continuous world pose, assuming the robot is at the centre of
      the given cell with the given heading.

    @param cell      New cell as (row, col).
    @param direction New maze heading.
    """
    def resetPose(self, cell: Cell, direction: Direction) -> None:
        self._currentCell = cell
        self._currentDirection = direction

        row, col = cell
        originX, originY = self._mazeOrigin
        worldX = originX + col * self._cellSize
        worldY = originY + row * self._cellSize
        theta = _directionToWorldTheta(direction)
        self._worldPose = (worldX, worldY, theta)

    # ------------------------------------------------------------------
    # Optional world-pose accessors
    # ------------------------------------------------------------------

    def _setWorldPose(self) -> None:
                # --- Continuous pose (world coordinates) ---
        # Start exactly at the centre of the start cell.
        (row, col) = self._currentCell
        (originX, originY) = self._mazeOrigin

        worldX = originX + row * self._cellSize
        worldY = originY - col * self._cellSize
        theta = _directionToWorldTheta(self._currentDirection)

        self._worldPose: Pose2D = (worldX, worldY, theta)

    """
    Optionally return the robot's precise world pose.

    For EPuckFacade this reflects the internal (x, y, theta) belief derived
    from the known start cell and subsequent motion. If you later decide not
    to maintain a continuous pose, you may return None instead.

    @return Tuple (x, y, theta) or None.
    """
    def getWorldPose(self) -> Optional[Pose2D]:
        return self._worldPose

    """
    Optionally return the robot's world position only (x, y).
    """
    def getWorldPosition(self) -> Optional[Tuple[float, float]]:
        if self._worldPose is None:
            return None
        (x, y) = self._worldPose
        return (x, y)

    # ------------------------------------------------------------------
    # Internal helpers (you’ll implement these as you go)
    # ------------------------------------------------------------------

    """
    Finish the current action with the given result and stop the motors.

    This is a small convenience to keep update() tidy.
    """
    def _finishAction(self, result: ActionResult) -> None:
        # TODO: actually stop motors here.
        # self._leftMotor.setVelocity(0.0)
        # self._rightMotor.setVelocity(0.0)

        self._lastActionResult = result
        self._state = RobotState.IDLE
        self._currentAction = None
