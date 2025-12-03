# controllers/maze_solver/robots/epuck_facade.py

from typing import Tuple, Optional
from math import cos, pi, sin

from controller import Robot as WebotsRobot

from .robot_interface import (
    RobotFacade,
    RobotState,
    MotionAction,
    ActionResult,
    Vec2,
    Pose2D,
    directionUnit,
)
from maze.maze import (
    Cell,
    Direction,
)  # if Pylance complains, you can switch to relative later


# --- Calibrated odometry parameters ---
# These values were tuned against observed forward travel and turn angles
# in Webots. They do NOT represent the physical real-world dimensions of
# the e-puck; they are effective simulation scaling factors that make
# encoder-based odometry match actual motion.
WHEEL_RADIUS = 0.02  # meters
TRACK_WIDTH = 0.057

# Tolerance Definitions
POSITION_TOLERANCE = 0.0005
ANGLE_TOLERANCE = 0.03  # ~1.7°

# Sensors Wall Threshold
IR_SESNOR_WALL_THRESHOLD = 80
FRONT_SESNOR_WALL_THRESHOLD = 900


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
        self._leftMotor = robot.getDevice("left wheel motor")
        self._rightMotor = robot.getDevice("right wheel motor")

        # Enable velocity mode
        self._leftMotor.setPosition(float("inf"))
        self._rightMotor.setPosition(float("inf"))
        self._leftMotor.setVelocity(0.0)
        self._rightMotor.setVelocity(0.0)

        # Enable encoders
        self._leftEncoder = robot.getDevice("left wheel sensor")
        self._rightEncoder = robot.getDevice("right wheel sensor")
        self._leftEncoder.enable(basicStepMs)
        self._rightEncoder.enable(basicStepMs)
        self._startLeftEnc = None
        self._startRightEnc = None

        # Enable IR Seonsors
        self._irSensors = []
        for i in range(8):
            sensor = robot.getDevice(f"ps{i}")
            sensor.enable(basicStepMs)
            self._irSensors.append(sensor)
        self._frontSensor = robot.getDevice("distance sensor")
        self._frontSensor.enable(basicStepMs)

        # set speed scalar
        self._baseForwardSpeedFrac = 0.6

        # Query max speeds
        self._maxLeftSpeed = self._leftMotor.getMaxVelocity()
        self._maxRightSpeed = self._rightMotor.getMaxVelocity()

        # For simplicity, use min of both
        self._maxSpeed = min(self._maxLeftSpeed, self._maxRightSpeed)

        self._compass = robot.getDevice("compass")
        self._compass.enable(basicStepMs)

        # --- Discrete pose (maze coordinates) ---
        self._currentCell: Cell = startCell
        self._currentDirection: Direction = startDirection

        self._setWorldPose()

        # --- Action / state tracking ---
        self._state: RobotState = RobotState.IDLE
        self._currentAction: Optional[MotionAction] = None
        self._lastActionResult: ActionResult = ActionResult.NONE
        self._targetPose = None

        # --- Turn tracking ---
        self._turnSpeed: float = 0.3 * self._maxSpeed  # base turn speed
        self._turnSign: Optional[int] = None  # +1 = left turn, -1 = right turn

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
        if self._state != RobotState.EXECUTING_ACTION:
            # Nothing to do; keep motors stopped.
            return

        # Debug
        # readings = self._readIrRaw()
        # print("IR:", " ".join(f"{v:5.1f}" for v in readings))
        # print("IR, blocked list: ", self.senseLocalPassages())
        # print("Front Sensor: ", self._frontSensor.getValue())

        if self._currentAction == MotionAction.MOVE_FORWARD_ONE_CELL:
            self._updateForwardAction()
        elif self._currentAction in (
            MotionAction.TURN_LEFT_90,
            MotionAction.TURN_RIGHT_90,
        ):
            self._updateTurnAction()

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
        theta = self._getCompassTheta()
        if theta is None:
            # Fallback to internal belief
            _, _, theta = self._worldPose
        return (cos(theta), sin(theta))

    """
    Get the robot's current heading snapped to the nearest cardinal direction.

    Initially you can just return self._currentDirection; later you can
    derive it from the heading vector with a tolerance.
    """

    def getHeadingDirection(self) -> Direction:
        return self._currentDirection

    # ------------------------------------------------------------------
    # Atomic movement requests
    # ------------------------------------------------------------------

    """
    Request: move forward exactly one maze cell.
    """

    def requestMoveForwardOneCell(self) -> None:
        if self._state == RobotState.EXECUTING_ACTION:
            # For now, ignore if already busy. Later you could raise or log.
            return

        self._state = RobotState.EXECUTING_ACTION
        self._currentAction = MotionAction.MOVE_FORWARD_ONE_CELL
        self._lastActionResult = ActionResult.NONE

        # Compute the discrete target cell and its world centre.
        targetCell = self._getForwardCell()  # based on currentCell + direction
        targetX, targetY = self._cellToWorld(targetCell)
        _, _, theta = self._worldPose
        self._targetPose = (targetX, targetY, theta)
        self._targetCell = targetCell

        # mark the values of the encoders at the start
        self._startLeftEnc = self._leftEncoder.getValue()
        self._startRightEnc = self._rightEncoder.getValue()

        # - start motors in the right direction
        # Compute base forward speed once for this action
        self._baseForwardSpeed = self._baseForwardSpeedFrac * self._maxSpeed
        self._leftMotor.setVelocity(self._baseForwardSpeed)
        self._rightMotor.setVelocity(self._baseForwardSpeed)

    """
    Request: turn 90 degrees left (counter-clockwise).
    """

    def requestTurnLeft90(self) -> None:
        self._requestTurn90(MotionAction.TURN_LEFT_90)

    def senseLocalPassages(self):
        # Debug
        # print("Raw Sensors: ", self._readIrRaw())
        # print("Front Sensor: ", self._frontSensor.getValue())
        frontSensorIndices = [0, 7]
        leftSensorsIndices = [5]
        rightSensorsIndices = [2]
        isFrontBlocked = (
            True
            if (
                (
                    max(self._irSensors[i].getValue() for i in frontSensorIndices)
                    > IR_SESNOR_WALL_THRESHOLD
                )
                or (self._frontSensor.getValue() < FRONT_SESNOR_WALL_THRESHOLD)
            )
            else None
        )
        isLeftBlocked = (
            True
            if (
                max(self._irSensors[i].getValue() for i in leftSensorsIndices)
                > IR_SESNOR_WALL_THRESHOLD
            )
            else None
        )
        isRightBlocked = (
            True
            if (
                max(self._irSensors[i].getValue() for i in rightSensorsIndices)
                > IR_SESNOR_WALL_THRESHOLD
            )
            else None
        )

        currentDir = self._currentDirection
        # Debug
        # print(
        #     "Heading after turn: ",
        #     self._currentDirection,
        #     "int=",
        #     int(self._currentDirection),
        #     "name=",
        #     self._currentDirection.name,
        # )
        order = [Direction.EAST, Direction.NORTH, Direction.WEST, Direction.SOUTH]
        idx = order.index(currentDir)

        frontDir = currentDir
        leftDir = order[(idx + 1) % 4]
        rightDir = order[(idx - 1) % 4]
        backDir = order[(idx + 2) % 4]

        return {
            frontDir: isFrontBlocked,
            leftDir: isLeftBlocked,
            rightDir: isRightBlocked,
            backDir: None,
        }

    """
    Request: turn 90 degrees right (clockwise).
    """

    def requestTurnRight90(self) -> None:
        self._requestTurn90(MotionAction.TURN_RIGHT_90)

    def _requestTurn90(self, action: MotionAction) -> None:
        if self._state == RobotState.EXECUTING_ACTION:
            return

        self._state = RobotState.EXECUTING_ACTION
        self._currentAction = action
        self._lastActionResult = ActionResult.NONE

        # +1 for left, -1 for right
        self._turnSign = 1 if action == MotionAction.TURN_LEFT_90 else -1

        # Store encoders at the start of the turn
        self._startLeftEnc = self._leftEncoder.getValue()
        self._startRightEnc = self._rightEncoder.getValue()

        # In-place turn: one wheel forward, the other backward
        leftSpeed = -self._turnSign * self._turnSpeed
        rightSpeed = self._turnSign * self._turnSpeed

        self._leftMotor.setVelocity(leftSpeed)
        self._rightMotor.setVelocity(rightSpeed)

    """
    Update logic for MOVE_FORWARD_ONE_CELL.
    stops when the target cell centre is reached or slightly overshot.
    """

    def _updateForwardAction(self) -> None:
        # 1. Read encoders
        l = self._leftEncoder.getValue()
        r = self._rightEncoder.getValue()

        dl = (l - self._startLeftEnc) * WHEEL_RADIUS
        dr = (r - self._startRightEnc) * WHEEL_RADIUS

        # Average distance travelled
        dist = 0.5 * (dl + dr)

        # 2. Simple straightness correction (odometry-only)
        wheel_delta = dr - dl  # >0 ⇒ right travelled further than left
        K = 5.0
        correction = K * wheel_delta

        left = self._baseForwardSpeed - correction
        right = self._baseForwardSpeed + correction

        # Clamp to motor limits
        left = max(-self._maxSpeed, min(self._maxSpeed, left))
        right = max(-self._maxSpeed, min(self._maxSpeed, right))

        self._leftMotor.setVelocity(left)
        self._rightMotor.setVelocity(right)

        # 3. Check if we've gone (almost) one cell
        if dist < (self._cellSize - POSITION_TOLERANCE):
            return

        # 4. Movement complete: snap to target cell and world pose
        self._leftMotor.setVelocity(0.0)
        self._rightMotor.setVelocity(0.0)

        self._currentCell = (
            self._targetCell
        )  # already computed in requestMoveForwardOneCell

        targetX, targetY, targetTheta = self._targetPose
        self._worldPose = (targetX, targetY, targetTheta)

        self._finishAction(ActionResult.SUCCESS)

        # Debug
        # print("Finished Action Forward: cell", self._currentCell)

    """
    Update logic for 90° turn actions (left or right).

    Uses wheel encoder odometry to estimate heading change and stops when
    the rotation magnitude is approximately 90° (within ANGLE_TOLERANCE).
    """

    def _updateTurnAction(self) -> None:
        # Read encoders
        l = self._leftEncoder.getValue()
        r = self._rightEncoder.getValue()

        dl = (l - self._startLeftEnc) * WHEEL_RADIUS
        dr = (r - self._startRightEnc) * WHEEL_RADIUS

        # Approximate heading change for differential drive
        # Positive dtheta = CCW (left turn), negative = CW (right turn)
        dtheta = (dr - dl) / TRACK_WIDTH  # signed heading change
        turned = abs(dtheta)  # magnitude of rotation
        target = 0.5 * pi  # 90 degrees

        # Debug
        # print(f"[EPuckFacade] turn action={self._currentAction.name} dtheta={dtheta:.3f} rad")

        # Have we turned (roughly) 90 degrees?
        if abs(turned - target) > ANGLE_TOLERANCE:
            # Still too far from 90°, keep turning
            return

        # Stop motors
        self._leftMotor.setVelocity(0.0)
        self._rightMotor.setVelocity(0.0)

        # Rotate discrete heading by the same sign as the turn
        order = [Direction.EAST, Direction.NORTH, Direction.WEST, Direction.SOUTH]
        idx = order.index(self._currentDirection)
        idx = (idx + self._turnSign) % 4
        self._currentDirection = order[idx]

        # Snap worldPose theta to the exact cardinal direction
        x, y, _oldTheta = self._worldPose
        self._worldPose = (x, y, _directionToWorldTheta(self._currentDirection))

        self._finishAction(ActionResult.SUCCESS)

        # Debug
        # print(f"[TURN] action={self._currentAction.name} dtheta={dtheta:.3f} rad")

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
        self._setWorldPose()

    # ------------------------------------------------------------------
    # Optional world-pose accessors
    # ------------------------------------------------------------------

    def _setWorldPose(self) -> None:
        # --- Continuous pose (world coordinates) ---
        # Start exactly at the centre of the start cell.
        (row, col) = self._currentCell
        (originX, originY) = self._mazeOrigin

        worldX = originX + col * self._cellSize
        worldY = originY - row * self._cellSize
        theta = _directionToWorldTheta(self._currentDirection)

        self._worldPose = (worldX, worldY, theta)

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
    # Internal helpers
    # ------------------------------------------------------------------

    """
    Finish the current action with the given result and stop the motors.

    This is a small convenience to keep update() tidy.
    """

    def _finishAction(self, result: ActionResult) -> None:
        self._lastActionResult = result
        self._state = RobotState.IDLE
        self._currentAction = None
        self._leftMotor.setVelocity(0.0)
        self._rightMotor.setVelocity(0.0)

    """
    Get the maze cell one step ahead of the current cell.

    This helper uses the robot's current discrete pose
    (self._currentCell and self._currentDirection) to compute
    the neighbour cell in front of the robot:

    - NORTH: (row - 1, col)
    - SOUTH: (row + 1, col)
    - EAST:  (row, col + 1)
    - WEST:  (row, col - 1)

    It does not perform any bounds checking; callers are responsible
    for ensuring that the returned cell is inside the maze.

    @return Cell one step forward as (row, col).
    """

    def _getForwardCell(self) -> Cell:
        r, c = self._currentCell
        dir = self._currentDirection
        if dir == Direction.NORTH:
            r -= 1
        elif dir == Direction.SOUTH:
            r += 1
        elif dir == Direction.EAST:
            c += 1
        else:
            c -= 1
        return (r, c)

    """
    Convert a maze cell (row, col) to the world coordinates of its centre.

    The mapping assumes:
    - self._mazeOrigin = (originX, originY) is the centre of cell (0, 0).
    - Columns increase to the right → +X direction.
    - Rows increase downward in the maze → -Y direction.

    Therefore:
        x = originX + col * cellSize
        y = originY - row * cellSize

    @param cell Maze cell as (row, col).
    @return World position (x, y) of the cell centre.
    """

    def _cellToWorld(self, cell: Cell) -> Vec2:
        xWorld = self._mazeOrigin[0] + cell[1] * self._cellSize
        yWorld = self._mazeOrigin[1] - cell[0] * self._cellSize
        return (xWorld, yWorld)

    def _readIrRaw(self) -> list[float]:
        return [s.getValue() for s in self._irSensors]
