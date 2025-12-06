# controllers/maze_solver/robots/epuck_facade.py

from typing import Tuple, Optional
from math import cos, pi, sin

from controller import Robot as WebotsRobot, Lidar as WebotsLidar
from maze_shared.logger import logDebug

from .robot_interface import (
    RobotFacade,
    RobotState,
    MotionAction,
    ActionResult,
    Vec2,
    Pose2D,
)
from maze.maze import (
    Cell,
    Direction,
)  # switch to relative imports later if desired
from maze_shared.maze_geometry import getCellCenterWorld
from maze_shared.direction_utils import (
    getDirectionDelta,
    rotateDirectionCounterClockwise,
)


# --- Calibrated odometry parameters ---
# Effective wheel radius and track width for the Webots e-puck model.
# These were tuned empirically so that encoder-based odometry matches
# the simulated forward motion and turn angles. They do NOT necessarily
# match the real e-puck's physical dimensions.
WHEEL_RADIUS = 0.02  # meters
TRACK_WIDTH = 0.057

# Position and heading tolerances used when checking motion completion.
POSITION_TOLERANCE = 0.0005
ANGLE_TOLERANCE = 0.03  # ~1.7 degrees

# Proximity sensor thresholds for treating readings as "wall detected".
IR_SENSOR_WALL_THRESHOLD = 80
FRONT_SENSOR_WALL_THRESHOLD = 900

"""
Convert a maze Direction into a world-frame orientation angle.

@param direction Maze heading to convert.
@return Heading angle in radians.
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
        perceptionMode: str = "lidar",  # "ir" or "lidar"
    ) -> None:

        self._robot = robot
        self._perceptionMode = perceptionMode
        self._lidar: Optional[WebotsLidar] = None
        self._lidarHRes: Optional[int] = None
        self._lidarFov: Optional[float] = None
        self._lidarAngles: list[float] = []
        self._lidarBlockRange: float = cellSizeMeters * 0.55

        # Maze geometry
        self._cellSize = cellSizeMeters
        self._mazeOrigin = mazeOriginWorld

        # Basic timestep (milliseconds to seconds) from Webots
        basicStepMs = int(self._robot.getBasicTimeStep())
        self._timeStepSeconds = (basicStepMs if basicStepMs > 0 else 32) / 1000.0

        # Devices (to be initialised downstream)
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

        # Enable IR Sensors
        self._irSensors = []
        for i in range(8):
            sensor = robot.getDevice(f"ps{i}")
            sensor.enable(basicStepMs)
            self._irSensors.append(sensor)
        self._frontSensor = robot.getDevice("distance sensor")
        self._frontSensor.enable(basicStepMs)

        # Lidar
        if self._perceptionMode == "lidar":
            self._lidar = robot.getDevice("lidar")
            self._lidar.enable(basicStepMs)
            self._lidar.enablePointCloud()
            self._lidarHRes = self._lidar.getHorizontalResolution()
            self._lidarFov = self._lidar.getFov()
            if self._lidarHRes and self._lidarFov:
                step = self._lidarFov / self._lidarHRes
                start = -self._lidarFov / 2.0
                for i in range(self._lidarHRes):
                    self._lidarAngles.append(start + (i + 0.5) * step)
            logDebug(
                "[LIDAR] init hres=%s fov=%.3f blockRange=%.3f"
                % (
                    self._lidarHRes,
                    self._lidarFov if self._lidarFov is not None else 0.0,
                    self._lidarBlockRange,
                )
            )

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

    @param timeStepSeconds Duration of this timestep in seconds.
    @return None
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

    For now a fallback is returned until the compass is fully wired.

    @return Heading unit vector (hx, hy).
    """

    def getHeadingVector(self) -> Vec2:
        theta = self._getCompassTheta()
        if theta is None:
            # Fallback to internal belief
            _, _, theta = self._worldPose
        return (cos(theta), sin(theta))

    """
    Get the robot's current heading snapped to the nearest cardinal direction.

    Initially this returns self._currentDirection; later it can be derived
    from the heading vector with a tolerance.

    @return Maze heading as Direction.
    """

    def getHeadingDirection(self) -> Direction:
        return self._currentDirection

    # ------------------------------------------------------------------
    # Atomic movement requests
    # ------------------------------------------------------------------

    """
    Request: move the robot forward exactly one maze cell.

    This sets up the internal state for a forward movement action:
      - marks the robot as executing an action,
      - determines the target maze cell based on the current heading,
      - computes the target world pose for the cell centre,
      - records the starting encoder values for odometry,
      - starts both wheel motors at the configured base speed.

    The actual motion is completed asynchronously by the update method
    (_updateForwardAction), which monitors encoder-based progress.

    @return None
    """

    def requestMoveForwardOneCell(self) -> None:
        if self._state == RobotState.EXECUTING_ACTION:
            # For now, ignore if already busy. A future version could raise or log.
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

    @return None
    """

    def requestTurnLeft90(self) -> None:
        self._requestTurn90(MotionAction.TURN_LEFT_90)

    """
    Sense nearby passages using the configured perception mode.

    When perceptionMode is \"lidar\", the lidar-based helper runs first
    and falls back to IR if unavailable. When perceptionMode is \"ir\",
    only the IR helper runs. Lidar returns True when a wall is likely,
    False when range exceeds the block threshold, and None when no
    usable reading exists. IR returns True when a wall is likely and
    None when the reading is inconclusive.

    @return Mapping Direction -> Optional[bool], where True indicates a
            detected wall, False indicates a clear passage (lidar only),
            and None indicates no reliable reading.
    """

    def senseLocalPassages(self):
        if self._perceptionMode == "lidar":
            return self._senseLocalPassagesLidar()

        return self._senseLocalPassagesIR()

    """
    Sense nearby passages using IR sensors only.

    The front, left and right proximity sensors, together with the
    dedicated forward distance sensor, contribute to wall detection
    relative to the current heading. Readings above calibrated thresholds
    are treated as positive evidence of a wall; lower readings yield no
    observation rather than a clear passage claim.

    @return Mapping Direction -> Optional[bool], where True indicates a
            detected wall and None indicates no reliable reading.
    """

    def _senseLocalPassagesIR(self):
        frontSensorIndices = [0, 7]
        leftSensorsIndices = [5]
        rightSensorsIndices = [2]
        isFrontBlocked = (
            True
            if (
                (
                    max(self._irSensors[i].getValue() for i in frontSensorIndices)
                    > IR_SENSOR_WALL_THRESHOLD
                )
                or (self._frontSensor.getValue() < FRONT_SENSOR_WALL_THRESHOLD)
            )
            else None
        )
        isLeftBlocked = (
            True
            if (
                max(self._irSensors[i].getValue() for i in leftSensorsIndices)
                > IR_SENSOR_WALL_THRESHOLD
            )
            else None
        )
        isRightBlocked = (
            True
            if (
                max(self._irSensors[i].getValue() for i in rightSensorsIndices)
                > IR_SENSOR_WALL_THRESHOLD
            )
            else None
        )

        currentDir = self._currentDirection

        frontDir = currentDir
        leftDir = rotateDirectionCounterClockwise(currentDir, 1)
        rightDir = rotateDirectionCounterClockwise(currentDir, -1)
        backDir = rotateDirectionCounterClockwise(currentDir, 2)

        return {
            frontDir: isFrontBlocked,
            leftDir: isLeftBlocked,
            rightDir: isRightBlocked,
            backDir: None,
        }

    """
    Sense nearby passages using the lidar sensor.

    Three lidar beams (left edge, centre, right edge) are sampled and
    their distances projected into forward and lateral components. Each
    direction is classified using a simple threshold test:
      - True   : obstacle within threshold (blocked),
      - False  : beam exceeds threshold (clear),
      - None   : no reliable reading.

    Results are mapped to maze-relative directions (front, left, right).
    The back direction is always None because the lidar provides no rear
    coverage.

    @return: Mapping Direction -> Optional[bool], where values indicate
    """

    def _senseLocalPassagesLidar(self):
        if self._lidar is None or self._lidarHRes is None:
            return self._senseLocalPassagesIR()

        ranges = self._lidar.getRangeImage() or []
        beamCount = len(ranges)
        if beamCount == 0:
            return self._senseLocalPassagesIR()

        leftIdx = 0
        centreIdx = beamCount // 2
        rightIdx = beamCount - 1

        def beam(idx: int) -> Tuple[float, float]:
            dist = ranges[idx]
            angle = self._lidarAngles[idx] if idx < len(self._lidarAngles) else 0.0
            return dist, angle

        leftDist, leftAngle = beam(leftIdx)
        centreDist, centerAngle = beam(centreIdx)
        rightDist, rightAngle = beam(rightIdx)

        def getBeamVector(dist: float, angle: float) -> tuple[float, float]:
            return dist * cos(angle), dist * sin(angle)

        centreForward, centreLateral = getBeamVector(centreDist, centerAngle)
        leftForward, leftLateral = getBeamVector(leftDist, leftAngle)
        rightForward, rightLateral = getBeamVector(rightDist, rightAngle)

        threshold = self._lidarBlockRange

        isFrontBlocked = (
            centreForward < threshold if centreForward is not None else None
        )
        isLeftBlocked = (
            abs(leftLateral) < threshold if leftLateral is not None else None
        )
        isRightBlocked = (
            abs(rightLateral) < threshold if rightLateral is not None else None
        )

        logDebug(
            "[LIDAR] beams idx L/C/R=%s/%s/%s dist=%.3f/%.3f/%.3f ang=%.3f/%.3f/%.3f "
            "proj(fwd,lat)= (%.3f,%.3f)/(%.3f,%.3f)/(%.3f,%.3f) threshold=%.3f"
            % (
                leftIdx,
                centreIdx,
                rightIdx,
                leftDist,
                centreDist,
                rightDist,
                leftAngle,
                centerAngle,
                rightAngle,
                leftForward,
                leftLateral,
                centreForward,
                centreLateral,
                rightForward,
                rightLateral,
                threshold,
            )
        )

        currentDir = self._currentDirection
        frontDir = currentDir
        leftDir = rotateDirectionCounterClockwise(currentDir, 1)
        rightDir = rotateDirectionCounterClockwise(currentDir, -1)
        backDir = rotateDirectionCounterClockwise(currentDir, 2)

        logDebug(
            f"[LIDAR] blocked? front={isFrontBlocked}, left={isLeftBlocked}, right={isRightBlocked}, back=None"
        )
        return {
            frontDir: isFrontBlocked,
            leftDir: isLeftBlocked,
            rightDir: isRightBlocked,
            backDir: None,  # no rear lidar coverage
        }

    """
    Request: turn 90 degrees right (clockwise).

    @return None
    """

    def requestTurnRight90(self) -> None:
        self._requestTurn90(MotionAction.TURN_RIGHT_90)

    """
    Internal helper to initiate a 90-degree turn action.

    This method:
      - marks the robot as executing an action,
      - stores the selected turn action (left or right),
      - records the starting encoder values for odometry,
      - sets the turn sign (+1 for left, -1 for right),
      - and commands an in-place rotation by driving the wheels in
        opposite directions at the configured turn speed.

    The turn is completed asynchronously by _updateTurnAction, which
    monitors the encoder-based heading change until approximately
    90 degrees of rotation is reached.

    @param action: MotionAction.TURN_LEFT_90 or MotionAction.TURN_RIGHT_90
    @return: None
    """

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

    Wheel encoder readings are converted to left and right travel
    distances (dl, dr) using the calibrated wheel radius. The forward
    displacement is estimated as:

        dist = 0.5 * (dl + dr)

    which corresponds to the standard differential-drive odometry model
    described in Siegwart, Nourbakhsh and Scaramuzza, Introduction to
    Autonomous Mobile Robots, Chapter 5, Section 5.2.4
    (Delta s = (Delta s_r + Delta s_l) / 2).

    During motion, a proportional correction term based on the difference
    between right and left wheel travel (dr - dl) adjusts the motor
    velocities to keep the robot approximately straight.

    Once the estimated travelled distance reaches one cell length (within
    POSITION_TOLERANCE), the motors are stopped, the robot's internal pose
    is snapped to the precomputed target cell centre, and the action is
    marked as complete.
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
        wheelDelta = dr - dl  # >0 => right travelled further than left
        K = 5.0
        correction = K * wheelDelta
        left = self._baseForwardSpeed + correction
        right = self._baseForwardSpeed - correction

        # Clamp to motor limits
        left = max(-self._maxSpeed, min(self._maxSpeed, left))
        right = max(-self._maxSpeed, min(self._maxSpeed, right))

        self._leftMotor.setVelocity(left)
        self._rightMotor.setVelocity(right)

        # 3. Check if the robot has travelled almost one cell
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
    Update logic for 90 degree turn actions (left or right).

    Wheel encoder readings are converted to left and right travel
    distances (dl, dr) using the calibrated wheel radius. The
    incremental heading change for the differential-drive robot is
    estimated as

        dtheta = (dr - dl) / TRACK_WIDTH,

    consistent with the standard odometry model for differential-drive
    robots (see Siegwart, Nourbakhsh and Scaramuzza, Introduction to
    Autonomous Mobile Robots, Chapter 5, Section 5.2.4).

    The controller continues commanding a turn until the magnitude of
    the estimated rotation |dtheta| is approximately 90 degrees
    (within ANGLE_TOLERANCE). At that point it:
      - stops both wheel motors,
      - updates the discrete maze-facing direction using the sign of
        the turn, and
      - snaps the stored world pose orientation to the corresponding
        cardinal heading.

    This keeps the internal pose representation aligned with the
    idealized grid while using encoder odometry only to detect when
    the 90 degree turn is complete.
    """

    def _updateTurnAction(self) -> None:
        # Read encoders
        l = self._leftEncoder.getValue()
        r = self._rightEncoder.getValue()

        dl = (l - self._startLeftEnc) * WHEEL_RADIUS
        dr = (r - self._startRightEnc) * WHEEL_RADIUS

        # Estimate heading change for differential drive.
        # Positive dtheta = CCW (left turn), negative = CW (right turn).
        dtheta = (dr - dl) / TRACK_WIDTH  # signed rotation
        turned = abs(dtheta)  # rotation magnitude
        target = 0.5 * pi  # 90 degrees

        # Debug
        # print(f"[EPuckFacade] turn action={self._currentAction.name} dtheta={dtheta:.3f} rad")

        # If still outside the [target +/- ANGLE_TOLERANCE] band,
        # the 90-degree turn is not complete yet, so continue turning.
        if abs(turned - target) > ANGLE_TOLERANCE:
            return

        # Stop motors (target reached)
        self._leftMotor.setVelocity(0.0)
        self._rightMotor.setVelocity(0.0)

        # Rotate discrete heading by the sign of the commanded turn
        self._currentDirection = rotateDirectionCounterClockwise(
            self._currentDirection, self._turnSign
        )

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

    @return True if an atomic action is in progress.
    """

    def isBusy(self) -> bool:
        return self._state == RobotState.EXECUTING_ACTION

    """
    Get the currently executing motion action, if any.

    @return Current MotionAction or None.
    """

    def getCurrentAction(self) -> Optional[MotionAction]:
        return self._currentAction

    """
    Get the result of the most recently completed action.

    @return Last ActionResult value.
    """

    def getLastActionResult(self) -> ActionResult:
        return self._lastActionResult

    # ------------------------------------------------------------------
    # Control / emergency behaviour
    # ------------------------------------------------------------------

    """
    Cancel any ongoing action and stop the robot safely.

    @return None
    """

    def cancelAction(self) -> None:
        # Stop motors immediately
        self._leftMotor.setVelocity(0.0)
        self._rightMotor.setVelocity(0.0)
        # TODO: wire this into an emergency stop path (e.g. if a sudden wall
        # is detected while moving forward) once higher-level detection exists.

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
    @return None
    """

    def resetPose(self, cell: Cell, direction: Direction) -> None:
        self._currentCell = cell
        self._currentDirection = direction
        self._setWorldPose()

    # ------------------------------------------------------------------
    # Optional world-pose accessors
    # ------------------------------------------------------------------

    """
    Compute and store the continuous world pose from the discrete state.

    @return None
    """

    def _setWorldPose(self) -> None:
        # --- Continuous pose (world coordinates) ---
        # Start exactly at the centre of the start cell.
        worldX, worldY = getCellCenterWorld(
            self._currentCell, self._mazeOrigin, self._cellSize
        )
        theta = _directionToWorldTheta(self._currentDirection)

        self._worldPose = (worldX, worldY, theta)

    """
    Optionally return the robot's precise world pose.

    For EPuckFacade this reflects the internal (x, y, theta) belief derived
    from the known start cell and subsequent motion. If the implementation
    stops maintaining a continuous pose, this may return None instead.

    @return Tuple (x, y, theta) or None.
    """

    def getWorldPose(self) -> Optional[Pose2D]:
        return self._worldPose

    """
    Optionally return the robot's world position only (x, y).

    @return Tuple (x, y) or None.
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

    @param result ActionResult to record.
    @return None
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
        dRow, dCol = getDirectionDelta(self._currentDirection)
        r, c = self._currentCell
        return (r + dRow, c + dCol)

    """
    Convert a maze cell (row, col) to the world coordinates of its centre.

    The mapping assumes:
    - self._mazeOrigin = (originX, originY) is the centre of cell (0, 0).
    - Columns increase to the right -> +X direction.
    - Rows increase downward in the maze -> -Y direction.

    Therefore:
        x = originX + col * cellSize
        y = originY - row * cellSize

    @param cell Maze cell as (row, col).
    @return World position (x, y) of the cell centre.
    """

    def _cellToWorld(self, cell: Cell) -> Vec2:
        return getCellCenterWorld(cell, self._mazeOrigin, self._cellSize)

    """
    Read raw IR sensor values.

    @return List of sensor readings.
    """

    def _readIrRaw(self) -> list[float]:
        return [s.getValue() for s in self._irSensors]
