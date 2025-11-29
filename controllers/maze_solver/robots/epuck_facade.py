# controllers/maze_solver/robots/epuck_facade.py

from typing import Tuple, Optional
from math import atan2, cos, hypot, pi, isnan, sin

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
POSITION_TOLERANCE = 0.0005
ANGLE_TOLERANCE = 0.02  # radians ≈ 1.7°

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
        self._leftMotor = robot.getDevice('left wheel motor')
        self._rightMotor = robot.getDevice('right wheel motor')

        # Enable velocity mode
        self._leftMotor.setPosition(float("inf"))
        self._rightMotor.setPosition(float("inf"))
        self._leftMotor.setVelocity(0.0)
        self._rightMotor.setVelocity(0.0)

        # set speed scalar
        self._baseForwardSpeedFrac = 0.3

        # Query max speeds
        self._maxLeftSpeed = self._leftMotor.getMaxVelocity()
        self._maxRightSpeed = self._rightMotor.getMaxVelocity()

        # For simplicity, use min of both
        self._maxSpeed = min(self._maxLeftSpeed, self._maxRightSpeed)

        print("[EPuckFacade] Max motor speeds:", self._maxLeftSpeed, self._maxRightSpeed)


        self._compass = robot.getDevice('compass')
        self._compass.enable(basicStepMs)
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

        # --- GPS calibration ---
        self._gpsOffset: Optional[Tuple[float, float]] = None
        self._gpsOffsetSamples: list[Tuple[float, float]] = []
        self._gpsCalibStepsRemaining: int = 10   # ~10 update cycles
        self._calibratingGps: bool = True

        # While calibrating, report as busy so planner doesn’t send actions
        self._state = RobotState.EXECUTING_ACTION

        # --- Turn tracking ---
        self._turnTargetTheta: Optional[float] = None
        self._targetDirection: Optional[Direction] = None
        self._turnSpeed: float = 0.2 * self._maxSpeed  # base turn speed
        self._turnSign: Optional[int] = None   # +1 = left turn, -1 = right turn

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

        # --- One-time GPS offset calibration phase ---
        if self._calibratingGps:
            if self._gpsCalibStepsRemaining > 0:
                gpsX, gpsY, _ = self._gps.getValues()
                if not isnan(gpsX) and not isnan(gpsY):
                    worldX, worldY, _theta = self._worldPose  # from _setWorldPose
                    offsetX = gpsX - worldX
                    offsetY = gpsY - worldY
                    self._gpsOffsetSamples.append((offsetX, offsetY))
                    print(
                        f"[EPuckFacade] GPS offset sample "
                        f"{len(self._gpsOffsetSamples)}: ({offsetX:.6f}, {offsetY:.6f})"
                    )

                self._gpsCalibStepsRemaining -= 1
                return  # stay "busy" during calibration

            # Done collecting samples → compute average offset
            if self._gpsOffsetSamples:
                n = len(self._gpsOffsetSamples)
                avgX = sum(s[0] for s in self._gpsOffsetSamples) / n
                avgY = sum(s[1] for s in self._gpsOffsetSamples) / n
                self._gpsOffset = (avgX, avgY)
                print("[EPuckFacade] Final GPS offset:", self._gpsOffset)

            self._calibratingGps = False
            self._gpsOffsetSamples.clear()
            self._state = RobotState.IDLE   # now planner can start issuing actions

            # fall through and do nothing else this cycle
            return

        if self._state != RobotState.EXECUTING_ACTION:
            # Nothing to do; keep motors stopped.
            return

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
    
    """
    Return GPS-based (x, y) corrected by the calibrated offset.

    If we don't yet have valid GPS or offset, returns None.
    """
    def _getCleanWorldXY(self) -> Optional[Tuple[float, float]]:
        if self._gpsOffset is None:
            return None

        gpsX, gpsY, _ = self._gps.getValues()

        print(
            f"[EPuckFacade] gpsX={gpsX} gpsY={gpsY}"
        )
        if isnan(gpsX) or isnan(gpsY):
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


        # Compute the discrete target cell and its world centre.
        targetCell = self._getForwardCell() # based on currentCell + direction
        print("targetCell", targetCell)
        targetX, targetY = self._cellToWorld(targetCell)
        _, _, theta = self._worldPose
        self._targetPose = (targetX, targetY, theta)
        self._targetCell = targetCell

        print("self._targetPose", self._targetPose)
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

        # Compute target theta = dirTheta ± π/2
        dTheta = self._turnSign * (0.5 * pi)
        dirTheta = _directionToWorldTheta(self._currentDirection)
        self._turnTargetTheta = self._wrapAngle(dirTheta + dTheta)

        # Apply motor speeds using sign
        leftSpeed  = -self._turnSign * self._turnSpeed
        rightSpeed =  self._turnSign * self._turnSpeed

        self._leftMotor.setVelocity(leftSpeed)
        self._rightMotor.setVelocity(rightSpeed)


    """
    Update logic for MOVE_FORWARD_ONE_CELL.

    Uses GPS (with offset) to estimate current (x, y), applies a small
    lateral steering correction to stay close to the ideal line, and
    stops when the target cell centre is reached or slightly overshot.
    """
    def _updateForwardAction(self) -> None:
        cleanPos = self._getCleanWorldXY()
        print("clean pos", cleanPos)
        if cleanPos is None:
            # No reliable GPS yet; keep driving, try again next tick.
            return

        currentX, currentY = cleanPos
        targetX, targetY, targetTheta = self._targetPose

        # Update internal world pose x,y; keep theta as before for now.
        _oldX, _oldY, currentTheta = self._worldPose
        self._worldPose = (currentX, currentY, currentTheta)

        # steering correction based on lateral error
        # Edit: It's too soft. Trying to do heading now.
        # lateralErr = self._computeLateralError(currentX, currentY)

        # ---------- HEADING-BASED STEERING + DEBUG ----------
        # Vector from current position to target.
        # EDIT: This also doesn't work. Dropping for now.
        # vx = targetX - currentX
        # vy = targetY - currentY
        # dist = hypot(vx, vy)

        # if dist > 1e-6:
        #     tx = vx / dist
        #     ty = vy / dist
        # else:
        #     # Already at target; steering not needed.
        #     tx, ty = 0.0, 0.0

        # # Desired forward direction from maze heading.
        # fx, fy = directionUnit[self._currentDirection]

        # # Signed heading error ~ sin(angle) between forward and target-dir.
        # # >0 → target lies to the LEFT of our forward axis.
        # # <0 → target lies to the RIGHT.
        # headingErr = fx * ty - fy * tx
        # self._applyHeadingCorrection(headingErr)
        # print(
        #     f"[FWD] cell={self._currentCell} dir={self._currentDirection.name} "
        #     f"pos=({currentX:.3f},{currentY:.3f}) target=({targetX:.3f},{targetY:.3f}) "
        #     f"v=({vx:.3f},{vy:.3f}) dist={dist:.4f} "
        #     f"t=({tx:.2f},{ty:.2f}) f=({fx:.2f},{fy:.2f}) "
        #     f"headingErr={headingErr:.4f} "
        # )
        
        # --- Existing target check / overshoot handling ---
        if not self._isAtTarget():
            # Not there yet; leave motors running (with corrected speeds).
            return

        # Movement complete: stop motors and finish.
        self._leftMotor.setVelocity(0.0)
        self._rightMotor.setVelocity(0.0)

        # Update discrete cell belief: we have moved one cell in currentDirection.
        self._currentCell = self._targetCell

        # Snap worldPose to the exact target.
        self._worldPose = (targetX, targetY, targetTheta)

        self._finishAction(ActionResult.SUCCESS)
        print("Finished Action Forward")

    """
    Update logic for 90° turn actions (left or right).

    Uses the compass to estimate current heading, compares to the
    desired cardinal heading, and stops when we are within a small
    angular tolerance.
    """
    def _updateTurnAction(self) -> None:
        if self._turnTargetTheta is None or self._turnSign is None:
            return

        theta = self._getCompassTheta()
        if theta is None:
            # No reliable compass yet; keep turning.
            return

        x, y, _oldTheta = self._worldPose
        self._worldPose = (x, y, theta)

        # Signed angular difference current → target in [-π, π]
        deltaTheta = self._wrapAngle(self._turnTargetTheta - theta)

        print(
            f"[EPuckFacade] turn action={self._currentAction.name} "
            f"theta={theta:.3f} target={self._turnTargetTheta:.3f} "
            f"deltaTheta={deltaTheta:.4f}"
        )

        if abs(deltaTheta) <= ANGLE_TOLERANCE:
            # Stop motors
            self._leftMotor.setVelocity(0.0)
            self._rightMotor.setVelocity(0.0)

            # Rotate discrete heading by the same sign as the turn
            order = [Direction.EAST, Direction.NORTH, Direction.WEST, Direction.SOUTH]
            idx = order.index(self._currentDirection)
            idx = (idx + self._turnSign) % 4
            self._currentDirection = order[idx]

            self._turnTargetTheta = None
            self._turnSign = None

            self._finishAction(ActionResult.SUCCESS)
            print("[EPuckFacade] Finished 90° turn")
            return
        
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
        # TODO: actually stop motors here.
        # self._leftMotor.setVelocity(0.0)
        # self._rightMotor.setVelocity(0.0)

        self._lastActionResult = result
        self._state = RobotState.IDLE
        self._currentAction = None

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
    

    """
    Check whether the robot has reached (or passed) the current target.

    Uses two criteria:

    1) Distance to target centre is within POSITION_TOLERANCE.
    2) Overshoot detection: if the target is *behind* the robot along the
       forward direction (dot product < 0), we consider the move complete
       even if distance is still > tolerance.

       This prevents forward motion from continuing forever after passing
       the target due to momentum, overshoot, or control inaccuracy.

    Current position is extracted from GPS and corrected by our offset.
    If GPS is not yet valid, returns False.
    """
    def _isAtTarget(self) -> bool:
        global POSITION_TOLERANCE

        global POSITION_TOLERANCE

        cleanPos = self._getCleanWorldXY()
        if cleanPos is None:
            return False

        currentX, currentY = cleanPos
        targetX, targetY, _ = self._targetPose

        # Vector from current → target
        vx = targetX - currentX
        vy = targetY - currentY

        # Distance
        dist = hypot(vx, vy)

        # Forward unit vector (from your directionUnit mapping)
        fx, fy = directionUnit[self._currentDirection]

        # Dot product determines overshoot
        dot = vx * fx + vy * fy

        # --- Debug output ---
        print(
            f"[EPuckFacade] isAtTarget? "
            f"dist={dist:.4f} "
            f"dot={dot:.4f} "
            f"pos=({currentX:.3f},{currentY:.3f}) "
            f"target=({targetX:.3f},{targetY:.3f}) "
            f"forward=({fx:.2f},{fy:.2f})"
        )

        # Condition 1: within distance tolerance
        if dist <= POSITION_TOLERANCE:
            return True

        # Condition 2: overshoot → target behind robot
        if dot < 0.0:
            return True

        return False
    
    """
    Compute signed lateral error from the ideal straight path between
    the start cell and the target cell for this forward move.

    The ideal path is the line through the centre of currentCell in the
    current maze heading direction. Positive error means the robot is
    displaced to one side of that line, negative to the other.

    @param currentX Current world X position (corrected GPS).
    @param currentY Current world Y position (corrected GPS).
    @return Lateral error in meters.
    """
    def _computeLateralError(self, currentX: float, currentY: float) -> float:
        # Ideal start point = centre of the current cell.
        startX, startY = self._cellToWorld(self._currentCell)

        # Displacement from start of this move.
        dx = currentX - startX
        dy = currentY - startY

        # Forward unit vector (maze heading, using your global map)
        fx, fy = directionUnit[self._currentDirection]

        # Perpendicular to forward (left-hand normal)
        # If forward = (fx, fy), one normal is n = (-fy, fx)
        nx = -fy
        ny = fx

        # Signed distance from ideal line
        lateralErr = dx * nx + dy * ny

        # DEBUG (optional)
        # Comment out when stable
        print(
            f"[EPuckFacade] lateralErr={lateralErr:.4f}m "
            f"pos=({currentX:.3f},{currentY:.3f}) "
            f"start=({startX:.3f},{startY:.3f}) "
            f"dxdy=({dx:.3f},{dy:.3f}) "
            f"forward=({fx:.2f},{fy:.2f}) "
            f"normal=({nx:.2f},{ny:.2f})"
        )

        # Signed lateral error: projection onto the normal
        return lateralErr
    
    """
    Apply a proportional steering correction based on lateral error.

    lateralErr is the signed distance from the ideal straight-line path:
    - lateralErr > 0 → robot is to the LEFT of the ideal line.
    - lateralErr < 0 → robot is to the RIGHT of the ideal line.

    We adjust left/right wheel speeds symmetrically around a base
    forward speed so that the robot steers back toward the path:

        left  = base + K * lateralErr
        right = base - K * lateralErr

    This means:
    - If lateralErr > 0 (too far left), the left wheel is faster than
      the right, causing a rightward turn back toward the path.
    - If lateralErr < 0 (too far right), the right wheel is faster than
      the left, causing a leftward turn.

    Speeds are clamped to the motor's max velocity.
    """
    def _applyLateralCorrection(self, lateralErr: float) -> None:
        base = self._baseForwardSpeed
        K = 50  # tune

        # If lateralErr > 0 we are "left" of the line.
        # To steer back, the LEFT wheel should be faster than the RIGHT.
        correction = K * lateralErr

        leftSpeed  = base - correction
        rightSpeed = base + correction

        # Clamp to motor limits
        leftSpeed = max(-self._maxSpeed, min(self._maxSpeed, leftSpeed))
        rightSpeed = max(-self._maxSpeed, min(self._maxSpeed, rightSpeed))

        self._leftMotor.setVelocity(leftSpeed)
        self._rightMotor.setVelocity(rightSpeed)

        print(
            f"[EPuckFacade] lateralErr={lateralErr:.4f} "
            f"base={base:.3f} left={leftSpeed:.3f} right={rightSpeed:.3f}"
        )

    """
    Apply a proportional steering correction based on heading error.

    headingErr is approximately sin(dTheta) between the robot's forward
    direction (maze heading) and the unit vector from current position
    to the target cell centre:

        headingErr > 0 → target lies to the LEFT of the forward axis.
        headingErr < 0 → target lies to the RIGHT.

    We steer back toward the target by asymmetrically adjusting speeds
    around a base forward speed:

        left  = base - K * headingErr
        right = base + K * headingErr

    This means:
    - If headingErr > 0 (target left), right wheel is faster → turn left.
    - If headingErr < 0 (target right), left wheel is faster → turn right.

    Returns the (leftSpeed, rightSpeed) actually commanded.
    """
    def _applyHeadingCorrection(self, headingErr: float) -> Tuple[float, float]:
        base = self._baseForwardSpeed  # set when the action starts

        # Start with a much stronger gain; tune down from here if it oscillates.
        K = 10.0 * self._maxSpeed

        correction = K * headingErr

        leftSpeed = base - correction
        rightSpeed = base + correction

        print(
            f"[EPuckFacade] correction={correction:.4f} base={base:.3f} "
            f"left={leftSpeed:.3f} right={rightSpeed:.3f}"
        )


        # Clamp to motor limits.
        if leftSpeed > self._maxSpeed:
            leftSpeed = self._maxSpeed
        if leftSpeed < -self._maxSpeed:
            leftSpeed = -self._maxSpeed

        if rightSpeed > self._maxSpeed:
            rightSpeed = self._maxSpeed
        if rightSpeed < -self._maxSpeed:
            rightSpeed = -self._maxSpeed

        self._leftMotor.setVelocity(leftSpeed)
        self._rightMotor.setVelocity(rightSpeed)

        print(
            f"[EPuckFacade] headingErr={headingErr:.4f} base={base:.3f} "
            f"left={leftSpeed:.3f} right={rightSpeed:.3f}"
        )

        return leftSpeed, rightSpeed
    
    """
    Read the compass and convert to a world-frame heading angle theta.

    Convention matches _directionToWorldTheta:
        EAST  → 0 rad
        NORTH → +π/2
        WEST  → π
        SOUTH → -π/2
    """
    def _getCompassTheta(self) -> Optional[float]:
        if self._compass is None:
            return None

        vals = self._compass.getValues()
        cx, _cy, cz = vals

        if isnan(cx) or isnan(cz):
            return None

        # Webots compass gives NORTH direction in robot frame as (x,z)
        # yaw_c = atan2(x, z), 0 when facing NORTH, +π/2 when facing EAST, etc.
        yaw_c = atan2(cx, cz)

        # Convert to our convention: EAST=0, NORTH=+π/2, WEST=π, SOUTH=-π/2
        theta = (0.5 * pi) - yaw_c

        # Wrap to [-π, π]
        if theta > pi:
            theta -= 2.0 * pi
        if theta < -pi:
            theta += 2.0 * pi

        return theta
    
    """
    Smallest signed angle from current → target, in [-π, π].
    """
    def _angleDiff(self, target: float, current: float) -> float:
        diff = target - current
        while diff > pi:
            diff -= 2.0 * pi
        while diff < -pi:
            diff += 2.0 * pi
        return diff
    
    """Wrap angle to [-π, π]."""
    def _wrapAngle(self, theta: float) -> float:
        return (theta + pi) % (2.0 * pi) - pi