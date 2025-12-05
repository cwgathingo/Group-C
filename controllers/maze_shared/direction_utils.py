from typing import Tuple, TYPE_CHECKING

if TYPE_CHECKING:
    from maze.maze import Direction

"""
Shared helpers for working with maze Directions.
Provides deltas for grid movement and rotation utilities to keep
direction arithmetic consistent across modules.
"""


"""
Return the row/col delta for a given maze direction.

@param direction: Direction enum value.
@return: Tuple (dRow, dCol) to move one step in that direction.
"""


def getDirectionDelta(direction: "Direction") -> Tuple[int, int]:

    # Lazy import prevents maze.py -> direction_utils -> maze.py cycles
    # and avoids hard-coded package paths when loaded by controllers.
    from maze.maze import Direction

    if direction == Direction.NORTH:
        return -1, 0
    if direction == Direction.EAST:
        return 0, 1
    if direction == Direction.SOUTH:
        return 1, 0
    return 0, -1


"""
Rotate a direction by a number of 90-degree steps counter-clockwise.

Positive steps rotate left (CCW), negative steps rotate right (CW).

@param direction: Starting Direction.
@param steps: Number of quarter-turns counter-clockwise (can be negative).
@return: Rotated Direction.
"""


def rotateDirectionCounterClockwise(direction: "Direction", steps: int) -> "Direction":

    from maze.maze import Direction

    return Direction((int(direction) - steps) % 4)
