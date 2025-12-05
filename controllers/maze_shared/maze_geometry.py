from typing import Tuple

"""
Shared geometry helpers for mapping maze cells and walls to Webots world
coordinates. Assumes the maze origin is at the centre of cell (0, 0) with
columns increasing to +x and rows increasing downward (negative y).
"""

"""
Compute the world (x, y) coordinates of the centre of a maze cell.

@param cell: Maze cell as (row, col).
@param origin: World (x, y) position of cell (0, 0) centre.
@param cellSizeMeters: Length of a maze cell edge in meters.
@return: Tuple (x, y) of the cell centre in world coordinates.
"""


def getCellCenterWorld(
    cell: Tuple[int, int],
    origin: Tuple[float, float],
    cellSizeMeters: float,
) -> Tuple[float, float]:

    row, col = cell
    xWorld = origin[0] + col * cellSizeMeters
    yWorld = origin[1] - row * cellSizeMeters
    return xWorld, yWorld


"""
Position of a vertical wall segment between (row, col) and (row, col+1).

@param row: Row index of the wall boundary.
@param col: Column index of the wall boundary.
@param origin: World (x, y) position of cell (0, 0) centre.
@param cellSizeMeters: Length of a maze cell edge in meters.
@return: Tuple (x, y) translation for the wall segment.
"""


def getVerticalWallPosition(
    row: int,
    col: int,
    origin: Tuple[float, float],
    cellSizeMeters: float,
) -> Tuple[float, float]:

    xWorld = origin[0] + (col + 0.5) * cellSizeMeters
    yWorld = origin[1] - row * cellSizeMeters
    return xWorld, yWorld


"""
Position of a horizontal wall segment between (row, col) and (row+1, col).

@param row: Row index of the wall boundary.
@param col: Column index of the wall boundary.
@param origin: World (x, y) position of cell (0, 0) centre.
@param cellSizeMeters: Length of a maze cell edge in meters.
@return: Tuple (x, y) translation for the wall segment.
"""


def getHorizontalWallPosition(
    row: int,
    col: int,
    origin: Tuple[float, float],
    cellSizeMeters: float,
) -> Tuple[float, float]:

    xWorld = origin[0] + col * cellSizeMeters
    yWorld = origin[1] - (row + 0.5) * cellSizeMeters
    return xWorld, yWorld
