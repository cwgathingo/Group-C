from typing import Tuple

"""
Shared geometry helper for mapping maze cells to Webots world coordinates.
Assumes the maze origin is at the centre of cell (0, 0) with columns
increasing to +x and rows increasing downward (negative y).
"""


def getCellCenterWorld(
    cell: Tuple[int, int],
    origin: Tuple[float, float],
    cellSizeMeters: float,
) -> Tuple[float, float]:
    """
    Compute the world (x, y) coordinates of the centre of a maze cell.

    @param cell: Maze cell as (row, col).
    @param origin: World (x, y) position of cell (0, 0) centre.
    @param cellSizeMeters: Length of a maze cell edge in meters.
    @return: Tuple (x, y) of the cell centre in world coordinates.
    """
    row, col = cell
    xWorld = origin[0] + col * cellSizeMeters
    yWorld = origin[1] - row * cellSizeMeters
    return xWorld, yWorld
