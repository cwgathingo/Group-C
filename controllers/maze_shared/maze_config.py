# controllers/maze_config.py

from typing import Tuple

SEED = 1115

# Maze dimensions (in cells)
ROWS: int = 6
COLS: int = 6

# Cell size in meters
CELL_SIZE: float = 0.14


def compute_maze_origin() -> Tuple[float, float]:
    """
    Return the world (x, y) coordinates of the centre of cell (0, 0),
    assuming the world origin (0, 0) is at the centre of the maze.

    Convention:
      - Columns increase to the right (+x).
      - Rows increase downward (-y).
      - Cell (0, 0) is the top-left cell in maze coordinates.

    For a COLS x ROWS grid, the x,y of cell (0,0) centre are:

        origin_x = -((COLS - 1) * CELL_SIZE) / 2
        origin_y = +((ROWS - 1) * CELL_SIZE) / 2
    """
    origin_x = -((COLS - 1) * CELL_SIZE) / 2.0
    origin_y = +((ROWS - 1) * CELL_SIZE) / 2.0
    return origin_x, origin_y


# World position of cell (0, 0) centre (x, y)
MAZE_ORIGIN: Tuple[float, float] = compute_maze_origin()

# Default start/goal cells in maze coordinates (row, col)
# These are just defaults; we can override them later via the supervisor.
DEFAULT_START: Tuple[int, int] = (ROWS - 1, COLS - 1)  # bottom-right
DEFAULT_GOAL: Tuple[int, int] = (ROWS - 1, 0)  # bottom-left
