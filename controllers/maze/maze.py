from enum import Enum, IntEnum, auto
from typing import Tuple, List, Dict, Optional

Cell = Tuple[int, int]  # (row, col)


"""
Represents the four cardinal directions around a cell.
"""
class Direction(IntEnum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3


"""
Tri-state representation of a passage between two cells.
"""
class PassageState(Enum):
    UNKNOWN = auto()
    OPEN = auto()
    BLOCKED = auto()


"""
Maintains the robot's belief about the maze layout.

Stores:
- Maze dimensions and start/goal cells.
- For each cell and direction, whether the passage is UNKNOWN, OPEN or BLOCKED.
- Whether each cell has been visited.

Provides:
- Query methods for beliefs about passages and visited cells.
- Update methods to incorporate new sensor information.
- An ASCII export for reporting and debugging.
"""
class Maze:

    """
    Initialise the maze belief.

    - Store rows, cols, start and goal.
    - Initialise all internal passages as UNKNOWN.
    - Mark all outer-border passages as BLOCKED.
    - Initialise all cells as not visited.

    @param rows Number of maze rows (>= 1).
    @param cols Number of maze columns (>= 1).
    @param start Starting cell as (row, col).
    @param goal Goal cell as (row, col).
    """
    def __init__(self, rows: int, cols: int, start: Cell, goal: Cell) -> None:

        # Store basic configuration
        self._rows = rows
        self._cols = cols
        self._start = start
        self._goal = goal

        # Initialise visited matrix
        # visited[r][c] = False
        self._visited: List[List[bool]] = [
            [False for _ in range(cols)] for _ in range(rows)
        ]

        # Initialise passages:
        # passages[r][c][direction] = UNKNOWN for all internal cells
        self._passages: List[List[Dict[Direction, PassageState]]] = []
        for r in range(rows):
            row_list = []
            for c in range(cols):
                row_list.append({
                    Direction.NORTH: PassageState.UNKNOWN,
                    Direction.EAST:  PassageState.UNKNOWN,
                    Direction.SOUTH: PassageState.UNKNOWN,
                    Direction.WEST:  PassageState.UNKNOWN,
                })
            self._passages.append(row_list)

        # ---------------------------------------------------------
        # Mark all outer-border passages as BLOCKED
        #
        # Top row:    NORTH is blocked
        # Bottom row: SOUTH is blocked
        # Left col:   WEST is blocked
        # Right col:  EAST is blocked
        # ---------------------------------------------------------

        # Top row
        for c in range(cols):
            self._passages[0][c][Direction.NORTH] = PassageState.BLOCKED

        # Bottom row
        for c in range(cols):
            self._passages[rows - 1][c][Direction.SOUTH] = PassageState.BLOCKED

        # Left column
        for r in range(rows):
            self._passages[r][0][Direction.WEST] = PassageState.BLOCKED

        # Right column
        for r in range(rows):
            self._passages[r][cols - 1][Direction.EAST] = PassageState.BLOCKED
    
    
    # ---------- Basic queries (read-only) ----------

    """
    Get the maze dimensions.

    @return Tuple (rows, cols).
    """
    def getShape(self) -> Tuple[int, int]:
        return (self._rows, self._cols)

    """
    Get the start cell.

    @return Start cell as (row, col).
    """
    def getStart(self) -> Cell:
        return self._start

    """
    Get the goal cell.

    @return Goal cell as (row, col).
    """
    def getGoal(self) -> Cell:
        return self._goal

    """
    Check if a cell is inside the maze bounds.

    @param cell Cell to check (row, col).
    @return True if 0 <= row < rows and 0 <= col < cols, False otherwise.
    """
    def inBounds(self, cell: Cell) -> bool:
        (row, col) = cell
        return 0 <= row < self._rows and 0 <= col < self._cols

    """
    Get the belief about the passage from a cell in a given direction.

    @param cell Source cell (row, col).
    @param direction Direction of the passage.
    @return PassageState.UNKNOWN, OPEN, or BLOCKED.
    """
    def getPassage(self, cell: Cell, direction: Direction) -> PassageState:
        (row, col) = cell
        return self._passages[row][col][direction]

    """
    Get the beliefs about all four passages around a cell.

    @param cell Cell to query (row, col).
    @return Dictionary mapping Direction to PassageState.
    """
    def getAllPassages(self, cell: Cell) -> Dict[Direction, PassageState]:
        (row, col) = cell
        return self._passages[row][col]

    """
    Check whether a cell has ever been visited.

    @param cell Cell to query (row, col).
    @return True if the cell has been marked visited, False otherwise.
    """
    def isVisited(self, cell: Cell) -> bool:
        (row, col) = cell
        return self._visited[row][col]

    """
    Get the neighbouring cell in a given direction.

    If moving from the given cell in the given direction would leave the maze
    bounds, this method returns None.

    @param cell Source cell (row, col).
    @param direction Direction in which to look for a neighbour.
    @return Neighbour cell as (row, col), or None if outside the maze bounds.
    """
    def getNeighbour(self, cell: Cell, direction: Direction) -> Optional[Cell]:
        (row, col) = cell

        if direction == Direction.NORTH:
            neighbour = (row - 1, col)
        elif direction == Direction.EAST:
            neighbour = (row, col + 1)
        elif direction == Direction.SOUTH:
            neighbour = (row + 1, col)
        else:  # Direction.WEST
            neighbour = (row, col - 1)

        if self.inBounds(neighbour):
            return neighbour
        return None

    """
    Get all neighbouring cells around a cell.

    The neighbours are returned in the fixed order:
    [NORTH, EAST, SOUTH, WEST].

    For each direction, the list contains either the neighbouring cell
    (row, col) if it is inside the maze bounds, or None otherwise.

    @param cell Cell to query (row, col).
    @return List of 4 neighbours in order [NORTH, EAST, SOUTH, WEST].
    """
    def getNeighbours(self, cell: Cell) -> List[Optional[Cell]]:
        return [
            self.getNeighbour(cell, Direction.NORTH),
            self.getNeighbour(cell, Direction.EAST),
            self.getNeighbour(cell, Direction.SOUTH),
            self.getNeighbour(cell, Direction.WEST),
        ]
    
    """
    Get the opposite of a given direction.

    The directions are arranged in the order:
    NORTH (0), EAST (1), SOUTH (2), WEST (3).

    Because the Direction enum is defined as an IntEnum with values 0–3,
    the opposite direction can be computed by adding 2 and taking modulo 4.

    For example:
    - NORTH → SOUTH
    - EAST  → WEST
    - SOUTH → NORTH
    - WEST  → EAST

    @param direction The direction whose opposite should be returned.
    @return The opposite Direction value.
    """
    def getOppositeDirection(self, direction: Direction) -> Direction:
        return Direction((direction + 2) % 4)
    

    # ---------- Mapping updates (belief updates) ----------

    """
    Mark the passage in a direction from a cell as OPEN or BLOCKED.

    Also updates the symmetric passage in the neighbouring cell
    if the neighbour is inside the maze.

    @param cell Source cell (row, col).
    @param direction Direction of the passage to update.
    @param passageState New passage state (typically OPEN or BLOCKED).
    """
    def markPassageState(self, cell: Cell, direction: Direction, passageState: PassageState) -> None:
        nCell = self.getNeighbour(cell, direction)

        # Neighbour is outside the maze bounds
        if nCell is None:
            print('Warning: Trying to set out of bound passage from cell:', cell, 'direction:', direction, '\nExiting')
            return

        (row, col) = cell
        (nRow, nCol) = nCell

        # Do not overwrite an already-known passage
        if self._passages[row][col][direction] != PassageState.UNKNOWN:
            print('Warning: Trying to set a passage that has already been set:',
                cell, direction, self._passages[row][col][direction], '\nExiting')
            return

        # Update this passage
        self._passages[row][col][direction] = passageState

        # Update the symmetric passage in the neighbour
        opposite = self.getOppositeDirection(direction)
        self._passages[nRow][nCol][opposite] = passageState



    """
    Mark a cell as visited in the robot's belief.

    @param cell Cell to mark visited (row, col).
    """
    def mark_visited(self, cell: Cell) -> None:
        (row, col) = cell
        self._visited[row][col] = True

    # ---------- ASCII export ----------

    """
    Export the current maze belief as an ASCII map.

    Layout:
    - Each maze cell is rendered as a 5x3 block of characters.
    - The borders of the block reflect the PassageState values:
      UNKNOWN, OPEN, or BLOCKED.
    - The block centre shows:
      'S' for start, 'G' for goal, 'V' for visited, '?' for unvisited.

    @return List of strings, each string is a row of the ASCII map.
    """
    def export_ascii_map(self) -> List[str]:
        pass
