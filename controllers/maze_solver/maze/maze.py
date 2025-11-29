from enum import Enum, IntEnum, auto
from typing import Tuple, List, Dict, Optional

Cell = Tuple[int, int]  # (row, col)


"""
Represents the four cardinal directions used when reasoning about
neighbouring maze cells. Values are ordered clockwise starting at NORTH.
"""


class Direction(IntEnum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3


"""
Tri-state belief about whether a passage between two adjacent cells
contains a wall (BLOCKED), is traversable (OPEN), or remains unobserved
(UNKNOWN).
"""


class PassageState(Enum):
    UNKNOWN = auto()
    OPEN = auto()
    BLOCKED = auto()


"""
Represents a rectangular maze belief map used by the robot.

The Maze stores:
- The maze shape and the designated start and goal cells.
- A passage state (UNKNOWN / OPEN / BLOCKED) for each cell and direction.
- A visited flag for each cell, used by exploration and planning.

It provides:
- Methods for querying neighbours and passage beliefs.
- Methods for updating beliefs from sensor information.
- An ASCII export for debugging and reporting.

This class does not contain any sensor or motion logic; it only maintains
the robot's internal belief about the maze structure.
"""


class Maze:
    """
    Initialise the maze belief structure.

    All passages start as UNKNOWN, except border passages which are marked
    BLOCKED to prevent planning outside the maze. All cells begin unvisited.

    @param rows: Number of maze rows (>= 1).
    @param cols: Number of maze columns (>= 1).
    @param start: Starting cell as (row, col).
    @param goal: Goal cell as (row, col).
    """

    def __init__(self, rows: int, cols: int, start: Cell, goal: Cell) -> None:

        # Store maze dimensions and the designated start/goal cells.
        self._rows = rows
        self._cols = cols
        self._start = start
        self._goal = goal

        # Initialise visited[r][c] to track whether each cell has been explored.
        self._visited: List[List[bool]] = [
            [False for _ in range(cols)] for _ in range(rows)
        ]

        # Initialise passages[r][c][direction] as UNKNOWN for all cells.
        self._passages: List[List[Dict[Direction, PassageState]]] = []
        for r in range(rows):
            row_list = []
            for c in range(cols):
                row_list.append(
                    {
                        Direction.NORTH: PassageState.UNKNOWN,
                        Direction.EAST: PassageState.UNKNOWN,
                        Direction.SOUTH: PassageState.UNKNOWN,
                        Direction.WEST: PassageState.UNKNOWN,
                    }
                )
            self._passages.append(row_list)

        # Mark maze boundaries as BLOCKED so the planner never steps outside the grid.
        # Top row:    NORTH is blocked
        # Bottom row: SOUTH is blocked
        # Left column: WEST is blocked
        # Right column: EAST is blocked

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
    Return the maze dimensions.

    @return Tuple (rows, cols).
    """

    def getShape(self) -> Tuple[int, int]:
        return (self._rows, self._cols)

    """
    Return the designated start cell.

    @return Cell (row, col).
    """

    def getStart(self) -> Cell:
        return self._start

    """
    Return the designated goal cell.

    @return Cell (row, col).
    """

    def getGoal(self) -> Cell:
        return self._goal

    """
    Check whether a cell lies within the maze boundaries.

    @param cell: Cell to check (row, col).
    @return True if the cell is inside the maze, False otherwise.
    """

    def inBounds(self, cell: Cell) -> bool:
        (row, col) = cell
        return 0 <= row < self._rows and 0 <= col < self._cols

    """
    Return the belief state of the passage from a cell in a given direction.

    @param cell: Source cell (row, col).
    @param direction: Direction of the passage.
    @return PassageState for that passage.
    """

    def getPassage(self, cell: Cell, direction: Direction) -> PassageState:
        (row, col) = cell
        return self._passages[row][col][direction]

    """
    Return all four passage beliefs for the given cell.

    @param cell: Cell to query (row, col).
    @return Dictionary {Direction: PassageState}.
    """

    def getAllPassages(self, cell: Cell) -> Dict[Direction, PassageState]:
        (row, col) = cell
        return self._passages[row][col]

    """
    Check whether the robot has ever visited this cell.

    @param cell: Cell to query (row, col).
    @return True if the cell has been marked visited, False otherwise.
    """

    def isVisited(self, cell: Cell) -> bool:
        (row, col) = cell
        return self._visited[row][col]

    """
    Return the neighbouring cell in the given direction, or None if the move
    would leave the maze.

    @param cell: Starting cell (row, col).
    @param direction: Direction in which to look.
    @return Adjacent cell as (row, col), or None if outside bounds.
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
    Return neighbouring cells in all four directions.

    @param cell: Cell to query (row, col).
    @return Dictionary {Direction: Optional[Cell]} mapping each direction to
            a neighbouring cell or None if out of bounds.
    """

    def getNeighbours(self, cell: Cell) -> Dict[Direction, Optional[Cell]]:
        return {
            Direction.NORTH: self.getNeighbour(cell, Direction.NORTH),
            Direction.EAST: self.getNeighbour(cell, Direction.EAST),
            Direction.SOUTH: self.getNeighbour(cell, Direction.SOUTH),
            Direction.WEST: self.getNeighbour(cell, Direction.WEST),
        }

    """
    Return the direction directly opposite the given one.

    The Direction enum is an IntEnum ordered clockwise, so the opposite
    direction is obtained by (direction + 2) % 4.

    @param direction: Direction whose opposite is required.
    @return Opposite Direction.
    """

    def getOppositeDirection(self, direction: Direction) -> Direction:
        return Direction((direction + 2) % 4)

    # ---------- Mapping updates (belief updates) ----------

    """
    Update the belief about a passage from the given cell.

    Updates both:
    - the passage in the given direction, and
    - the symmetric passage in the neighbouring cell (if inside bounds).

    This method only updates passages currently marked UNKNOWN.

    @param cell: Source cell (row, col).
    @param direction: Direction of the passage.
    @param passageState: New state (typically OPEN or BLOCKED).
    """

    def markPassageState(
        self, cell: Cell, direction: Direction, passageState: PassageState
    ) -> None:
        nCell = self.getNeighbour(cell, direction)

        # Neighbour is outside the maze bounds
        if nCell is None:
            print(
                "Warning: ignoring markPassageState for out-of-bounds passage "
                "from cell",
                cell,
                "direction",
                direction,
            )
            return

        (row, col) = cell
        (nRow, nCol) = nCell
        currentState = self._passages[row][col][direction]

        # Do not overwrite an already-known passage
        if currentState != PassageState.UNKNOWN:
            print(
                "Warning: ignoring markPassageState; passage already known.",
                "cell",
                cell,
                "direction",
                direction,
                "current state",
                currentState,
                "requested state",
                passageState,
            )
            return

        # Update this passage
        self._passages[row][col][direction] = passageState

        # Update the symmetric passage in the neighbour
        opposite = self.getOppositeDirection(direction)
        self._passages[nRow][nCol][opposite] = passageState

    """
    Mark a cell as visited in the belief map.

    @param cell: Cell to mark (row, col).
    """

    def markVisited(self, cell: Cell) -> None:
        (row, col) = cell
        self._visited[row][col] = True

    # ---------- ASCII export ----------

    """
    Generate an ASCII representation of the maze belief.

    Each cell is rendered using a fixed 3-row block showing:
    - The passage state on each side:
        '1' = BLOCKED
        '0' = OPEN
        '?' = UNKNOWN
    - The centre character:
        'V' = visited
        '?' = unvisited

    Used for debugging and assignment reporting.

    @return List[str] representing the full ASCII map.
    """

    def exportAsciiMap(self) -> List[str]:
        asciiRows: List[str] = []

        for r in range(self._rows):
            rowTop = ""
            rowMiddle = ""
            rowBottom = ""

            for c in range(self._cols):

                # Determine centre character
                if self._visited[r][c]:
                    centre = "V"
                else:
                    centre = "?"

                # Convert each directional passage state (N, E, S, W) into a single character.
                chars: Dict[Direction, str] = {}
                for direction in Direction:
                    state = self._passages[r][c][direction]
                    if state == PassageState.BLOCKED:
                        chars[direction] = "1"
                    elif state == PassageState.OPEN:
                        chars[direction] = "0"
                    else:
                        chars[direction] = "?"

                north = chars[Direction.NORTH]
                east = chars[Direction.EAST]
                south = chars[Direction.SOUTH]
                west = chars[Direction.WEST]

                # Build the cell’s three-line ASCII block:
                #   top:    "  N   "
                #   middle: "W C E "
                #   bottom: "  S   "
                rowTop += "  " + north + "    "
                rowMiddle += west + " " + centre + " " + east + "  "
                rowBottom += "  " + south + "    "

            # Append the three lines for this maze row and a blank separator
            asciiRows.append(rowTop)
            asciiRows.append(rowMiddle)
            asciiRows.append(rowBottom)
            asciiRows.append("")

        return asciiRows

    """
    Print the ASCII representation of the current maze belief to the console.

    This is a convenience wrapper around exportAsciiMap().

    @return None
    """

    def printAsciiMap(self) -> None:
        asciiRows = self.exportAsciiMap()
        for row in asciiRows:
            print(row)
