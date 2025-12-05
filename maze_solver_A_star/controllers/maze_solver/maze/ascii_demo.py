# ascii_demo.py

from maze import Maze, Direction, PassageState


def demoBasicMaze():
    print("=== Basic 2x2 Maze ===")

    maze = Maze(2, 2, (0, 0), (1, 1))

    # Optional: modify some passages so you can see variation
    maze.markPassageState((0, 0), Direction.EAST, PassageState.OPEN)
    maze.markPassageState((0, 0), Direction.SOUTH, PassageState.BLOCKED)
    maze.markPassageState((1, 0), Direction.EAST, PassageState.OPEN)

    # Mark some visited cells
    maze.markVisited((0, 0))
    maze.markVisited((1, 0))

    # Print the ASCII output
    maze.printAsciiMap()


def demoLargerMaze():
    print("=== 3x3 Maze Demo ===")

    maze = Maze(3, 3, (0, 0), (2, 2))

    # Set a few known passages
    maze.markPassageState((0, 0), Direction.EAST, PassageState.OPEN)
    maze.markPassageState((0, 1), Direction.EAST, PassageState.OPEN)
    maze.markPassageState((0, 2), Direction.SOUTH, PassageState.OPEN)
    maze.markPassageState((1, 2), Direction.SOUTH, PassageState.OPEN)

    # Mark the resulting path visited
    maze.markVisited((0, 0))
    maze.markVisited((0, 1))
    maze.markVisited((0, 2))
    maze.markVisited((1, 2))
    maze.markVisited((2, 2))

    maze.printAsciiMap()


def demoFourByFourPath() -> None:
    print("=== 4x4 Maze with step-by-step robot movement and local sensing ===")

    maze = Maze(4, 4, (0, 0), (3, 3))

    # Robot path:
    path = [(0, 0), (0, 1), (1, 1), (2, 1)]

    # Ground-truth blocked passages (world model).
    # These are *real* walls in the environment, but the Maze only learns
    # about them when the robot is adjacent and "senses" them.
    groundTruthBlocked = [
        ((0, 2), Direction.SOUTH),
        ((1, 0), Direction.EAST),
        ((1, 2), Direction.EAST),
        ((2, 0), Direction.EAST),
        ((2, 2), Direction.SOUTH),
        ((3, 1), Direction.EAST),
    ]

    def isBlockedInWorld(cell, direction):
        return (cell, direction) in groundTruthBlocked

    # Only set walls the robot can currently sense (BLOCKED only).
    def senseAt(cell):
        for direction in [
            Direction.NORTH,
            Direction.EAST,
            Direction.SOUTH,
            Direction.WEST,
        ]:
            neighbour = maze.getNeighbour(cell, direction)
            if neighbour is None:
                # Outer border is already BLOCKED by Maze.__init__, no need to set.
                continue
            if isBlockedInWorld(cell, direction):
                maze.markPassageState(cell, direction, PassageState.BLOCKED)

    # Open passage between two adjacent cells in the path.
    def openPassageBetween(cellA, cellB):
        (r1, c1) = cellA
        (r2, c2) = cellB

        if r2 == r1 and c2 == c1 + 1:
            maze.markPassageState(cellA, Direction.EAST, PassageState.OPEN)
        elif r2 == r1 and c2 == c1 - 1:
            maze.markPassageState(cellA, Direction.WEST, PassageState.OPEN)
        elif c2 == c1 and r2 == r1 + 1:
            maze.markPassageState(cellA, Direction.SOUTH, PassageState.OPEN)
        elif c2 == c1 and r2 == r1 - 1:
            maze.markPassageState(cellA, Direction.NORTH, PassageState.OPEN)

    # Simulate robot movement step-by-step
    for stepIndex in range(len(path)):
        cell = path[stepIndex]

        print(f"\n--- Robot step {stepIndex + 1}: visiting {cell} ---")

        # 1. Sense local walls around current cell (BLOCKED only)
        senseAt(cell)

        # 2. Mark current cell as visited
        maze.markVisited(cell)

        # 3. Open passage to next cell on the path (if any)
        if stepIndex + 1 < len(path):
            nextCell = path[stepIndex + 1]
            openPassageBetween(cell, nextCell)

        # 4. Print current belief map
        maze.printAsciiMap()


if __name__ == "__main__":
    demoBasicMaze()
    print("\n")
    demoLargerMaze()
    print("\n")
    demoFourByFourPath()
