# test_maze.py

from maze import Maze, Direction, PassageState, Cell


def testShapes() -> None:
    print("Running maze shape tests...")
    shapes = [
        (1, 1),
        (1, 5),
        (5, 1),
        (2, 2),
        (3, 4),
        (6, 3),
        (8, 8),
    ]

    for rows, cols in shapes:
        start = (0, 0)
        goal = (rows - 1, cols - 1)
        maze = Maze(rows, cols, start, goal)

        # Shape integrity
        assert maze.getShape() == (rows, cols), f"Wrong shape for {rows}x{cols}"

        # Start and goal
        assert maze.getStart() == start
        assert maze.getGoal() == goal

        # All border passages are BLOCKED
        for c in range(cols):
            assert maze.getPassage((0, c), Direction.NORTH) == PassageState.BLOCKED
            assert (
                maze.getPassage((rows - 1, c), Direction.SOUTH) == PassageState.BLOCKED
            )

        for r in range(rows):
            assert maze.getPassage((r, 0), Direction.WEST) == PassageState.BLOCKED
            assert (
                maze.getPassage((r, cols - 1), Direction.EAST) == PassageState.BLOCKED
            )

        # Non-border passages start UNKNOWN
        for r in range(rows):
            for c in range(cols):
                if r == 0 or r == rows - 1 or c == 0 or c == cols - 1:
                    continue  # border cell

                for d in [
                    Direction.NORTH,
                    Direction.EAST,
                    Direction.SOUTH,
                    Direction.WEST,
                ]:
                    assert maze.getPassage((r, c), d) == PassageState.UNKNOWN

    print("Shape tests passed.")


def testInBounds() -> None:
    print("Running inBounds tests...")

    rows = 4
    cols = 5
    maze = Maze(rows, cols, (0, 0), (3, 4))

    assert maze.inBounds((0, 0)) is True
    assert maze.inBounds((rows - 1, cols - 1)) is True
    assert maze.inBounds((-1, 0)) is False
    assert maze.inBounds((0, cols)) is False

    print("inBounds tests passed.")


def testNeighbours() -> None:
    print("Running neighbour tests...")

    rows = 4
    cols = 5
    maze = Maze(rows, cols, (0, 0), (3, 4))

    # Middle cell neighbours
    assert maze.getNeighbour((1, 1), Direction.NORTH) == (0, 1)
    assert maze.getNeighbour((1, 1), Direction.EAST) == (1, 2)
    assert maze.getNeighbour((1, 1), Direction.SOUTH) == (2, 1)
    assert maze.getNeighbour((1, 1), Direction.WEST) == (1, 0)

    # Out-of-bounds neighbours
    assert maze.getNeighbour((0, 0), Direction.NORTH) is None
    assert maze.getNeighbour((0, 0), Direction.WEST) is None

    # getNeighbours contents (dict)
    neighbours = maze.getNeighbours((1, 1))
    assert neighbours[Direction.NORTH] == (0, 1)
    assert neighbours[Direction.EAST] == (1, 2)
    assert neighbours[Direction.SOUTH] == (2, 1)
    assert neighbours[Direction.WEST] == (1, 0)

    print("Neighbour tests passed.")


def testOppositeDirection() -> None:
    print("Running opposite direction tests...")

    maze = Maze(2, 2, (0, 0), (1, 1))

    assert maze.getOppositeDirection(Direction.NORTH) == Direction.SOUTH
    assert maze.getOppositeDirection(Direction.SOUTH) == Direction.NORTH
    assert maze.getOppositeDirection(Direction.EAST) == Direction.WEST
    assert maze.getOppositeDirection(Direction.WEST) == Direction.EAST

    print("Opposite direction tests passed.")


def testMarkPassageState() -> None:
    print("Running markPassageState tests...")

    rows = 4
    cols = 5
    maze = Maze(rows, cols, (0, 0), (3, 4))

    cell: Cell = (1, 1)
    neighbour = maze.getNeighbour(cell, Direction.EAST)
    assert neighbour == (1, 2)

    # Initially UNKNOWN
    assert maze.getPassage(cell, Direction.EAST) == PassageState.UNKNOWN
    assert maze.getPassage(neighbour, Direction.WEST) == PassageState.UNKNOWN

    # Set passage to OPEN
    maze.markPassageState(cell, Direction.EAST, PassageState.OPEN)

    # This side should be OPEN
    assert maze.getPassage(cell, Direction.EAST) == PassageState.OPEN

    # Neighbour's WEST should also be OPEN
    assert maze.getPassage(neighbour, Direction.WEST) == PassageState.OPEN

    # Trying to set it again should not overwrite (your method just warns and returns)
    maze.markPassageState(cell, Direction.EAST, PassageState.BLOCKED)
    assert maze.getPassage(cell, Direction.EAST) == PassageState.OPEN
    assert maze.getPassage(neighbour, Direction.WEST) == PassageState.OPEN

    print("markPassageState tests passed.")


def testVisited() -> None:
    print("Running visited cell tests...")

    maze = Maze(4, 5, (0, 0), (3, 4))

    target: Cell = (2, 3)
    assert maze.isVisited(target) is False
    maze.markVisited(target)
    assert maze.isVisited(target) is True

    print("Visited cell tests passed.")


if __name__ == "__main__":
    testShapes()
    testInBounds()
    testNeighbours()
    testOppositeDirection()
    testMarkPassageState()
    testVisited()
    print("All tests passed.")
