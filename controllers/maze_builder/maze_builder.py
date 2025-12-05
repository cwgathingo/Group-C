import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from controller import Supervisor
import random
from typing import Tuple, List, Set
from maze_shared.maze_config import ROWS, COLS, CELL_SIZE, MAZE_ORIGIN, SEED

Cell = Tuple[int, int]

# Maze dimensions - must match MazeController
# ROWS = 4
# COLS = 4

# Geometry - must match your controller setup:
#   cellSizeMeters = 0.15
#   mazeOriginWorld = (-0.225, 0.225)
# CELL_SIZE = 0.15
# ORIGIN_X = -0.225
# ORIGIN_Y = 0.225
ORIGIN_X, ORIGIN_Y = MAZE_ORIGIN


"""
Depth-first search (iterative) returning a set of open edges.

@param rows Number of maze rows.
@param cols Number of maze columns.
@param rng Random number generator to use.
@return Set of open edges between adjacent cells.
"""


def generatePerfectMaze(
    rows: int, cols: int, rng: random.Random
) -> Set[tuple[Cell, Cell]]:
    stack: List[Cell] = []
    visited = [[False] * cols for _ in range(rows)]
    openEdges: Set[tuple[Cell, Cell]] = set()

    def neighbours(cell: Cell):
        r, c = cell
        # Fixed neighbour order -> determinism given RNG
        for dr, dc in [(-1, 0), (0, 1), (1, 0), (0, -1)]:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                yield (nr, nc)

    start: Cell = (0, 0)
    stack.append(start)
    visited[start[0]][start[1]] = True

    while stack:
        current = stack[-1]
        unvisited = [n for n in neighbours(current) if not visited[n[0]][n[1]]]
        if not unvisited:
            stack.pop()
            continue

        nxt = rng.choice(unvisited)
        visited[nxt[0]][nxt[1]] = True

        edge = tuple(sorted([current, nxt]))
        openEdges.add(edge)

        stack.append(nxt)

    return openEdges


# ---------- Geometry helpers ----------


"""
Position of a vertical wall segment between (r, c) and (r, c+1).
For borders we allow c = -1 (left of col 0) and c = COLS-1 (right of last).

@param r Row index.
@param c Column index.
@return (x, y) translation for the wall.
"""


def verticalWallPos(r: int, c: int) -> Tuple[float, float]:
    x = ORIGIN_X + (c + 0.5) * CELL_SIZE
    y = ORIGIN_Y - r * CELL_SIZE
    return x, y


"""
Position of a horizontal wall segment between (r, c) and (r+1, c).
For borders we allow r = -1 (above row 0) and r = ROWS-1 (below last).

@param r Row index.
@param c Column index.
@return (x, y) translation for the wall.
"""


def horizontalWallPos(r: int, c: int) -> Tuple[float, float]:
    x = ORIGIN_X + c * CELL_SIZE
    y = ORIGIN_Y - (r + 0.5) * CELL_SIZE
    return x, y


"""
Remove all existing Wall nodes and create a rectangular frame of
Wall segments around the 4x4 cell grid.

This step ignores the maze connectivity; it's just to verify that
our coordinates and sizes are correct.

@param supervisor Webots Supervisor controlling the world.
@return None
"""


def buildBorderWalls(supervisor: Supervisor) -> None:
    root = supervisor.getRoot()
    children = root.getField("children")

    # 1) Remove any existing Wall nodes (including the ones from the .wbt)
    i = children.getCount() - 1
    removed = 0
    while i >= 0:
        node = children.getMFNode(i)
        if node is not None and node.getTypeName() == "Wall":
            children.removeMF(i)
            removed += 1
        i -= 1
    print(f"[maze_builder] removed {removed} existing Wall nodes")

    # 2) Add top and bottom border (horizontal walls)
    for c in range(COLS):
        # Top border: between "virtual row -1" and row 0
        x, y = horizontalWallPos(-1, c)
        children.importMFNodeFromString(
            -1,
            f"Wall {{ translation {x} {y} 0 size {CELL_SIZE} 0.01 0.05 }}",
        )

        # Bottom border: between row ROWS-1 and "virtual row ROWS"
        x, y = horizontalWallPos(ROWS - 1, c)
        children.importMFNodeFromString(
            -1,
            f"Wall {{ translation {x} {y} 0 size {CELL_SIZE} 0.01 0.05 }}",
        )

    # 3) Add left and right border (vertical walls)
    for r in range(ROWS):
        # Left border: between "virtual col -1" and col 0
        x, y = verticalWallPos(r, -1)
        children.importMFNodeFromString(
            -1,
            f"Wall {{ translation {x} {y} 0 size 0.01 {CELL_SIZE} 0.05 }}",
        )

        # Right border: between col COLS-1 and "virtual col COLS"
        x, y = verticalWallPos(r, COLS - 1)
        children.importMFNodeFromString(
            -1,
            f"Wall {{ translation {x} {y} 0 size 0.01 {CELL_SIZE} 0.05 }}",
        )

    print("[maze_builder] border walls created")


"""
Create internal walls based on the maze connectivity in openEdges.

For every pair of adjacent cells:
  - If the edge is in openEdges -> NO wall (passage).
  - Otherwise -> add a wall segment at the boundary between them.

@param supervisor Webots Supervisor controlling the world.
@param openEdges Set of passages that remain open between adjacent cells.
@return None
"""


def buildInternalWalls(
    supervisor: Supervisor, openEdges: Set[tuple[Cell, Cell]]
) -> None:
    root = supervisor.getRoot()
    children = root.getField("children")

    def isOpen(a: Cell, b: Cell) -> bool:
        return tuple(sorted([a, b])) in openEdges

    # --- Vertical internal walls (between (r,c) and (r,c+1)) ---
    for r in range(ROWS):
        for c in range(COLS - 1):
            a = (r, c)
            b = (r, c + 1)
            if not isOpen(a, b):
                x, y = verticalWallPos(r, c)
                children.importMFNodeFromString(
                    -1,
                    f"Wall {{ translation {x} {y} 0 size 0.01 {CELL_SIZE} 0.05 }}",
                )

    # --- Horizontal internal walls (between (r,c) and (r+1,c)) ---
    for r in range(ROWS - 1):
        for c in range(COLS):
            a = (r, c)
            b = (r + 1, c)
            if not isOpen(a, b):
                x, y = horizontalWallPos(r, c)
                children.importMFNodeFromString(
                    -1,
                    f"Wall {{ translation {x} {y} 0 size {CELL_SIZE} 0.01 0.05 }}",
                )

    print("[maze_builder] internal walls created")


"""
Find the RectangleArena node and update its size / tile size / wall height
based on ROWS, COLS, CELL_SIZE from the shared config.

@param supervisor Webots Supervisor controlling the world.
@return None
"""


def updateRectangleArena(supervisor: Supervisor) -> None:
    root = supervisor.getRoot()
    children = root.getField("children")

    arena_node = None
    for i in range(children.getCount()):
        node = children.getMFNode(i)
        if node is not None and node.getTypeName() == "RectangleArena":
            arena_node = node
            break

    if arena_node is None:
        print(
            "[maze_builder] WARNING: no RectangleArena node found; skipping arena update"
        )
        return

    # Compute desired sizes from config
    floorWidth = COLS * CELL_SIZE
    floorHeight = ROWS * CELL_SIZE

    tileWidth = 2.0 * CELL_SIZE  # so each checker ~ one maze cell visually
    tileHeight = 2.0 * CELL_SIZE

    # Update fields
    floorSizeField = arena_node.getField("floorSize")
    tileSizeField = arena_node.getField("floorTileSize")
    wallHeightField = arena_node.getField("wallHeight")

    if floorSizeField is not None:
        floorSizeField.setSFVec2f([floorWidth, floorHeight])

    if tileSizeField is not None:
        tileSizeField.setSFVec2f([tileWidth, tileHeight])

    if wallHeightField is not None:
        wallHeightField.setSFFloat(0.001)  # almost flat

    print(
        "[maze_builder] RectangleArena updated: "
        f"floorSize=({floorWidth:.3f}, {floorHeight:.3f}), "
        f"floorTileSize=({tileWidth:.3f}, {tileHeight:.3f}), "
        "wallHeight=0.001"
    )


"""
Convert a maze cell to its world centre coordinates.

@param cell Maze cell as (row, col).
@return Tuple (x, y) in world coordinates.
"""


def cellToWorldCenter(cell: Cell) -> tuple[float, float]:
    r, c = cell
    x = ORIGIN_X + c * CELL_SIZE
    y = ORIGIN_Y - r * CELL_SIZE
    return x, y


"""
Move MAZE_ROBOT to the centre of the given maze cell.

@param supervisor Webots Supervisor controlling the world.
@param cell Target cell (row, col).
@return None
"""


def moveRobotToCell(supervisor: Supervisor, cell: Cell) -> None:
    robotNode = supervisor.getFromDef("MAZE_ROBOT")
    if robotNode is None:
        print("[maze_builder] WARNING: MAZE_ROBOT not found; cannot move robot")
        return

    txField = robotNode.getField("translation")
    if txField is None:
        print("[maze_builder] WARNING: robot has no translation field")
        return

    x, y = cellToWorldCenter(cell)
    current = txField.getSFVec3f()
    z = current[2] if len(current) >= 3 else 0.0

    txField.setSFVec3f([x, y, z])
    print(f"[maze_builder] moved robot to cell {cell} at ({x:.3f}, {y:.3f})")


"""
Choose a random start cell on the bottom row and a random goal cell on the top row.
Both are guaranteed to be adjacent to opposite outer walls.

@param rng Random number generator.
@return Tuple (startCell, goalCell).
"""


def chooseStartAndGoal(rng: random.Random) -> tuple[Cell, Cell]:
    startRow = ROWS - 1  # bottom
    goalRow = 0  # top

    startCol = rng.randrange(COLS)
    goalCol = rng.randrange(COLS)

    startCell = (startRow, startCol)
    goalCell = (goalRow, goalCol)

    print(f"[maze_builder] chosen start={startCell}, goal={goalCell}")
    return startCell, goalCell


"""
Place a small visual marker (green cylinder) at the centre of the goal cell.

@param supervisor Webots Supervisor controlling the world.
@param goalCell Goal cell coordinates.
@return None
"""


def placeGoalMarker(supervisor: Supervisor, goalCell: Cell) -> None:
    root = supervisor.getRoot()
    children = root.getField("children")

    x, y = cellToWorldCenter(goalCell)

    node_string = f"""
Solid {{
  translation {x:.3f} {y:.3f} 0.02
  name "goal_marker"
  children [
    Shape {{
      appearance Appearance {{
        material Material {{
          diffuseColor 0 1 0
          emissiveColor 0 0.5 0
        }}
      }}
      geometry Cylinder {{
        radius 0.03
        height 0.02
      }}
    }}
  ]
}}
"""
    children.importMFNodeFromString(-1, node_string)
    print(f"[maze_builder] placed goal marker at cell {goalCell} at ({x:.3f}, {y:.3f})")


"""
Build the customData payload that will be written to MAZE_ROBOT.

Keys are separated by ';' and coordinates use 'row,col'.
Includes startDir to keep heading aligned with placement.

@param startCell Selected start cell.
@param goalCell Selected goal cell.
@param seed      Random seed used.
@return String payload for customData.
"""


def buildCustomDataPayload(startCell: Cell, goalCell: Cell, seed: int) -> str:
    startDir = "north"
    fields = [
        ("world_ready", "1"),
        ("start", f"{startCell[0]},{startCell[1]}"),
        ("goal", f"{goalCell[0]},{goalCell[1]}"),
        ("startDir", startDir),
        ("rows", str(ROWS)),
        ("cols", str(COLS)),
        ("cell_size", f"{CELL_SIZE}"),
        ("seed", str(seed)),
    ]
    return ";".join(f"{k}={v}" for k, v in fields) + ";"


"""
Render the simulation time onto the display.

@param display Webots display device.
@param sim_time Current simulation time in seconds.
@return None
"""


def updateTimeDisplay(display, sim_time: float) -> None:
    # Convert seconds -> h:m:s:ms
    total_ms = int(sim_time * 1000)
    hours = total_ms // (3600 * 1000)
    minutes = (total_ms // (60 * 1000)) % 60
    seconds = (total_ms // 1000) % 60
    millis = total_ms % 1000

    time_str = f"{hours:02d}:{minutes:02d}:{seconds:02d}.{millis:03d}"

    # Draw to display
    display.setColor(0xFFFFFF)  # white background
    display.fillRectangle(0, 0, display.getWidth(), display.getHeight())

    display.setColor(0x000000)  # black text
    display.drawText(time_str, 10, 10)


"""
Entry point for building the maze and signalling readiness.

@return None
"""


def main():
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep()) or 32

    timeDisplay = supervisor.getDevice("TIME_DISPLAY")
    timeDisplay.setFont("Arial", 20, True)

    # Initialise world_ready to 0 for this run
    robotNode = supervisor.getFromDef("MAZE_ROBOT")
    customField = None
    if robotNode is not None:
        customField = robotNode.getField("customData")
        if customField is not None:
            customField.setSFString("world_ready=0")
            print("[maze_builder] world_ready initialised to 0")

    rng = random.Random(SEED)

    print(f"[maze_builder] started with seed={SEED}")

    # NEW: choose start & goal on opposite borders
    startCell, goalCell = chooseStartAndGoal(rng)

    updateRectangleArena(supervisor)
    moveRobotToCell(supervisor, startCell)

    openEdges = generatePerfectMaze(ROWS, COLS, rng)
    print("[maze_builder] open passages (edges between cells):")
    for edge in sorted(openEdges):
        print("  ", edge)

    buildBorderWalls(supervisor)
    buildInternalWalls(supervisor, openEdges)

    # NEW: place a visual marker at the goal cell
    placeGoalMarker(supervisor, goalCell)

    # Mark world as ready *after* maze + robot placement + marker
    if customField is not None:
        payload = buildCustomDataPayload(startCell, goalCell, SEED)
        customField.setSFString(payload)
        print(f"[maze_builder] customData set: {payload}")

    while supervisor.step(timestep) != -1:
        updateTimeDisplay(timeDisplay, supervisor.getTime())
        pass


if __name__ == "__main__":
    main()
