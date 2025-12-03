import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))

from controller import Supervisor
import random
from typing import Tuple, List, Set
from maze_shared.maze_config import ROWS, COLS, CELL_SIZE, MAZE_ORIGIN, SEED

Cell = Tuple[int, int]

# Maze dimensions – must match MazeController
# ROWS = 4
# COLS = 4

# Geometry – must match your controller setup:
#   cellSizeMeters = 0.15
#   mazeOriginWorld = (-0.225, 0.225)
# CELL_SIZE = 0.15
# ORIGIN_X = -0.225
# ORIGIN_Y = 0.225
ORIGIN_X, ORIGIN_Y = MAZE_ORIGIN


def generate_perfect_maze(
    rows: int, cols: int, rng: random.Random
) -> Set[tuple[Cell, Cell]]:
    """
    Depth-first search (recursive backtracker style, but iterative)
    that returns a set of OPEN edges.
    Each edge is stored as ((r1, c1), (r2, c2)) with endpoints sorted,
    so order is deterministic.
    """
    stack: List[Cell] = []
    visited = [[False] * cols for _ in range(rows)]
    open_edges: Set[tuple[Cell, Cell]] = set()

    def neighbours(cell: Cell):
        r, c = cell
        # Fixed neighbour order → determinism given RNG
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
        open_edges.add(edge)

        stack.append(nxt)

    return open_edges


# ---------- Geometry helpers ----------


def vertical_wall_pos(r: int, c: int) -> Tuple[float, float]:
    """
    Position of a vertical wall segment between (r, c) and (r, c+1).
    For borders we allow c = -1 (left of col 0) and c = COLS-1 (right of last).
    """
    x = ORIGIN_X + (c + 0.5) * CELL_SIZE
    y = ORIGIN_Y - r * CELL_SIZE
    return x, y


def horizontal_wall_pos(r: int, c: int) -> Tuple[float, float]:
    """
    Position of a horizontal wall segment between (r, c) and (r+1, c).
    For borders we allow r = -1 (above row 0) and r = ROWS-1 (below last).
    """
    x = ORIGIN_X + c * CELL_SIZE
    y = ORIGIN_Y - (r + 0.5) * CELL_SIZE
    return x, y


def build_border_walls(supervisor: Supervisor) -> None:
    """
    Remove all existing Wall nodes and create a rectangular frame of
    Wall segments around the 4x4 cell grid.

    This step ignores the maze connectivity; it's just to verify that
    our coordinates and sizes are correct.
    """
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
        x, y = horizontal_wall_pos(-1, c)
        children.importMFNodeFromString(
            -1,
            f"Wall {{ translation {x} {y} 0 size {CELL_SIZE} 0.01 0.05 }}",
        )

        # Bottom border: between row ROWS-1 and "virtual row ROWS"
        x, y = horizontal_wall_pos(ROWS - 1, c)
        children.importMFNodeFromString(
            -1,
            f"Wall {{ translation {x} {y} 0 size {CELL_SIZE} 0.01 0.05 }}",
        )

    # 3) Add left and right border (vertical walls)
    for r in range(ROWS):
        # Left border: between "virtual col -1" and col 0
        x, y = vertical_wall_pos(r, -1)
        children.importMFNodeFromString(
            -1,
            f"Wall {{ translation {x} {y} 0 size 0.01 {CELL_SIZE} 0.05 }}",
        )

        # Right border: between col COLS-1 and "virtual col COLS"
        x, y = vertical_wall_pos(r, COLS - 1)
        children.importMFNodeFromString(
            -1,
            f"Wall {{ translation {x} {y} 0 size 0.01 {CELL_SIZE} 0.05 }}",
        )

    print("[maze_builder] border walls created")


def build_internal_walls(
    supervisor: Supervisor, open_edges: Set[tuple[Cell, Cell]]
) -> None:
    """
    Create internal walls based on the maze connectivity in open_edges.

    For every pair of adjacent cells:
      - If the edge is in open_edges → NO wall (passage).
      - Otherwise → add a wall segment at the boundary between them.
    """
    root = supervisor.getRoot()
    children = root.getField("children")

    def is_open(a: Cell, b: Cell) -> bool:
        return tuple(sorted([a, b])) in open_edges

    # --- Vertical internal walls (between (r,c) and (r,c+1)) ---
    for r in range(ROWS):
        for c in range(COLS - 1):
            a = (r, c)
            b = (r, c + 1)
            if not is_open(a, b):
                x, y = vertical_wall_pos(r, c)
                children.importMFNodeFromString(
                    -1,
                    f"Wall {{ translation {x} {y} 0 size 0.01 {CELL_SIZE} 0.05 }}",
                )

    # --- Horizontal internal walls (between (r,c) and (r+1,c)) ---
    for r in range(ROWS - 1):
        for c in range(COLS):
            a = (r, c)
            b = (r + 1, c)
            if not is_open(a, b):
                x, y = horizontal_wall_pos(r, c)
                children.importMFNodeFromString(
                    -1,
                    f"Wall {{ translation {x} {y} 0 size {CELL_SIZE} 0.01 0.05 }}",
                )

    print("[maze_builder] internal walls created")


def update_rectangle_arena(supervisor: Supervisor) -> None:
    """
    Find the RectangleArena node and update its size / tile size / wall height
    based on ROWS, COLS, CELL_SIZE from the shared config.
    """
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
    floor_width = COLS * CELL_SIZE
    floor_height = ROWS * CELL_SIZE

    tile_width = 2.0 * CELL_SIZE  # so each checker ≈ one maze cell visually
    tile_height = 2.0 * CELL_SIZE

    # Update fields
    floor_size_field = arena_node.getField("floorSize")
    tile_size_field = arena_node.getField("floorTileSize")
    wall_height_field = arena_node.getField("wallHeight")

    if floor_size_field is not None:
        floor_size_field.setSFVec2f([floor_width, floor_height])

    if tile_size_field is not None:
        tile_size_field.setSFVec2f([tile_width, tile_height])

    if wall_height_field is not None:
        wall_height_field.setSFFloat(0.001)  # almost flat

    print(
        "[maze_builder] RectangleArena updated: "
        f"floorSize=({floor_width:.3f}, {floor_height:.3f}), "
        f"floorTileSize=({tile_width:.3f}, {tile_height:.3f}), "
        "wallHeight=0.001"
    )


def cell_to_world_center(cell: Cell) -> tuple[float, float]:
    r, c = cell
    x = ORIGIN_X + c * CELL_SIZE
    y = ORIGIN_Y - r * CELL_SIZE
    return x, y


def move_robot_to_cell(supervisor: Supervisor, cell: Cell) -> None:
    """
    Move MAZE_ROBOT to the centre of the given maze cell.
    """
    robot_node = supervisor.getFromDef("MAZE_ROBOT")
    if robot_node is None:
        print("[maze_builder] WARNING: MAZE_ROBOT not found; cannot move robot")
        return

    tx_field = robot_node.getField("translation")
    if tx_field is None:
        print("[maze_builder] WARNING: robot has no translation field")
        return

    x, y = cell_to_world_center(cell)
    current = tx_field.getSFVec3f()
    z = current[2] if len(current) >= 3 else 0.0

    tx_field.setSFVec3f([x, y, z])
    print(f"[maze_builder] moved robot to cell {cell} at ({x:.3f}, {y:.3f})")


def choose_start_and_goal(rng: random.Random) -> tuple[Cell, Cell]:
    """
    Choose a random start cell on the bottom row and a random goal cell on the top row.
    Both are guaranteed to be adjacent to opposite outer walls.
    """
    start_row = ROWS - 1  # bottom
    goal_row = 0  # top

    start_col = rng.randrange(COLS)
    goal_col = rng.randrange(COLS)

    start = (start_row, start_col)
    goal = (goal_row, goal_col)

    print(f"[maze_builder] chosen start={start}, goal={goal}")
    return start, goal


def place_goal_marker(supervisor: Supervisor, goal_cell: Cell) -> None:
    """
    Place a small visual marker (green cylinder) at the centre of the goal cell.
    """
    root = supervisor.getRoot()
    children = root.getField("children")

    x, y = cell_to_world_center(goal_cell)

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
    print(
        f"[maze_builder] placed goal marker at cell {goal_cell} at ({x:.3f}, {y:.3f})"
    )


DYNAMIC_CONFIG_PATH = os.path.join(
    os.path.dirname(__file__),
    "..",
    "maze_shared",
    "dynamic_config.py",
)


def write_dynamic_config(start_cell: Cell, goal_cell: Cell, seed: int) -> None:
    content = f"""# AUTO-GENERATED AT RUNTIME — DO NOT EDIT
START = ({start_cell[0]}, {start_cell[1]})
GOAL = ({goal_cell[0]}, {goal_cell[1]})
SEED = {seed}
ROWS = {ROWS}
COLS = {COLS}
CELL_SIZE = {CELL_SIZE}
"""
    try:
        with open(DYNAMIC_CONFIG_PATH, "w", encoding="utf-8") as f:
            f.write(content)
        print(f"[maze_builder] wrote config to {DYNAMIC_CONFIG_PATH}")
    except OSError as e:
        print(f"[maze_builder] ERROR writing dynamic_config.py: {e}")


def main():
    supervisor = Supervisor()
    timestep = int(supervisor.getBasicTimeStep()) or 32

    # Initialise world_ready to 0 for this run
    robot_node = supervisor.getFromDef("MAZE_ROBOT")
    custom_field = None
    if robot_node is not None:
        custom_field = robot_node.getField("customData")
        if custom_field is not None:
            custom_field.setSFString("world_ready=0")
            print("[maze_builder] world_ready initialised to 0")

    rng = random.Random(SEED)

    print(f"[maze_builder] started with seed={SEED}")

    # NEW: choose start & goal on opposite borders
    start_cell, goal_cell = choose_start_and_goal(rng)
    write_dynamic_config(start_cell, goal_cell, SEED)

    update_rectangle_arena(supervisor)
    move_robot_to_cell(supervisor, start_cell)

    open_edges = generate_perfect_maze(ROWS, COLS, rng)
    print("[maze_builder] open passages (edges between cells):")
    for edge in sorted(open_edges):
        print("  ", edge)

    build_border_walls(supervisor)
    build_internal_walls(supervisor, open_edges)

    # NEW: place a visual marker at the goal cell
    place_goal_marker(supervisor, goal_cell)

    # Mark world as ready *after* maze + robot placement + marker
    if custom_field is not None:
        custom_field.setSFString("world_ready=1")
        print("[maze_builder] world_ready set to 1")

    while supervisor.step(timestep) != -1:
        pass


if __name__ == "__main__":
    main()
