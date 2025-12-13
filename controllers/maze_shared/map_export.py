# controllers/maze_shared/map_export.py
# Provenance: The rendering logic in this file (grid/walls/path/legend/labels via Pillow)
# was developed iteratively with AI assistance to create a convenient, non-core
# visualization of the maze belief. Prompts covered exporting the belief map to PNG,
# adding visited vs unvisited shading, drawing walls and a traversed path, adding
# start/goal markers, row/column labels, a legend, and tuning layout/margins.
# Tool: OpenAI ChatGPT (GPT-5.1), sessions on 07 Dec 2025, accessed via
# https://chat.openai.com/chat. This is a convenience utility; no external code
# snippets or textbooks/online posts were copied.

from typing import List, Tuple
from typing import TYPE_CHECKING

from maze_shared.logger import logInfo, logWarn

if TYPE_CHECKING:
    from maze.maze import Maze, Cell, Direction, PassageState


def export_belief_map_png(
    maze: "Maze",
    path: List["Cell"],
    filename: str,
    status: str = "",
    cell_px: int = 70,
    margin: int = 35,
) -> None:
    """
    Export the belief map (and optional path) to a PNG file.

    - Visited cells are shaded.
    - BLOCKED passages are drawn as walls.
    - The traversed path is overlaid in red when provided.
    - Start/goal cells are highlighted.

    Requires Pillow; otherwise a warning is logged and export is skipped.
    """
    try:
        from PIL import Image, ImageDraw, ImageFont
    except ImportError:
        logWarn("[map_export] Pillow not available; skipping PNG export.")
        return

    rows, cols = maze.getShape()
    legend_height = 40
    left_margin = margin
    right_margin = margin
    top_margin = margin + 10  # small extra padding on top
    bottom_margin = margin

    width = cols * cell_px + left_margin + right_margin
    height = rows * cell_px + top_margin + bottom_margin + legend_height

    img = Image.new("RGB", (width, height), (255, 255, 255))
    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.load_default()
    except Exception:  # noqa: BLE001
        font = None

    def cell_rect(cell: "Cell") -> Tuple[float, float, float, float]:
        r, c = cell
        x0 = left_margin + c * cell_px
        y0 = top_margin + r * cell_px
        return (x0, y0, x0 + cell_px, y0 + cell_px)

    def cell_center(cell: "Cell") -> Tuple[float, float]:
        r, c = cell
        x = left_margin + c * cell_px + 0.5 * cell_px
        y = top_margin + r * cell_px + 0.5 * cell_px
        return (x, y)

    # Fill cells (visited vs unknown)
    for r in range(rows):
        for c in range(cols):
            rect = cell_rect((r, c))
            fill = (200, 200, 200) if maze.isVisited((r, c)) else (255, 255, 255)
            draw.rectangle(rect, fill=fill, outline=(180, 180, 180))

    # Grid lines for clarity
    grid_color = (150, 150, 150)
    for c in range(cols + 1):
        x = left_margin + c * cell_px
        draw.line(
            [(x, top_margin), (x, top_margin + rows * cell_px)],
            fill=grid_color,
            width=1,
        )
    for r in range(rows + 1):
        y = top_margin + r * cell_px
        draw.line(
            [(left_margin, y), (left_margin + cols * cell_px, y)],
            fill=grid_color,
            width=1,
        )

    # Draw walls for BLOCKED passages
    from maze.maze import Direction, PassageState  # local import to avoid cycles

    wall_color = (0, 0, 0)
    wall_w = 3
    for r in range(rows):
        for c in range(cols):
            cell = (r, c)
            x0, y0, x1, y1 = cell_rect(cell)
            passages = maze.getAllPassages(cell)
            if passages[Direction.NORTH] == PassageState.BLOCKED:
                draw.line([(x0, y0), (x1, y0)], fill=wall_color, width=wall_w)
            if passages[Direction.SOUTH] == PassageState.BLOCKED:
                draw.line([(x0, y1), (x1, y1)], fill=wall_color, width=wall_w)
            if passages[Direction.WEST] == PassageState.BLOCKED:
                draw.line([(x0, y0), (x0, y1)], fill=wall_color, width=wall_w)
            if passages[Direction.EAST] == PassageState.BLOCKED:
                draw.line([(x1, y0), (x1, y1)], fill=wall_color, width=wall_w)

    # Draw path (if any)
    if len(path) >= 2:
        path_pts = [cell_center(cell) for cell in path]
        draw.line(path_pts, fill=(255, 0, 0), width=4)

    # Mark start and goal
    start = maze.getStart()
    goal = maze.getGoal()
    radius = cell_px * 0.25
    sx, sy = cell_center(start)
    gx, gy = cell_center(goal)
    draw.ellipse(
        [(sx - radius, sy - radius), (sx + radius, sy + radius)],
        fill=(0, 200, 0),
        outline=(0, 120, 0),
        width=2,
    )
    draw.ellipse(
        [(gx - radius, gy - radius), (gx + radius, gy + radius)],
        fill=(0, 120, 255),
        outline=(0, 80, 180),
        width=2,
    )

    try:
        # Row/col labels (outside cells)
        if font is not None:
            label_color = (120, 120, 120)
            # Column labels along top
            for c in range(cols):
                x = left_margin + c * cell_px + 0.4 * cell_px
                y = max(0, top_margin * 0.2)
                draw.text((x, y), str(c), fill=label_color, font=font)
            # Row labels along left
            for r in range(rows):
                x = max(0, left_margin * 0.35)
                y = top_margin + r * cell_px + 0.25 * cell_px
                draw.text((x, y), str(r), fill=label_color, font=font)

            # Legend at bottom
            legend_y = top_margin + rows * cell_px + (legend_height * 0.15)
            legend_items = [
                ("Visited", (200, 200, 200)),
                ("Unvisited", (255, 255, 255)),
                ("Wall", (0, 0, 0)),
                ("Start", (0, 200, 0)),
                ("Goal", (0, 120, 255)),
                ("Path", (255, 0, 0)),
            ]
            lx = margin
            box_size = 16
            spacing = 10
            for label, color in legend_items:
                draw.rectangle(
                    [(lx, legend_y), (lx + box_size, legend_y + box_size)],
                    fill=color,
                    outline=(0, 0, 0),
                )
                draw.text(
                    (lx + box_size + 6, legend_y - 2),
                    label,
                    fill=(60, 60, 60),
                    font=font,
                )
                lx += box_size + 6 + draw.textlength(label, font=font) + spacing

        img.save(filename, format="PNG")
        logInfo(
            f"[map_export] Exported final belief map to {filename} (status={status})."
        )
    except Exception as exc:  # noqa: BLE001
        logWarn(f"[map_export] Failed to export final map image: {exc}")
