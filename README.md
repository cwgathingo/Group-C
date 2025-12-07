# Group-C

Micromouse-style solver built in Webots. A supervisor builds a perfect maze, chooses start/goal on opposite borders, and hands the config to the robot via `customData`. The robot keeps a belief map while it navigates with either a wavefront (NF1) or A* planner, printing ASCII maps and optional planner traces as it goes.

## Quick start
1. Install Webots with Python controller support (course-standard R2025a is expected).
2. Open `worlds/CSCK505_robot_solve_maze.wbt` (controllers are already set: supervisor=`maze_builder`, robot=`maze_solver`).
3. Press Play. The supervisor rebuilds the maze from the shared config/seed, picks start (bottom row) and goal (top row), writes them to `customData`, then sets `world_ready=1`. The robot waits for that handshake, applies runtime overrides, and begins solving.
4. Watch the Webots console for the evolving ASCII map, wavefront/A* prints (when enabled), and status messages. The timer display in the world UI shows simulation time.

## Configuration knobs (`controllers/maze_shared/maze_config.py`)
- Maze: `ROWS`, `COLS`, `CELL_SIZE`, `SEED` (maze shape/seed); `MAZE_ORIGIN` is derived automatically.
- Planning: `DEFAULT_PLANNER` (`wavefront` or `a_star`); `A_STAR_UNKNOWN_COST` (penalty for unknown passages), `A_STAR_TRACE` (extra debug when `LOG_LEVEL` is `DEBUG`).
- Perception: `DEFAULT_PERCEPTION_MODE` (`lidar` or `ir`) sets wall sensing for the robot facade. Can be overridden at runtime via `customData` (`perception=...`).
- Logging/exports: `LOG_LEVEL` controls verbosity; `EXPORT_FINAL_MAP_TO_PNG` saves the final belief map and path to `docs/maps/` (install Pillow: `pip install pillow`).

## Runtime overrides from the supervisor
The supervisor writes a `customData` payload with `world_ready`, `start`, `goal`, `startDir`, `rows`, `cols`, `cell_size`, `seed`, plus optional `planner` and `perception`. The solver blocks until `world_ready==1`, merges the payload with the defaults above, and uses that merged config for the run. Change `SEED` (or the chooser in `maze_builder.py`) for different deterministic mazes and start/goal pairs.

## Outputs
- Console: evolving ASCII belief map, planner traces (wavefront always; A* trace when `A_STAR_TRACE` and debug logging are on), and high-level status.
- Files: when PNG export is enabled, a `map_seed{seed}_{rows}x{cols}_{planner}_[...].png` is written to `docs/maps/` summarising visited cells, walls, start/goal, and the traversed path.

## Notes
- Coding standards: `docs/Coding_Standards.md`.
- References/citations: `docs/REFERENCES.md`.
- No automated test suite is bundled; validate via Webots runs and the generated console/PNG outputs.
