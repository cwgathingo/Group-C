# Group-C

Phase 1 maze-solving robot for the Micromouse-style assignment. The current solution:
- Builds and updates a belief map from on-robot sensing while navigating.
- Uses wavefront planning (dynamic start/goal selected by the supervisor) instead of a hard-coded path.
- Runs encoder-based forward/turn primitives with IR/front sensing for walls.
- Prints the evolving ASCII map and wavefront matrix to the console during the run.

## Running the simulation
1. Install Webots with Python controller support (use the course-standard version).
2. Open the world: `worlds/CSCK505_robot_solve_maze.wbt`.
3. Controllers are preassigned: supervisor `maze_builder`, robot `maze_solver`.
4. Press Play. The supervisor picks a start on the bottom row and a goal on the top row, writes them to `customData`, and signals readiness. The solver waits for that handshake, then plans via wavefront and drives the maze while updating the map. Check the Webots console for the ASCII map and wavefront prints.

## Notes
- Coding standards: see `docs/Coding_Standards.md`.
- References/citations: see `docs/REFERENCES.md`.
- No automated test suite is bundled right now; validation is via the Webots run and console output.

## Maze configuration and tweaking
- Config: `controllers/maze_shared/maze_config.py` (`ROWS`, `COLS`, `CELL_SIZE`, `SEED`; origin is derived automatically). Start/goal normally come from the supervisor; defaults exist only as a fallback.
- To change the maze, edit those values and rerun; the supervisor rebuilds walls from the config and seed.
- If you change `CELL_SIZE`, keep corridors wide enough for the robot to traverse but narrow enough for the sensors to see the walls; shrinking may require retuning IR distance thresholds.
