# Robot Interface Method Usage

Current state of `controllers/maze_solver/robots/robot_interface.py` implementations and where they are used.

| Method | Used? | Where | Notes / Suggested use |
| --- | --- | --- | --- |
| `getHeadingVector()` | No | — | Optional: expose continuous heading for debugging, compass-based alignment, or snapping heading in the controller. |
| `getState()` | No | — | Could be used by the controller/supervisor UI to report idle/executing/error state. |
| `getCurrentAction()` | No | — | Useful for status reporting/telemetry; could aid debugging or recovery logic. |
| `getLastActionResult()` | Yes | `maze_solver.MazeController._handleCompletedAction()` | Already used to gate post-action updates and stop on failure. |
| `cancelAction()` | No | — | Hook for emergency stop or supervisor interruption if an action overruns or environment changes. |
| `resetPose(cell, direction)` | No | — | For re-localisation or test resets; could be invoked by a supervisor script or debug command. |
| `getWorldPose()` | No | — | Optional for logging/visualisation or to cross-check odometry against ground truth. |
| `getWorldPosition()` | No | — | Convenience for 2D-only position logging/visualisation. |

Summary: only `getLastActionResult` is exercised in `maze_solver.py`. Other methods are currently unused but remain valuable extension points for UI/telemetry, debugging, safety (cancel), or reinitialisation (resetPose). No functional changes needed unless you want to start wiring these into the controller/supervisor.
