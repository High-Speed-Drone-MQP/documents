# VizFlyt Simulation Setup

This document describes how to run the **VizFlyt** variant of the Fast-Planner + NMPC simulation. VizFlyt adds Gaussian splat depth rendering via the `vizflyt_node` to the same stack as the standard Gazebo FP sim.

**Prerequisites:** Complete the [main simulation setup](sim.markdown) first (§1 Initial workspace setup and §2 External dependencies). The same ROS 2 workspace and external apps (PX4, Gazebo, MicroXrceDDS, etc.) are used.

---

## Difference from standard Gazebo FP sim

| Item | Standard (`gazebo_fp`) | VizFlyt (`gazebo_fp_vizflyt`) |
|------|------------------------|-------------------------------|
| Launch file | `gazebo_fp.launch.py` | `gazebo_fp_vizflyt.launch.py` |
| Extra components | GZ–ROS 2 depth/pointcloud bridge | **VizFlyt splat renderer** (`vizflyt_node`) for depth rendering |

The run flow is the same: three terminals (PX4 + MicroXRCEAgent + QGC + RViz → autonomy launch → Fast-Planner test node). Only the launch file in Terminal 2 changes.

---

## Running the VizFlyt simulation (three terminals)

Use the **same ROS 2 workspace** and **ACP_DIR** as in the main sim doc. Source the workspace in each terminal.

### Terminal 1: PX4 SITL, MicroXRCEAgent, QGC, RViz

Same as the standard sim:

```bash
cd /path/to/your/ros2_workspace
source install/setup.bash
# Optional: export ACP_DIR=/path/to/external/apps

./test_scripts/fastplanner_test.sh
```

Optional world: `WORLD=walls ./test_scripts/fastplanner_test.sh` (default is `pillars`).

### Terminal 2: VizFlyt Gazebo FP autonomy launch

Source the workspace, then launch the **VizFlyt** autonomy stack:

```bash
source /path/to/your/ros2_workspace/install/setup.bash

ros2 launch acp_autonomy gazebo_fp_vizflyt.launch.py
```

This starts the same Fast-Planner + bridge + DQ-NMPC stack as `gazebo_fp.launch.py`, plus:

- **vizflyt_node** (Gaussian splat depth rendering)
- Point cloud processing (`pc2_reheader_transform`) for depth → world frame

PX4 and MicroXRCEAgent must already be running (e.g. from Terminal 1).

### Terminal 3: Fast-Planner test node (mission script)

Same as the standard sim:

```bash
source /path/to/your/ros2_workspace/install/setup.bash

ros2 run mav_manager_test fastplanner_test
```

Optional namespace: `ros2 run mav_manager_test fastplanner_test quadrotor`

---

## Summary

| Step | Action |
|------|--------|
| Setup | Follow [sim.markdown](sim.markdown) (§1 and §2). Ensure the workspace builds (including the `vizflyt_node` package if present in `src/`). |
| Run | **Terminal 1:** `./test_scripts/fastplanner_test.sh` — **Terminal 2:** `ros2 launch acp_autonomy gazebo_fp_vizflyt.launch.py` — **Terminal 3:** `ros2 run mav_manager_test fastplanner_test` |

Launch file: `src/acp-autonomy-stack/launch/simulator/gazebo_fp_vizflyt.launch.py`
