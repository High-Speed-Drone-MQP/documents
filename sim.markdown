# Simulation Setup

This document describes how to set up the repository for simulations and how to run the Fast-Planner + NMPC simulation.

**Workspace vs external apps:** The **ROS 2 workspace** you set up below is the only workspace used for the sim (NMPC, Fast-Planner, bridge, acp_autonomy, etc.). Everything else—PX4, Gazebo, MicroXrceDDS, QGroundControl—lives **outside** the workspace under **`ACP_DIR`** (see §2).

---

## 1. Initial workspace setup

### 1.1 Clone platform-setup in your ROS 2 workspace

Use a ROS 2 workspace root that will contain both `platform-setup` and the `src` directory. This is the **normal** ROS 2 workspace you will source everywhere. From that workspace root:

```bash
cd /path/to/your/ros2_workspace
git clone <platform-setup-repo-url> platform-setup
```

Your layout should be:

```
<ros2_workspace>/
├── platform-setup/
│   └── sim-setup/
│       └── setup_mqp_stack_sim.sh
├── src/
└── ...
```

### 1.2 Run the sim stack setup script

From the workspace root (or from `platform-setup/sim-setup`), run:

```bash
cd /path/to/your/ros2_workspace/platform-setup/sim-setup
./setup_mqp_stack_sim.sh
```

The script will:

- Clone **test_scripts** into the workspace
- Clone and build ROS 2 packages in `src/`, including:
  - **acp-quadrotor-control**, **waypoint_navigation_plugin**, **acp-autonomy-stack**
  - **px4_msgs**, **px4_ros_com**, **acp-px4-interface**, **odom_transform**
  - **dq_nmpc**, **acados** (with tera_renderer), **dq_cpp**
  - **Fast-Planner-ROS2-Humble**, **fastplanner_to_nmpc_bridge**, **pc2_tools_cpp**

**After the script finishes**, add the printed environment variables to your `~/.bashrc` (the script will remind you), for example:

```bash
export ACADOS_INSTALL_DIR=/path/to/your/ros2_workspace/src/acados_install
export ACADOS_SOURCE_DIR=/path/to/your/ros2_workspace/src/acados
export LD_LIBRARY_PATH=${ACADOS_INSTALL_DIR}/lib:${LD_LIBRARY_PATH}
export WS=/path/to/your/ros2_workspace
```

**Prerequisites for the script:**

- ROS 2 Humble
- SSH key added (e.g. `ssh-add ~/.ssh/id_ed25519`) for cloning private repos
- Python packages: `pip install "scipy<1.13" pyyaml pynput casadi osqp`

---

## 2. External dependencies for Fast-Planner test

The **ROS 2 workspace** (the one you set up in §1) is the only workspace used for the control stack (NMPC, Fast-Planner, bridge, etc.).

All other code and applications live **outside** the workspace under **`ACP_DIR`**. Set `ACP_DIR` to the root directory where you keep these external pieces (e.g. `~/Documents/ACP`). Typical contents:

| Dependency | Purpose | Location |
|------------|--------|----------|
| **PX4** | PX4 SITL with Gazebo. The test script runs `make px4_sitl gz_x500_depth`. | `$ACP_DIR/PX4-Autopilot-INDI` |
| **Gazebo** | Used by PX4 SITL (`gz_x500_depth`). Use the **Ubuntu 22.04** Gazebo version required by your PX4 fork (e.g. Gazebo Fortress). | System / ACP_DIR as needed |
| **MicroXrceDDS** | PX4–ROS 2 DDS bridge. The test script runs `MicroXRCEAgent udp4 -p 8888`. | Install so `MicroXRCEAgent` is on PATH |
| **QGroundControl** (optional) | Launched in a tmux pane for monitoring. | Install so `qgc` is on PATH |

**Summary:** `ACP_DIR` = source of all code/applications that are **not** part of the ROS 2 workspace (PX4, MicroXrceDDS, Gazebo, QGroundControl, etc.). The ROS 2 workspace can live anywhere; it is detected by the test script as the parent of `test_scripts/`.

Ensure PX4, Gazebo (Ubuntu 22.04), and MicroXrceDDS are installed and that `PX4-Autopilot-INDI` builds successfully with `make px4_sitl gz_x500_depth`.

---

## 3. Running the simulation (three terminals)

Run the following in **three separate terminals**. Source the **same ROS 2 workspace** (the one set up in §1) in each terminal; no separate NMPC workspace is used.

### Terminal 1: PX4 SITL, MicroXRCEAgent, QGC, RViz

From your ROS 2 workspace (optional: set `ACP_DIR` if PX4 is not under `~/Documents/ACP`):

```bash
cd /path/to/your/ros2_workspace
source install/setup.bash
# Optional: export ACP_DIR=/path/to/external/apps   # default: ~/Documents/ACP

./test_scripts/fastplanner_test.sh
```

The script detects the workspace as the parent of `test_scripts/` and sources `install/setup.bash` in the tmux panes.

This starts a **tmux** session that runs:

- PX4 SITL with Gazebo (`PX4_GZ_WORLD=pillars` or `walls`, e.g. `./fastplanner_test.sh walls`)
- **MicroXRCEAgent** on UDP port 8888
- QGroundControl (`qgc`)
- RViz2 with a SITL config

Optional world: `WORLD=walls ./test_scripts/fastplanner_test.sh` (default is `pillars`).

### Terminal 2: Gazebo FP autonomy launch (Fast-Planner + Bridge + NMPC)

Source the workspace, then:

```bash
source /path/to/your/ros2_workspace/install/setup.bash

ros2 launch acp_autonomy gazebo_fp.launch.py
```

This launches the Fast-Planner + bridge + DQ-NMPC stack, GZ–ROS 2 bridges, PX4 interface nodes, and related nodes. PX4 and MicroXRCEAgent are assumed to already be running (e.g. from Terminal 1).

### Terminal 3: Fast-Planner test node (mission script)

Source the same workspace, then:

```bash
source /path/to/your/ros2_workspace/install/setup.bash

ros2 run mav_manager_test fastplanner_test
```

Optional namespace (if your launch uses a quad name):

```bash
ros2 run mav_manager_test fastplanner_test quadrotor
```

This runs the state machine that arms, takes off, sends waypoints to Fast-Planner, waits for the trajectory and bridge, switches to NullTracker, and then runs the mission and lands.

---

## 4. Summary

| Step | Action |
|------|--------|
| Setup | In your ROS 2 workspace: clone **platform-setup**, then run `platform-setup/sim-setup/setup_mqp_stack_sim.sh`. Add the printed env vars to `~/.bashrc`. |
| External | Set **ACP_DIR** to the root for external apps. Install **PX4** (at `$ACP_DIR/PX4-Autopilot-INDI`), **Gazebo** (Ubuntu 22.04), and **MicroXrceDDS** (MicroXRCEAgent). Optionally install QGC. |
| Run | **Terminal 1:** `./test_scripts/fastplanner_test.sh` (uses this workspace) — **Terminal 2:** `ros2 launch acp_autonomy gazebo_fp.launch.py` — **Terminal 3:** `ros2 run mav_manager_test fastplanner_test` |

The Fast-Planner test node lives in:

`src/acp-quadrotor-control/manager/mav_manager_test/mav_manager_test/fastplanner_test.py`

and is invoked via the `mav_manager_test` package entry point `fastplanner_test`.

---

## 5. VizFlyt simulation variant

For the **VizFlyt** sim (Gaussian splat depth rendering with the same stack), use the same three-terminal flow but launch `gazebo_fp_vizflyt.launch.py` in Terminal 2 instead of `gazebo_fp.launch.py`. See **[sim_vizflyt.markdown](sim_vizflyt.markdown)** for full steps.
