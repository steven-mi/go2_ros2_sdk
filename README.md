# Unitree Go2 ROS2 SDK

ROS2 integration for the Unitree Go2 (AIR / PRO / EDU) over **WebRTC (Wi‑Fi)** or **CycloneDDS (Ethernet)**. Close the mobile app before connecting when using WebRTC.

[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/22.04/)
![ROS2 Build](https://github.com/abizovnuralem/go2_ros2_sdk/actions/workflows/ros_build.yaml/badge.svg)
[![License](https://img.shields.io/badge/license-BSD--2-yellow.svg)](https://opensource.org/licenses/BSD-2-Clause)

## Features

| Area | Capabilities |
|------|--------------|
| **Robot I/O** | URDF, joint states, IMU, foot force sensors, joystick teleop |
| **Sensors** | LiDAR point cloud (~7 Hz), laser scan, front camera stream |
| **Mapping & nav** | SLAM (`slam_toolbox`), Nav2 autonomous navigation, map save/load |
| **Web dashboard** | React UI — 2D occupancy map, 3D LiDAR view, camera, click-to-navigate |
| **Visualization** | RViz2, Foxglove bridge (`ws://localhost:8765`) |
| **Perception** | COCO object detection (`coco_detector`) |
| **Connectivity** | WebRTC (Wi‑Fi), CycloneDDS (Ethernet), multi-robot support |
| **Data export** | Raw `.ply` LiDAR dumps; Nav2 maps (`.yaml` + `.pgm`) via web UI or slam_toolbox |

---

## Getting started (Docker)

Two-terminal workflow: keep one long-lived container, edit code on the host, rebuild inside the container, and run different ROS commands without rebuilding the image each time.

> **Note:** Full launch files (`mapping.launch.py`, `navigation.launch.py`, `robot.launch.py`) each start their own `go2_driver_node`. Stopping one launch and starting another disconnects WebRTC and reconnects to the dog. The workflow below uses a persistent container shell so you control what runs and when; see [Switching launch files](#switching-launch-files) for the driver caveat.

---

## Prerequisites

- Docker and Docker Compose
- Unitree Go2 on the same network; mobile app **closed** before connecting
- Robot IP (from the Unitree app: Device → Data → STA Network `wlan0`)
- For RViz on the host: `xhost +local:docker` and `export DISPLAY=:0`

---

## One-time image build

From the `docker/` directory:

```bash
cd docker
export ROBOT_IP=192.168.x.x   # your dog's IP

docker compose build
```

The image installs dependencies and runs an initial `colcon build`. After that, the **source mount** overrides `/ros2_ws/src` with your host files at runtime.

---

## What is mounted

| Host path | Container path | Purpose |
|-----------|----------------|---------|
| `.` (this repo) | `/ros2_ws/src` | Live code — edit on host |
| `docker/data/` | `/ros2_ws/data` | Maps, `.ply` dumps (persists across restarts) |

X11 and joystick mounts are unchanged (RViz, gamepad).

---

## Start the container (Terminal 1 — session)

Keep the container running without tying it to one launch file:

```bash
cd docker
export ROBOT_IP=192.168.x.x

docker compose run -d --name go2_dev --service-ports unitree_ros sleep infinity
```

Open a shell in that container (`go2_dev` is the container name — use `docker exec`, not `docker compose exec`):

```bash
docker exec -it go2_dev bash
```

You should see a prompt inside the container. ROS and the workspace are already sourced via `/ros_entrypoint.sh`.

Verify the mount:

```bash
ls /ros2_ws/src/go2_robot_sdk
```

---

## After code changes — rebuild in the container

Still inside the container (or via `exec`):

```bash
cd /ros2_ws
colcon build --packages-select go2_dashboard go2_robot_sdk   # or omit --packages-select for all
source install/setup.bash
```

Repeat after each edit. You do **not** need `docker compose build` unless you change the Dockerfile or system dependencies.

For Python-only changes, sometimes restarting the affected node is enough; for C++ (`lidar_processor_cpp`) or launch files, always rebuild and restart the launch.

**Web dashboard only:** use `web_dev:=true` or `npm run dev` in `go2_dashboard/web/` — no `colcon build` needed. For production (`:8080`), rebuild `go2_dashboard` after UI changes.

---

## Run the robot stack (Terminal 1 — inside container)

Pick **one** launch and leave it running in this terminal.

The web dashboard starts automatically on **http://localhost:8080** (rosbridge on **9090**). For UI dev with hot reload use `web_dev:=true` → **http://localhost:5173** — see [Web dashboard](#web-dashboard-react).

```bash
cd /ros2_ws/data
ros2 launch go2_robot_sdk mapping.launch.py rviz:=false
```

Other modes:

```bash
# Full demo stack (SLAM + Nav2 + RViz + joystick)
ros2 launch go2_robot_sdk robot.launch.py

# Navigate on a saved map (web UI + Nav2)
ros2 launch go2_robot_sdk navigation.launch.py map:=/ros2_ws/data/my_map.yaml rviz:=false
```

Disable the web UI if needed: `web_ui:=false`

`Ctrl+C` stops that launch (and disconnects the dog if the launch included `go2_driver_node`).

---

## Web dashboard (React)

The dashboard lives in `go2_dashboard/web/` — a **React + Vite** app with a **2D occupancy map**, **3D LiDAR point cloud**, camera feed, and click-to-navigate. The Docker image includes **Node.js 20** so `colcon build` can compile the UI and you can run Vite dev mode inside the container.

| URL | Purpose |
|-----|---------|
| http://localhost:8080 | Production UI (built `web/dist`, served after `colcon build`) |
| http://localhost:5173 | Dev UI with **hot reload** when `web_dev:=true` |
| ws://localhost:9090 | rosbridge WebSocket (used by the browser) |
| ws://localhost:8765 | Foxglove (optional) |

Ros topics used by the UI: `/map`, `/map_updates`, `/pointcloud/filtered`, `/amcl_pose`, `/camera/compressed`.

### Production (default)

After frontend or launch changes:

```bash
cd /ros2_ws
colcon build --packages-select go2_dashboard
source install/setup.bash
```

`colcon build` runs `npm ci && npm run build` automatically. Then start mapping/navigation as usual and open **http://localhost:8080** on the host.

```bash
ros2 launch go2_robot_sdk mapping.launch.py rviz:=false
```

### Development (hot reload)

Edit React files on the host under `go2_dashboard/web/src/` — the container sees them via the source mount. One-time (or after `package.json` changes):

```bash
docker exec -it go2_dev bash
cd /ros2_ws/src/go2_dashboard/web
npm install
```

Start the stack with Vite instead of the static server:

```bash
ros2 launch go2_robot_sdk mapping.launch.py rviz:=false web_dev:=true
```

Open **http://localhost:5173** on the host. Saves reload instantly; no `colcon build` needed for UI-only edits.

Alternatively, run Vite in a second container shell while a normal launch keeps rosbridge up:

```bash
# Terminal 1 — robot stack (rosbridge on 9090, static UI on 8080 can be ignored)
ros2 launch go2_robot_sdk mapping.launch.py rviz:=false web_ui:=false

# Terminal 2 — Vite dev server only
cd /ros2_ws/src/go2_dashboard/web && npm run dev
```

### Mapping workflow

1. Open the dashboard (8080 prod or 5173 dev)
2. Drive the dog with the gamepad/joystick
3. Watch the **2D map** and **3D point cloud** update live
4. Enter a name (e.g. `apartment`) and click **Save Map** + **Serialize Map**
5. Files appear in `docker/data/` on the host

### Navigation workflow

1. Place the dog at a known start position
2. Open the dashboard
3. Click **Set Pose** and click where the dog is on the 2D map (AMCL)
4. Click **Go To** and click the destination
5. Use **Cancel** to stop an active goal

### Troubleshooting

```bash
# rosbridge / topics
ros2 topic list | head
ros2 topic hz /map
ros2 topic hz /pointcloud/filtered

# HTTP servers (host network — same ports inside container)
curl -I http://localhost:8080
curl -I http://localhost:5173

# Rebuild UI only
cd /ros2_ws/src/go2_dashboard/web && npm run build
cd /ros2_ws && colcon build --packages-select go2_dashboard && source install/setup.bash
```

If the dashboard shows **Disconnected**, confirm the launch is running and port **9090** is reachable. The UI connects to `ws://<browser-hostname>:9090` (works with Docker `network_mode: host`).

The 3D LiDAR view uses a **CPU Canvas 2D** renderer (no WebGL/GPU required). Rebuild `go2_dashboard` after web UI changes.

More detail: [go2_dashboard/web/README.md](go2_dashboard/web/README.md).

---

## Terminal 2 — scripts and ROS CLI (host)

While Terminal 1’s launch is running, open a **second terminal on the host**:

```bash
cd docker
docker exec -it go2_dev bash
source /ros2_ws/install/setup.bash
```

Examples:

```bash
# Inspect
ros2 topic list
ros2 topic echo /map --once
ros2 topic echo /amcl_pose

# Send a navigation goal (navigation mode, map loaded)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
{pose: {header: {frame_id: map},
  pose: {position: {x: 2.0, y: 1.0, z: 0.0},
         orientation: {w: 1.0}}}}"

# Driver params
ros2 param set /go2_driver_node obstacle_avoidance true
```

Do **not** start a second full launch here while Terminal 1 already runs one — two drivers will conflict on WebRTC.

---

## Switching launch files

| Action | What happens |
|--------|----------------|
| `Ctrl+C` in Terminal 1, then new `ros2 launch ...` | Driver stops → WebRTC drops → new launch reconnects (~few seconds). Dog hardware does not reboot. |
| Terminal 2 only runs `ros2 topic`, `ros2 action`, `ros2 service` | Driver stays up; safe while Terminal 1 launch runs. |

To swap mapping → navigation:

1. `Ctrl+C` in Terminal 1 (disconnects).
2. Save map in RViz if you were mapping (files go to `docker/data/` on the host).
3. Start navigation launch in Terminal 1:

   ```bash
   ros2 launch go2_robot_sdk navigation.launch.py map:=/ros2_ws/data/my_map.yaml
   ```

---

## Maps and data

Nav2 navigation needs a **2D occupancy map** from slam_toolbox (`.yaml` + `.pgm`). That is **not** the same as a `.ply` LiDAR dump.

| File | Source | Used for |
|------|--------|----------|
| `my_map.yaml` + `my_map.pgm` | Web UI **Save Map** or slam_toolbox | **Navigation / AMCL** |
| `my_map.ply` | LiDAR node when `MAP_SAVE=true` | Debug only — **not** Nav2 |

### Create a Nav2 map (first time or after loss)

```bash
# Terminal 1 — mapping mode (working dir should be /ros2_ws/data)
cd /ros2_ws/data
ros2 launch go2_robot_sdk mapping.launch.py rviz:=false
```

1. Open http://localhost:8080
2. Drive the dog with the joystick until the 2D map looks complete
3. Enter map name `my_map` → click **Save Map** (creates `my_map.yaml` + `my_map.pgm`)
4. Optionally click **Serialize Map** (`.data` / `.posegraph` for continuing SLAM later)
5. `Ctrl+C`, then start navigation:

```bash
ros2 launch go2_robot_sdk navigation.launch.py map:=/ros2_ws/data/my_map.yaml
```

On the host, files appear under `docker/data/`.

### "No map" / empty RViz / costmap errors

If you only see `my_map.ply` in `docker/data/`, you have a LiDAR dump but **no Nav2 map**. Re-run mapping mode and **Save Map** as above.

Check inside the container:

```bash
ls -la /ros2_ws/data/*.yaml /ros2_ws/data/*.pgm
ros2 topic echo /map --once   # should show width/height after navigation launch
```

Set `MAP_NAME`, `MAP_FILE` via `docker-compose.yml` environment if needed.

---

## Stop everything

```bash
# Terminal 1: Ctrl+C if a launch is running, then exit shell

# On host
cd docker
docker compose down
docker rm -f go2_dev   # if you used docker compose run --name go2_dev
```

---

## Quick reference

```bash
# Host — start dev container
cd docker && export ROBOT_IP=...
docker compose run -d --name go2_dev unitree_ros sleep infinity

# Host — shell
docker exec -it go2_dev bash

# Container — rebuild after edits
cd /ros2_ws && colcon build && source install/setup.bash

# Container — run stack (Terminal 1)
ros2 launch go2_robot_sdk mapping.launch.py rviz:=false
# Browser: http://localhost:8080  (prod)  or  web_dev:=true → :5173

# Host — second shell for CLI (Terminal 2)
docker exec -it go2_dev bash
```

---

## License

This project is licensed under the BSD 2-clause License — see [LICENSE](LICENSE).
