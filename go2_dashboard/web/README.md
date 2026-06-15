# Go2 Dashboard (React + Vite)

## Development (hot reload)

```bash
cd src/go2_dashboard/web
npm install
npm run dev
```

Open `http://localhost:5173`. The app connects to rosbridge at `ws://<hostname>:9090` — start mapping/navigation (or `web_ui.launch.py` for rosbridge only) first.

From ROS with auto-reload via launch:

```bash
ros2 launch go2_dashboard web_ui.launch.py web_dev:=true
```

## Production build

```bash
npm run build
```

`colcon build --packages-select go2_dashboard` runs this automatically when `npm` is available.

Served on port 8080 via `web_ui.launch.py` (default).

## 3D point cloud

The LiDAR pane uses a **CPU-bound Canvas 2D** renderer (orbit: drag, zoom: scroll). No WebGL or GPU drivers are required — works on headless VMs, remote desktops, and `llvmpipe` setups.
