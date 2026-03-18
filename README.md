# Group 7 — Autonomous Mobile Robot (AMR)

ROS 2 Humble workspace for the CDE2310 AMR project. The robot autonomously
explores an arena, detects AprilTag targets, and delivers a ping-pong ball
using a spring-loaded launcher.

## Build

```bash
source /opt/ros/humble/setup.bash
cd ~/Group7_AMR
colcon build
source install/setup.bash
```

Incremental build for a single package:

```bash
colcon build --packages-select amr_navigation
```

## Packages

| Package | Description |
|---|---|
| `amr_navigation` | SLAM, frontier detection, exploration, coverage monitoring |
| `amr_perception` | AprilTag detection, camera processing, docking logic |
| `amr_launcher` | Servo & spring actuation, ball delivery |
| `amr_bringup` | Mission orchestration and launch files |
| `amr_utils` | Legacy lab exercise nodes for testing/debugging |

## Repository Layout

```
Group7_AMR/            ← colcon workspace root
├── src/               ← ROS 2 packages (colcon builds these)
│   ├── amr_navigation/
│   ├── amr_perception/
│   ├── amr_launcher/
│   ├── amr_bringup/
│   └── amr_utils/
├── hardware/          ← CAD files (COLCON_IGNORE)
├── docs/              ← Reports & end-user docs (COLCON_IGNORE)
└── data/              ← Recorded bags, maps (COLCON_IGNORE)
```

## Team

- Jeon Won Je — Systems lead
- Clara — Perception
- Kumaresan — Navigation
- Shashwat — Launcher
- Daniel — Integration
