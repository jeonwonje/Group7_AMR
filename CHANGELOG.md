# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Gazebo simulation support: maze world, AprilTag models, full-mission sim launch (`gazebo_mission.launch.py`).
- `nav_tuner.launch.py` and `slam_test.launch.py` for isolated Nav2 and SLAM tuning.
- Mechanical hardware documentation completed in `hardware/README.md`.

### Changed
- Nav2 global planner switched to Dijkstra; global and local costmap inflation radii split and retuned for tight corridors.
- Documentation rewritten against the actual codebase: removed references to non-existent nodes (`apriltag_detector`, `launcher_node`, `rpi_shooter_node`, `static_station`, `auto_nav`). AprilTag detection is provided by the external `apriltag_ros` package — not a team-written node.

### Fixed
- Reverted a Gazebo-only tuning of `minimal_nav2.yaml` that had degraded real-robot behaviour.

## [1.2.0] - 2026-04-17

### Added
- Consolidated `delivery_server` node (`delivery_server_consolidated.py`) handling both static timed delivery (tag 0) and dynamic tag-3 reactive firing on the RPi.
- `gazebo_mission.launch.py` integrating TurtleBot3 spawn, Cartographer, Nav2, and the full mission stack in simulation.
- `fire_test.py` servo-preload bench utility.
- Semver release tags (`v1.0.0`, `v1.1.0`, `v1.2.0`).
- README rewritten as the single consolidated project document (architecture, launch instructions, end-user documentation).

### Changed
- Mission timeout management centralised in `mission_coordinator_v3` (replaces per-subsystem timers).
- Nav2 controller retuned for tight corridors: raised angular velocity/acceleration caps, tightened inflation radius.
- Docking and search-zone parameters refined after live-robot testing.

### Removed
- **Breaking:** separate `launcher_node` and `rpi_shooter_node` modules — functionality merged into `delivery_server_consolidated`. The `/fire_ball` service and shooter-node launch wiring no longer exist.

## [1.1.0] - 2026-04-13

### Added
- G2 systems engineering documentation under `docs/reports/`: ConOps, requirements specification, high-level design, subsystem design, interface control document, software development plan, testing documentation.
- End-user documentation under `docs/end_user_doc/`.

### Fixed
- Missing console-script entry points added to `setup.py`.

### Removed
- **Breaking:** legacy scaffold packages `amr_navigation`, `amr_perception`, `amr_launcher`, `amr_bringup`, `amr_utils`, and the later `amr_nav` package — superseded by `CDE2310_AMR_Trial_Run` and `auto_explore_v2`. The 29 pathfinding unit tests from `amr_nav/test/test_pathfinding.py` are recoverable from git history (commit `044e346`) but are no longer shipped.
- `archive/` directory.

## [1.0.0] - 2026-04-13

Initial integrated-system release. First end-to-end assembly of navigation, exploration, docking, delivery, and mission coordination.

### Added
- `auto_explore_v2` package: BFS frontier detection (`find_frontiers`) and frontier scoring with Nav2 goal posting plus `ComputePathToPose` pre-flight validation (`score_and_post`).
- `CDE2310_AMR_Trial_Run` package:
  - `mission_coordinator` FSM orchestrating explore → dock → deliver → undock → search.
  - `docking_server` — discrete geometric visual-servoing state machine (stage → intercept → square-up → plunge) with blacklist-on-failure.
  - `search_stations` — zone-sweep fallback with 360° scans.
  - `delivery_server`, initial `launcher_node`, and RPi shooter node for servo PWM ball delivery (later consolidated in 1.2.0).
  - Static and dynamic station handling.
- Cartographer SLAM config, minimal Nav2 params, and launch files (`mission.launch.py`, `full_mission.launch.py`, `minimal_nav2.launch.py`).
- Early custom Dijkstra/A* exploration implementation — superseded by the Nav2 + BFS scoring stack above.
- TurtleBot3 assembly CAD (`hardware/chassis/`) and launcher mechanism CAD — barrel, plunger (face + rack), spur gear, V1/V2 prototypes (`hardware/launcher/`).
- 3MF manufacturing files for 3D printing.
- Repository scaffolding: `src/`, `hardware/`, `docs/`, `data/`; `CLAUDE.md`, `AGENT_GIT_GUIDE.md`, `README.md`, `.gitignore`, `requirements.txt`.
