# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Gazebo simulation support: maze world, AprilTag models, full-mission sim launch (`gazebo_mission.launch.py`).
- `nav_tuner.launch.py` and `slam_test.launch.py` for isolated Nav2 and SLAM tuning.
- Mechanical hardware documentation completed in `hardware/README.md`.
- `apriltag_docking` ROS 2 package (`src/apriltag_docking/`): custom RPi-side perception pipeline composing `camera_ros`, `image_proc` (resize + rectify), and `apriltag_ros` into a zero-copy container, plus per-station static dock-target TF frames and `detected_dock_pose_publisher` nodes that publish `/detected_dock_pose_{0,2}` (PoseStamped) consumed by the Nav2 docker.
- `docs/MANUFACTURING.md`: operator-facing Bambu Lab P2S `.3mf` print workflow for reproducing the team's 3D-printed parts (default Bambu Studio preset + tree supports), linked from the slim README and complementing Daniel's `hardware/README.md`.

### Changed
- Nav2 global planner switched to Dijkstra; global and local costmap inflation radii split and retuned for tight corridors.
- Documentation rewritten against the actual codebase: removed references to non-existent nodes (`apriltag_detector`, `launcher_node`, `rpi_shooter_node`, `static_station`, `auto_nav`). AprilTag detection now lives in the team-authored `apriltag_docking` package, which wraps the upstream `apriltag_ros` detector.
- `README.md` reduced from ~585 lines to a ~70-line doc index pointing at the split SDD under `docs/reports/`; all intra-README anchor links dropped.
- End User Documentation rewritten for the printed 5-page Final Mission hand-in: corrects platform (Burger, not Waffle Pi), actuator (XL430-W210), ball capacity (7, not 3), docking distances (staging 0.40 m / stop 0.10 m), and deployment flow (2 terminals, not 7). §3 Acceptable Defect Log and §5 Maintenance & Part Replacement Log populated.
- Subsystem Design §4.3 Docking expanded from a 3-phase description to the real 8-state FSM in `docker.py` (IDLE → NAV_TO_STAGING → COMPUTE_GEOMETRY → INTERCEPT → SQUARE_UP → EVALUATE_POSITION → RETRY_BACKUP → FINAL_PLUNGE → UNDOCKING).
- Mission FSM diagrams: removed phantom "map exhausted" branch. Documented the real transitions — exploration-timeout to SEARCH (480 s default), master-timeout to MISSION_TIMED_OUT (1200 s), all-stations-serviced to MISSION_COMPLETE. `EXPLORATION_COMPLETE` / `"explorer"` status values in the ICD are now annotated as dangling contracts (never emitted by any node).
- Hardware-Software Mapping: removed non-existent buck converter; RPi is powered directly by the OpenCR 5 V regulator (NFR-07, ICD GND row, HLD Layer-1 diagram).
- Tag edge length corrected from 0.16 m to 0.0986 m across all reports (per `apriltag_docking/config/apriltags_36h11.yaml`).
- ICD TF tree redrawn with the real `camera_link → camera → tag36h11:{0,2} → nav2_dock_target_{0,2}` chain; bogus `tag36h11:3` TF branch removed (tag 3 is inspected via `/detections` only, never broadcast as a frame).
- ICD topic table: added `/detected_dock_pose_{0,2}`, `/camera/resized/{image_raw,camera_info,image_rect}`, and `/camera/camera_info`; corrected publisher for `/detections`. ICD services: added `clear_blacklist` (std_srvs/Empty, score_and_post → mission_coordinator).
- Testing §3 Unit & Lint Tests consolidated and honest: unit coverage is limited to ament lint scaffolds; pathfinding / frontier-scoring / docking-geometry suites flagged as planned-but-unimplemented (TST-UT-01…29 phantom rows removed).
- Software Development §Dependencies split into Team Packages and Upstream Deps, reflecting `apriltag_docking` as team-authored with the full upstream list pulled from its `package.xml`. AI-attribution section bumped to Claude Opus 4.7.
- Subsystem Design §2.6 Search: `max_safe_search_radius` doc-code mismatch fixed (0.6 m per `search_stations.py:35`, previously documented as 1.5 m).

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
