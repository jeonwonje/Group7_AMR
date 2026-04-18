# Subsystem Design

| Field          | Value                                              |
|----------------|----------------------------------------------------|
| Document ID    | AMR-SSD-001                                        |
| Version        | 1.0                                                |
| Date           | 2026-04-13                                         |
| Author(s)      | Group 7 — Jeon, Kumaresan, Clara, Shashwat, Daniel |
| Module         | CDE2310 Engineering Systems Design                 |
| Status         | Baselined for G2                                   |

---

## 1  Purpose

This document provides detailed design information for each subsystem of the
Group 7 AMR: navigation, perception, docking, delivery, and mission coordination.
For each subsystem the document specifies purpose, key algorithms, tunable
parameters, state machines, and ROS 2 interfaces.

---

## 2  Subsystem Descriptions

### 2.1  Navigation Subsystem

#### 2.1.1  Exploration — `auto_explore_v2`

**Owner:** Kumaresan

**Purpose:** Autonomously cover the maze by repeatedly selecting and navigating
to the most promising frontier.

**Nodes:**

| Node             | File                | Role                              |
|------------------|---------------------|-----------------------------------|
| `auto_explore`   | `find_frontiers.py` | BFS frontier detection + clustering |
| `score_and_post` | `score_and_post.py` | Frontier scoring, Nav2 goal dispatch |

**Algorithm — Frontier Detection (find_frontiers.py):**

1. Subscribe to `/map` (OccupancyGrid).
2. Build a dictionary `(x, y) → cell_value` for the full grid.
3. For each free cell (value = 0), check 4-connected neighbours.
   If any neighbour is unknown (value = −1), the cell is a frontier.
4. Flood-fill (BFS) to cluster contiguous frontier cells.
   Clusters smaller than `FRONTIER_MIN_SIZE` (3) are discarded.
5. Publish cluster centroids and the BFS distance transform on
   `frontiers` and `bfs_distance_transform` topics (JSON-encoded).

**Algorithm — Frontier Scoring (score_and_post.py):**

1. For each cluster centroid, compute:
   - `bfs_dist`: cost from BFS distance transform (lower is closer).
   - `size`: number of cells in the cluster (larger frontier = more information).
2. Score = f(size, bfs_dist) — larger clusters closer to the robot are preferred.
3. Optionally pre-flight the top candidate via `ComputePathToPose` action.
   If the path is blocked (occupancy ≥ 51), discard and try the next candidate.
4. Post the winning goal to Nav2 via `NavigateToPose` action.
5. On completion or timeout, re-score and repeat.
6. When no clusters remain, publish `EXPLORATION_COMPLETE` on `/mission_status`.

**Toggle service:** `toggle_exploration` (SetBool) — the mission coordinator
pauses/resumes exploration during docking/delivery.

**Parameters:**

| Parameter              | Value | Unit  | Description                          |
|------------------------|-------|-------|--------------------------------------|
| FRONTIER_MIN_SIZE      | 3     | cells | Minimum cluster size to consider     |
| PATH_BLOCKED_OCC_MIN   | 51    | —     | Occupancy threshold for blocked path |
| PREFLIGHT_TIMEOUT_SEC  | 10.0  | s     | Timeout for ComputePathToPose check  |

---

*Note: A custom Nav2-free navigation package (`amr_nav`) was explored during
development but ultimately removed in favour of the Nav2-based stack (see
CHANGELOG 1.1.0). Its Dijkstra/A* algorithms and earlier pathfinding unit tests
are recoverable from git history (commit `044e346`).*

---

### 2.2  Perception Subsystem — External `apriltag_ros`

**Owner:** Clara (integration, calibration, configuration)

**Purpose:** Detect tag36h11 AprilTag markers in the RPi camera feed, compute
6-DOF poses, and broadcast the resulting transforms to the TF tree so that
mission, docking, and delivery nodes can react to tag geometry.

**Provenance:** The team does **not** ship a bespoke `apriltag_detector` node.
Detection and pose estimation are provided by the upstream
[`apriltag_ros`](https://github.com/christianrauch/apriltag_ros) ROS 2 package,
launched on the RPi alongside the camera driver. The team's contribution to
perception is limited to: (a) calibrating the RPi Camera V2 intrinsics,
(b) configuring the detector's tag family and size, and (c) wiring its output
into the mission stack.

**Pipeline:**

1. The RPi camera driver publishes `/camera/image_raw` (Image) and
   `/camera/camera_info` (CameraInfo).
2. `apriltag_ros` subscribes to both, runs tag36h11 detection, and solves
   each tag's 6-DOF pose using the calibrated intrinsics.
3. It publishes:
   - `/detections` (`apriltag_msgs/AprilTagDetectionArray`) — full per-tag
     detection payload.
   - TF frames `camera_link → tag36h11:<id>` for each visible tag.
4. Downstream consumers:
   - `docker` reads tag TF frames to compute lateral/yaw error for servoing.
   - `delivery_server` subscribes to `/detections` for dynamic-station
     crosshair logic against tag36h11:3.
   - `mission_coordinator` monitors the TF tree with a 0.5 s staleness
     threshold before acting on a tag.

**Configuration:**

| Parameter              | Value        | Description                                         |
|------------------------|--------------|-----------------------------------------------------|
| Tag family             | tag36h11     | Per mission brief.                                  |
| Tag size               | 0.16 m       | Physical side length; must match `apriltag_ros` YAML.|
| Camera intrinsics      | Calibrated   | Produced via `camera_calibration`; loaded by driver.|
| Target tag IDs         | 0, 2, 3      | Static station, dynamic station, moving target.     |

**Robustness:** Mission-side consumers guard against stale detections with a
0.5 s TF age limit and, for docking, a 1.0 s camera-dropout coast window.

---

### 2.3  Docking Subsystem — `CDE2310_AMR_Trial_Run/docker.py`

**Owner:** Shashwat

**Purpose:** Execute precision docking from a staging waypoint to the tag face
using discrete geometric visual servoing.

**Node:** `docking_server` (docker.py)

**Algorithm — Geometric Visual Servoing:**

The docking server replaces continuous PID control with a 3-phase discrete
state machine:

```
  Nav2 staging goal (0.40 m from tag)
         │
         ▼
  ┌──────────────────┐
  │  INTERCEPT        │  Drive at a slant to reduce lateral (Y) error.
  │                   │  Y tolerance: 0.03 m
  │  Abort boundary:  │  If X < abort_ratio × staging → backup
  └────────┬──────────┘
           │ Y error ≤ tolerance
  ┌────────▼──────────┐
  │  SQUARE_UP         │  Rotate in place to align yaw with tag normal.
  │                   │  Yaw tolerance: 0.05 rad (≈ 3°)
  └────────┬──────────┘
           │ yaw aligned
  ┌────────▼──────────┐
  │  FINAL_PLUNGE      │  Drive straight until stop_distance (0.10 m).
  │                   │  If Y > max_allowed_y_error → backup + retry
  └────────┬──────────┘
           │
           ▼
  DOCKING_COMPLETE / DOCKING_FAILED
```

**Parameters:**

| Parameter              | Value  | Unit  | Description                            |
|------------------------|--------|-------|----------------------------------------|
| staging_distance       | 0.40   | m     | Nav2 drop-off distance from tag        |
| stop_distance          | 0.10   | m     | Final approach stop from tag           |
| intercept_ratio        | 0.7    | —     | Dynamic lookahead fraction             |
| abort_ratio            | 0.3    | —     | Hard safety boundary fraction          |
| intercept_y_tolerance  | 0.03   | m     | Centreline alignment tolerance         |
| square_yaw_tolerance   | 0.05   | rad   | Yaw alignment tolerance                |
| slow_linear_speed      | 0.03   | m/s   | Approach speed                         |
| max_angular_speed      | (cap)  | rad/s | Rotation speed cap                     |
| max_docking_time       | 180    | s     | Hard timeout for entire sequence       |
| sensor_drop_tolerance  | 1.0    | s     | Camera dropout coast time              |
| fallback_staging_offset| 0.15   | m     | Subtracted on Nav2 rejection retry     |

**Robustness:**
- Camera dropout: coast on last-known TF for up to `sensor_drop_tolerance` (1 s).
- Nav2 rejection: subtract `fallback_staging_offset` (0.15 m) and retry once.
- Timeout: abort after `max_docking_time` (180 s).
- Backup: if Y error too large at plunge phase, reverse at `backup_speed` for
  `backup_duration`, then retry (up to `max_retries`).

---

### 2.4  Delivery Subsystem

**Owner:** Clara (launcher), Jeon (delivery_server)

#### 2.4.1  delivery_server (`CDE2310_AMR_Trial_Run/delivery_server_consolidated.py`)

**Purpose:** Orchestrate ball delivery for both static and dynamic stations.

**Static Station Protocol (tag36h11:0):**
1. Receive `START_DELIVERY` command.
2. Fire ball → wait 4 s → fire → wait 6 s → fire.
3. Publish `DELIVERY_COMPLETE`.

**Dynamic Station Protocol (tag36h11:2):**
1. Receive `START_DELIVERY` command.
2. Subscribe to `/detections` (AprilTagDetectionArray).
3. When tag ID 3 is detected, fire immediately.
4. Enter 4 s cooldown.
5. Repeat until 3 shots fired or `max_dynamic_shots` reached.
6. Publish `DELIVERY_COMPLETE`.

**Parameters:**

| Parameter          | Value | Unit | Description                         |
|--------------------|-------|------|-------------------------------------|
| cooldown_seconds   | 4.0   | s    | Cooldown between dynamic shots      |
| max_dynamic_shots  | 3     | —    | Maximum shots at dynamic station    |
| TARGET_TAG_ID      | 3     | —    | Tag ID for dynamic station target   |

#### 2.4.2  Servo Control (consolidated in delivery_server)

The delivery_server directly controls the MG90 servo via GPIO 12 (hardware PWM)
on the RPi. There is no separate shooter node or `/fire_ball` service.

**Mechanism:** Servo rotates the spur gear, which drives the rack-and-pinion
plunger. The plunger compresses the spring and releases, propelling the ball
out of the barrel via a gravity-fed tube (7-ball capacity).

---

### 2.5  Mission Coordination — `mission_coordinator_v3`

**Owner:** Kumaresan (v1–v3), Jeon (robustness patches)

**Purpose:** Central FSM that orchestrates all subsystems and manages the
mission lifecycle.

**State Machine:**

```
  INIT ──► EXPLORING ──tag seen──► DOCKING ──success──► DELIVERING
               ▲                      │                      │
               │                      │ fail (blacklist)     │
               │                      ▼                      ▼
               └──── resume ◄──── EXPLORING              UNDOCKING
               │                                             │
               └──── resume ◄────────────────────────────────┘
               │
               ▼ (exploration complete, tags remain)
           SEARCHING ──tag seen──► DOCKING ──► ...
               │
               ▼ (all zones exhausted)
           MISSION_COMPLETE
```

**Key Mechanisms:**

| Mechanism             | Implementation                                          |
|-----------------------|---------------------------------------------------------|
| Tag monitoring        | Poll TF tree at 10 Hz for target tags                   |
| Staleness filtering   | Reject TF transforms older than 0.5 s                  |
| Blacklisting          | HashMap `{tag: expiry_time}`, 30 s duration             |
| Exploration toggle    | SetBool service call to `toggle_exploration`            |
| Command dispatch      | JSON on `/mission_command` topic                        |
| Status listening      | JSON on `/mission_status` topic                         |
| Search fallback       | Dispatches `START_SEARCH` with docked tag list          |
| Completion detection  | `docked_tags == target_tags` → MISSION_COMPLETE         |

---

### 2.6  Search Subsystem — `search_stations`

**Owner:** Kumaresan

**Purpose:** When exploration is complete but tags remain un-serviced, navigate
to pre-computed zones and spin to scan for the missing tag.

**Algorithm:**

1. On `START_SEARCH`, compute absolute search zones from relative offsets
   anchored to the robot's start position.
2. For each zone:
   a. Snap the zone coordinate to the nearest free cell on the occupancy grid
      (within `max_safe_search_radius` = 1.5 m).
   b. Navigate to the safe cell via Nav2 `NavigateToPose`.
   c. On arrival (within `arrival_tolerance` = 0.4 m), spin 360° (0.5 rad/s × 13 s).
   d. If a tag interrupt occurs during spin, abort search.
3. If all zones exhausted with no detection → publish `SEARCH_FAILED`.

**Parameters:**

| Parameter              | Value     | Unit  | Description                    |
|------------------------|-----------|-------|--------------------------------|
| relative_search_offsets| [(-0.75,-0.3),(1.25,2.7)] | m | Offsets from start pose |
| max_safe_search_radius | 1.5       | m     | Max snap distance for free cell|
| spin_velocity          | 0.5       | rad/s | Rotation speed for scan        |
| spin_duration          | 13.0      | s     | Duration (covers > 360°)       |
| max_nav_retries        | 3         | —     | Nav2 retry limit per zone      |
| arrival_tolerance      | 0.4       | m     | Distance to trigger spin       |

---

## 3  Revision History

| Version | Date       | Author | Changes            |
|---------|------------|--------|--------------------|
| 1.0     | 2026-04-13 | Jeon   | Initial baseline   |
