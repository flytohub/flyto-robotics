# Changelog

All notable project changes are recorded here.

## 0.5.0 — 2026-10-05

- Straight drives (`motion.advance`, `motion.retreat`) under `lidar_clearance` are guarded on every scan by a braking envelope (`flyto_robotics/braking_envelope.py`): they stop while the room left before any return reaches the 0.35 m floor is still at least `v * t_latency + v^2 / (2 * a_decel)`. Until now the floor was checked only before a motion started; while it ran, the only stop was Nav2's own DriveOnHeading collision check (footprint radius plus `simulate_ahead_time * v` against the local costmap), which knows nothing of the floor. On the physical TurtleBot3 a 2.0 m advance ended with a return 0.181 m from the LiDAR.
- The room is measured along the way the robot is going and includes returns beside the path it would pass closer than the floor (`x - sqrt(floor^2 - y^2)`).
- A drive starts at no more than `0.8 * v_max(room)` and is re-sent slower, as a preempting goal with the rest of its distance, as the room shrinks (rosbridge transport; the rclpy transport stops but does not re-send). A drive that could not stop at the floor from 0.02 m/s is refused before it moves. An unreadable or quiet LiDAR stops a guarded drive (`sensor_stale`).
- `t_latency` is the larger of `FLYTO_ROS2_STOP_LATENCY_S` (0.5 s) and the measured budget (scan interval, scan delivery delay each way, control period, actuation); `a_decel` the smaller of `FLYTO_ROS2_STOP_DECEL_MPS2` (0.5 m/s^2) and the robot's declared `/velocity_smoother:max_decel`.
- `motion_outcome` adds `braking` (profile and sources, speeds, slowdowns, trip, measured stop distance) and `stop_clearance` (nearest return at rest with its bearing and side from the direction of travel, `floor_held`, `travel_floor_held`); the operator line names the side and says whether the floor held. A guard stop reports `obstacle_blocked` (or `sensor_stale`), not `cancelled`. All additive.
- An adapter serves only the resource configured on its computer. `adapter_provider.build_adapter(resource_id)` and the process protocol refuse any id but `FLYTO_ROS2_RESOURCE_ID` (`ResourceNotServed`) instead of labelling the adapter with the id they were handed: live, a job for `turtlebot3-twin` (simulation) was built an adapter on a computer reconfigured for the physical `burger-01`, ran on the robot and was recorded as the twin.
- `GenericROS2Adapter.served_identity()` (process op `served_identity`) returns `{"resource_id", "deployment_mode"}` for the Flyto2 Cloud host's pre-job check: the configured resource, and `simulation`/`real` confirmed against the live graph (`/clock` iff simulation). A mode the graph contradicts, an empty graph, or simulation claimed with the marker check disabled raises `ServedIdentityError`.

## 0.4.1 — 2026-10-05

- An inflation escape goes by its waypoint and then the goal as two NavigateToPose legs. One NavigateThroughPoses goal was refused in under a second on TurtleBot3 Jazzy, whose stock configuration routes through-poses goals to a behaviour tree that reads a single goal, so the planner was handed an empty pose ("Failed to transform from  to map"). Measured live on the twin.

## 0.4.0 - 2026-10-04

- `motion.navigate` checks, before sending a goal, whether the robot starts inside an obstacle's costmap inflation (`flyto_robotics/inflation_escape.py`): the nearest LiDAR return in the front sector against `inflation_radius + robot_radius`, both read from the live costmaps' parameters (`/local_costmap/local_costmap`, `/global_costmap/global_costmap` via the standard `get_parameters` service; footprint taken as its circumscribed radius; Nav2's defaults 0.55 m / 0.1 m when unreadable, reported as `costmap.source`).
- A pinned start escapes first: a straight back-off (Nav2 `BackUp`, 0.05 m/s) by `clamp(threshold - front, 0, FLYTO_ROS2_ESCAPE_MAX_BACKOFF_M)`, only as far as the rear sector keeps the 0.35 m floor; then one lateral waypoint on the side with more LiDAR room, offset past the obstacle's extent (`range * sin(bearing)`) by the robot radius plus `FLYTO_ROS2_ESCAPE_MARGIN_M`, then the original goal. One `NavigateThroughPoses` goal when the graph has it, else sequential `NavigateToPose` goals. The original goal stays the final pose under the call's own id, so arrival is still judged against the real target. Every straight escape segment keeps the floor from every LiDAR return; an unreadable sector is never room.
- No safe escape (no side clears the floor, or the obstacle is wider than `FLYTO_ROS2_ESCAPE_MAX_LATERAL_M`): refused before any motion with `reason_code: no_escape_room`, a detail led by it, and the measured clearances, instead of Nav2 thrashing for minutes.
- Every navigate result carries `evidence.navigation_escape` (decision, clearances, costmap geometry, back-off, side, waypoint, legs) when a LiDAR sweep was available. `FLYTO_ROS2_INFLATION_ESCAPE=off` disables the check. A cancel during an escape leg withdraws that leg and stops the rest.

## 0.3.0 - 2026-10-04

- Named places, kept on the execution host (`flyto_robotics/places.py`, `flyto.robot-places.v1`): one JSON file per robot per map (`FLYTO_ROS2_PLACES_FILE`, or `places/<resource>/<FLYTO_ROS2_MAP_ID>.json` under `FLYTO_ROBOTICS_DATA_DIR` / the XDG data dir), entries `{name, frame: "map", x, y, yaw}`, free-text names unique per map ignoring case, atomic writes, and a file that cannot be read exactly as written refused (never overwritten).
- `places.list` (read-only; `evidence.places` and a `places` `application/json` artifact) and `places.mark` (saves the current `map_pose`; the same call id returns its first result) are declared beside `motion.navigate`.
- `motion.navigate` takes `place` as an alternative to `x`/`y`; exactly one target, and `yaw_radians` only with `x`/`y`. An unknown place is refused with `evidence.known_places`, an unreadable file is refused, both before any motion. Every navigate result carries `evidence.navigation_target`; a call by place carries `evidence.resolved_arguments`. A resumed call keeps the target it was first sent to. `navigate`'s declaration schema hash changes (x and y are no longer individually required); declarations without text arguments hash as before.
- `DeclaredArgument.max_length` for text arguments (emitted only when set); JSON schemas give text `minLength`/`maxLength`.
- The Generic ROS 2 adapter reports the robot's pose in the map frame (`map_pose`) beside odometry whenever a fresh map->odom transform exists: in observation bundles (optional field), motion evidence, `motion_outcome` (`start_map_pose`, `final_map_pose`, and the operator line) and `recovery_context`. Odometry fields and motion verification are unchanged.

## 0.2.0 - 2026-10-04

- Results carry the evidence the adapter can vouch for, additively: `evidence_items` (`passage.clearance` against the adapter's own 0.35 m floor; `robot.arrival` with odometry before, after and once settled, in the exact shape Desktop projected), `artifacts` (photo JPEG, occupancy map drawn as JPEG with Pillow or PNG without, in the `flyto.capability-contract.v1` transport) beside the legacy `capture`, and `recovery_context` on a failed or timed-out advance/retreat (reason, travelled vs requested along the heading, ranges, sweep at the stop).
- The adapter keeps the observation from before a motion per call id and returns the same enriched result for a repeated call id.
- `open_rmf.fleet` external adapter and resource discoverer entry points: a `fleet:<name>` resource, requests pinned to that fleet (never a robot), calls that wait for Open-RMF's terminal task state up to the deadline, no second dispatch for a repeated call id, `robot.arrival` for a finished navigate/dock, `deployment_mode` from `FLYTO_RMF_DEPLOYMENT_MODE`.
- Optional `capture` extra (`Pillow>=10`).

- A motion's reason now follows the collision monitor's state at the stop (seeded from the state in force at send), so a cleared slowdown no longer hides a specific Nav2 error; failed and timed-out rosbridge motions return without waiting for quiet sensors, so the host safe stop is not delayed.
- Every Generic ROS 2 motion result carries `evidence.motion_outcome` with a machine-readable `reason` (`obstacle_blocked`, `sensor_stale`, `timeout`, ...), start/final pose, distance travelled vs requested and the nearest LiDAR return at the stop; failed motions lead their detail with it.
- Ship Generic ROS2/rosbridge, OpenRMF and vision-stream integrations as external adapter providers usable by the built-in AI Space host or optional process hosts such as Flyto2 Runtime, instead of Cloud-bundled transport implementations.
- Add canonical `flyto2-adapter-provider-ros2-generic` packaging and passive `flyto.resource-manifest.v1` discovery while keeping assignment authority and the 0.35 m motion safety floor separate.

- Reframe production around a standard ROS 2 robot controlled from an external
  AI Space computer. The robot is commanded equipment, not a Flyto2 worker.
- Remove the complete retired Pi-appliance implementation from current source:
  job runner, delivery gateway, lifecycle installer/profile registry, robot
  doctor, recovery portal, Flyto-specific watchdog, credential provisioning,
  and Flyto-specific robot systemd units.
- Keep only optional upstream-only ROS 2 site configuration examples under
  `deploy/native_ros2/`; no Flyto2 package or credential is required on a
  TurtleBot3.
- Add `flyto.robotics.ros2-observation-bundle.v1` so simulation and physical
  resources share one pose/LiDAR/camera/map-TF observation and provenance
  contract.
- Treat uncalibrated camera frames as valid raw visual evidence while refusing
  to claim metric/geometric vision readiness without a bound calibration
  snapshot.
- Separate upstream SLAM Toolbox and Nav2 startup, run Nav2 non-composed with
  respawn, and retain standard Nav2 action semantics for external adapters.
- Use the camera's native YUYV encoding on the physical Pi to avoid unnecessary
  RGB conversion load.
- Record the physical middleware convergence from Fast DDS to CycloneDDS after
  a real data-plane stall left graph discovery visible while new local
  subscribers could not receive odom, LiDAR, or camera samples.
- Add an external native-ROS2 soak probe that can be streamed over stdin for
  hardware acceptance without installing Flyto2 code on the robot.
- Replace current installation, architecture, state, security, and contract
  guidance so a clean upstream reinstall requires only standard ROS 2
  rediscovery from the external execution host.
- Preserve retired architecture evidence only in dated handoffs and Git
  history; it is no longer executable current source.

## 0.1.0 — 2026-07-28

- Created the independent `flyto-robotics` ROS 2 package.
- Added versioned, transport-neutral job and result contracts.
- Added atomic `navigate` and `dwell` primitives with injectable workflows.
- Expanded the executable vocabulary with `follow_line`, `wait_until_clear`,
  `ask_human`, `resume`, and `safe_stop`.
- Added strict AI-plan policy for terminal stopping, line-transition
  consistency, and paired human approval/resume gates.
- Added the signed human-decision contract, HMAC-SHA256 verification, short
  expiry, job/robot binding, and nonce replay rejection.
- Added the conditional ROS `/flyto/human_decision` adapter and signing CLI.
- Added structured sequence, step, capability, and actor audit evidence.
- Added namespaced capability manifests, runtime compatibility hard filters,
  language-neutral Goal Frames, canonical affordance/effect ranking, bounded
  LLM shortlists, registry snapshots, semantic coverage, ambiguity evidence,
  and shortlist enforcement.
- Added transport-neutral integration contracts for Flyto AI, trusted
  Blueprint hints, and scoped Flyto Core discovery.
- Added atomic `save_current_location` and `navigate_to_location` abilities.
- Added map-scoped semantic location storage, optimistic revisions, bounded
  Unicode labels, atomic writes, and fail-closed map identity checks.
- Added separate full-map and coordinate-free planner-catalog contracts.
- Added multilingual Goal Frame, semantic map, teaching, navigation, CLI, and
  Gazebo launch examples without changing existing atom contracts.
- Added the deterministic controller, safety stops, ROS adapter, and CLI.
- Added the self-contained hospital world and differential-drive lidar rover.
- Added Jazzy/Harmonic container verification and CI.
- Verified a complete Gazebo pharmacy-to-ward mission on Linux ARM64.
- Verified the signed CareFlow human-gate mission in Gazebo physics on Linux
  ARM64, including successful resume and replay rejection.
- Added bounded overhead-camera frame recording and reproducible H.264 encoding
  through `make gazebo-video`.
- Verified a 19.0-second, 960×540 Gazebo evidence video covering the injected
  obstacle, blue/yellow/purple traversal, approval, and terminal safe stop.
- Added the atomic `move_relative` controller with bounded signed distance,
  odometry origin capture, speed clamping, obstacle stopping, and mandatory
  terminal `safe_stop`.
- Added the versioned `input-event.v1` shortcut boundary, validated workflow
  catalog, exact source/control bindings, replay and sequence rejection,
  heartbeat dead-man timeout, release/disconnect cancellation, and audit
  events.
- Added the `shortcut.forward.30cm.v1` workflow-card example.
- Added a deterministic 30-run shortcut soak with 30/30 completions and six
  verified obstacle stops before recovery.
