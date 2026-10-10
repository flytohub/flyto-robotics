# Changelog

All notable project changes are recorded here.

## 0.7.0 — 2026-10-07

- Added independent ROS2 readiness mobile provider with strict schema,
  freshness, SHA-256 observation evidence and heartbeat refresh; no
  motion permission or external Cloud dependency introduced.


- Breaking wire change, lab planner only. The plan contract is now `flyto.capability-plan.v1` (was `flyto.robotics.plan.v1`) and names the commanded equipment `resource_id` (was `robot_id`); the planner request is `flyto.robotics.planner-request.v2` and carries `resource_id`. This matches flyto-ai's planner, which emits that contract and refuses a v1 request by name, so a 0.6.6 lab tool paired with a current planner fails with a reason instead of a mismatch. `contracts/plan-v1.schema.json` is now `contracts/capability-plan-v1.schema.json`; the example plans, `goal_planner` and the MCP benchmark use the new shape. Python callers pass `planner_request(resource_id=...)` / `request_ai_plan(resource_id=...)` and read `RobotPlan.resource_id`.
- Jobs, results, planning sessions and the MCP `robot.plan.prepare` tool keep `robot_id`: those are the executor's own contracts, not the planner's. A plan's `resource_id` must equal the job's `robot_id`, as before.
- No released Desktop is affected: flyto-cloud loads this package for the adapter provider and never imports the planner client, so no compatibility window is kept. `tests/fixtures/capability-plan-exchange.v1.json` is a byte-identical copy of flyto-ai's and both repos pin its digest.

## 0.6.6 — 2026-10-07

- A motion cut short by what stands ahead no longer reports `completed`. Twin, task t-41169022a09d95b6 (0.6.5): `motion.advance` asked for 1.2 m with a box ahead; speed governance measured 0.0901 m of room to the floor at send and shortened the goal to 0.0211 m, the braking guard also tripped on clearance, Nav2 DriveOnHeading succeeded, and `motion_outcome` said `completed` / `succeeded` with 0.0345 of 1.2 m travelled. Cloud saw a completed step, planned no way round, and the task failed as not proven. A run that started a little further back drove, was tripped mid-drive with a `canceled` status, reported `obstacle_blocked` and was routed round: the verdict depended on where the robot stood. 0.6.4's planned-stop rule only fired when the guard had not tripped, and the action status was checked before the guard's cause.
- One table, `motion_outcome.STOP_RULES`, decides from the motion's facts (`StopFacts`: requested, commanded and travelled distance, `plan.shortened`, the guard's trip, whether the nearest return the way the robot was going is inside the floor), never from the action status, for any straight action (DriveOnHeading, BackUp) and whether the stop came from the before-send plan or a mid-motion trip. First match wins: `no_room_to_drive` (commanded below `braking_envelope.MIN_DISTANCE_M`, 0.01 m) -> `path_blocked`; `reached_request` (a planned or guarded stop within `mission.relative_move_tolerance`, min(0.03 m, d/10), of the request) -> `completed`; `guard_blind` -> `sensor_stale`; `stopped_short_by_clearance` (short by more than the tolerance and shortened or tripped on clearance) and `stopped_short_facing_obstacle` (short, and the nearest return ahead inside the floor) -> `obstacle_blocked`. Anything no row matches keeps the existing account (status, Nav2 error code, collision monitor). The result adds `motion_outcome.stop_rule` (name, reason, basis, requested/commanded/travelled, shortfall, tolerance), additively.
- A goal its server reported as succeeded takes its call outcome from that reason: anything but `completed` is `failed`, with the reason leading the detail. A drive refused before moving because no distance is left after stopping now reports `evidence.reason_code: path_blocked` (was `obstacle_blocked`): it was refused, not stopped on the way. Both codes ask flyto-cloud for a way round. The floor, the plan and the guard are unchanged.

## 0.6.5 — 2026-10-06

- Read-only; no motion behaviour changes. Every motion result says how fast a simulator's clock ran during it. Twin, 2026-10-06 22:01 local: a detour took 68 s where the same detour usually takes ~33 s. Every phase was stretched 1.8-2.4x in wall time, including Nav2's BackUp (9.1 s against ~5 s for the same 0.20 m), with no stall and no recovery. The simulator had fallen behind real time, which the result could not show.
- `motion_outcome.sim_time` (additive): in a simulation deployment (the adapter's own deployment, the configured mode confirmed against the graph as `served_identity` reports it, never a resource name), the rosbridge transport samples the simulator clock (`FLYTO_ROS2_SIM_MARKER_TOPIC`, `/clock`, read only, at most every 100 ms). The result reports `rtf_mean` (simulated seconds per wall second over the motion), `rtf_min` (over windows of at least 1 s), `wall_s`, `sim_s`, `samples` and `clock_reset_seen`, plus `sim_slow` when `rtf_min` is below `sim_time.SIM_SLOW_RTF` (0.5; `FLYTO_ROS2_SIM_SLOW_RTF` overrides it with a factor in (0, 1]), with `sim_slow_threshold` and its `basis`. A real deployment reports `{"applicable": false, "reason": "real deployment: no simulator clock"}` and reads nothing new. A simulated motion on the rclpy transport reports that it does not read the clock.
- `navigation_escape.legs[].sim_time` gives each leg's record. `navigation_escape.sim_time` covers the whole call: the legs' clocks summed, and the slowest window of any leg.

## 0.6.4 — 2026-10-06

- A straight drive's speed is governed the way the server running it can take, and on Nav2 no goal is ever sent mid-drive. The braking guard slowed a drive by sending a slower goal that preempted the running one, but Nav2's straight-drive behaviors do not support preemption in any release: the twin's behavior server logged "Received a preemption request for drive_on_heading, however feature is currently not implemented. Aborting and stopping" on every slowdown (2026-10-06 19:32, 20:13, 20:20, 20:26 local), stopped the robot and restarted the drive from rest, and the guard tripped the restarted drive within a scan (20:13: a 1.2 m advance ended after 0.052 m, `obstacle_blocked`), which set off the detour. It always stopped with the floor held, but the slowdown it was designed to do never happened.
- One decision, `braking_envelope.decide_governance`, from one table, `PREEMPTION_BY_IMPLEMENTATION`, keyed by what the running server says implements the drive: the behavior plugin class it loaded (read once per action from `<behavior>.plugin` on `FLYTO_ROS2_BEHAVIOR_SERVER`, default `/behavior_server`; the twin reports `nav2_behaviors::DriveOnHeading`), else the action type it serves (`nav2_msgs/action/DriveOnHeading`, `BackUp`). Those are not preemptible; anything undeclared is treated as not preemptible. The transport also counts (rosbridge can send a second goal, rclpy cannot). Nav2 has no parameter that says whether a behavior preempts, so what implements the drive decides; no ROS distro name is read.
- `planned_before_send` (Nav2, rclpy, undeclared types): speed and distance are both fixed before sending from the room to the floor measured along the swept path then (`braking_envelope.plan_drive`): `v = min(requested, 0.8 * a(-t + sqrt(t^2 + 2 room / a)))`, `distance = min(requested, room - (v t + v^2 / 2a))`, so the drive ends where the whole stopping distance still leaves the robot at or beyond the 0.35 m floor. The guard is a hard safety stop only (zero, cancel, zero when the room reaches the stopping distance, or the LiDAR is unreadable or quiet). A drive that ends at its planned stop point, short of the request, is `failed` with `motion_outcome.reason: obstacle_blocked` as a guard stop is, so Cloud still plans the detour. A drive left with less than 0.01 m to drive is refused before it moves, as one that cannot be given 0.02 m/s already was. `preemptive_resend` (a server declared preemptible) keeps the slowdown path unchanged. The floor is unchanged.
- `motion_outcome.braking` adds `governance` (`mode`, `action_type`, `preemption_supported`, `source`), `plan` (requested and commanded speed and distance, room at send, `shortened`) and `ended: planned_stop`; `motion_outcome` adds `commanded_distance_m` and keeps `requested_distance_m` as asked; the operator line names the planned stop and the governance mode. All additive.

## 0.6.3 — 2026-10-06

- Instrumentation only; no motion behaviour changes. A goal that stands still now says what was commanded while it did. Twin, 2026-10-06, t-f044dad913d21612 (0.6.2): the escape's goal leg was sent 0.01 s after the waypoint leg succeeded, then stood still for ~100 s while Nav2's controller ran (8 "Failed to make progress", recoveries clear/spin/wait/backup, no controller "no valid trajectory" error, no collision-monitor action) and moved 16 s after the backup recovery. Its start pose and heading matched a 20 s run of the same release. The logs could not tell whether the controller commanded zero or commanded motion that never reached the base, so the cause is not identified and nothing was changed on its account.
- `motion_outcome.stalls` (additive): every spell of at least `STALL_MIN_S` (2 s) in which odometry stays below 0.01 m/s and 0.05 rad/s while a goal runs, with `at_s`, `duration_s`, the pose, and per watched topic the number of commands and their largest linear and angular magnitude. `motion_outcome.recoveries_at_s` (additive): when Nav2's `number_of_recoveries` rose. The rosbridge transport reads, never writes, `FLYTO_ROS2_COMMAND_WATCH_TOPICS` (default `/cmd_vel_nav,/cmd_vel`: the controller's output and what reaches the base), sampled at 10 Hz, typed by the graph so stamped and unstamped commands both work; empty turns it off. Not wired on the rclpy transport.
- `navigation_escape.legs[]` adds `started_at_s` and `elapsed_s` per leg (from the call's start), so a gap the adapter adds between legs is visible in the result instead of only in Nav2's logs.

## 0.6.2 — 2026-10-06

- The inflation escape's lateral waypoint is sent at a heading from which every arrival Nav2 accepts points past the obstacle. It was sent facing the final goal, a line that grazes the obstacle's near corner, and Nav2 ends a leg anywhere within the goal checker's `yaw_goal_tolerance` of the heading it was sent; ending at the far edge of that band left the robot facing the obstacle and the stock DWB controller could not leave (twin, 2026-10-06, task t-c3e495596129a8d4: waypoint sent at -0.278 rad, ended at -0.523 rad, then eight "Failed to make progress", nine recoveries (clear, spin, wait, backup), 119 s goal leg; the 16 other escapes that day whose goal leg started within 0.07 rad of the line of travel took 9-19 s with no recovery). The heading is now the goal's bearing turned toward the open side until heading +/- tolerance lies on the open side of a bound computed from the LiDAR returns (tangent at robot radius + margin round each return of the obstacle, and never back across the start's line of travel, because the obstacle's side may run on behind its visible face). The tolerance is the largest `yaw_goal_tolerance` of the controller's goal checkers (Nav2's 0.25 rad when unreadable; a quarter turn or more is not believed). Nav2 parameters are unchanged; the twin and the robot both run TurtleBot3's `burger.yaml`.
- `navigation_escape` evidence adds `heading_tolerance_rad`, `clear_heading_rad` and `waypoint_heading` (`heading_rad`, `toward_goal_rad`, `clear_heading_rad`, `tolerance_rad`, `limited_by: goal|obstacle`, in the start's robot frame), additively. The 0.35 m floor, the back-off, the lateral offset and the goal leg (the original target, unchanged) are unchanged.

## 0.6.1 — 2026-10-06

- An inflation escape no longer narrows the obstacle at a LiDAR bin it could not read. The obstacle cluster ended at the first unread bin, so one dropout in a box's face cut its extent on that side; the lateral waypoint was then placed beside only part of the box, the goal leg started inside its inflation and Nav2 failed to make progress until it aborted. Unread bins inside the object or just past its last return now count as the object (at the last return's range, within the `CLUSTER_JUMP_M` arc that joins two returns), so the extent only grows for what the scan did not see. Twin, 2026-10-06, ten runs of the same box: 8 waypoints at 0.39-0.41 m arrived in 15-18 s; one at 0.285 m (extent 0.085 m) stalled 165 s in Nav2 recoveries; one at ~0.22 m (extent ~0.02 m) failed after 180 s (`error 105`). Replaying the recorded sweep through the 0.6.0 code with one face bin unread gives 0.226 m and 0.284 m; this release gives 0.41 m for both.
- The lateral waypoint lies at least Nav2's arrival tolerance plus the margin from where the back-off ends. The tolerance is the largest `xy_goal_tolerance` of the controller's goal checkers (`goal_checker_plugins` on `FLYTO_ROS2_CONTROLLER_NODES`, default `/controller_server`; Nav2's 0.25 m when unreadable). A waypoint inside it is "reached" where the robot stands: on the twin a 0.22 m waypoint against 0.25 m succeeded in 20 ms without moving. `navigation_escape` evidence adds `arrival_tolerance_m` and `obstacle.unread_bins`, additively. The 0.35 m floor and every clearance check are unchanged.

## 0.6.0 — 2026-10-06

- The SSH local forward to a loopback-bound robot is owned by the adapter (`flyto_robotics/ssh_transport.py`), configured by `FLYTO_ROS2_SSH_HOST` (implies rosbridge), `FLYTO_ROS2_SSH_IDENTITY`, `FLYTO_ROS2_SSH_KNOWN_HOSTS`, `FLYTO_ROS2_SSH_FORWARDS` (default `rosbridge=9090`) and `FLYTO_ROS2_SSH_LOCAL_PORTS` (else ephemeral). `FLYTO_ROSBRIDGE_URL` is derived from the forward; a contradicting explicit URL is refused. Until now the forward was started by hand and died with every robot reboot.
- Security: key-only (`BatchMode`, password/keyboard-interactive off), `StrictHostKeyChecking=yes` (an unknown or changed host key is `failed` with how to trust it once), loopback-only forwards on both ends (anything else refused), `-N` (no remote command), host validated and placed after `--`.
- Lifecycle: supervised ssh (`ExitOnForwardFailure`, `ServerAliveInterval`, private `ControlMaster`), re-resolved every attempt, exponential backoff 1 s to 30 s with jitter; host-key and auth failures wait for an operator. One forward per configuration, held by every adapter on it, stopped after `FLYTO_ROS2_SSH_LINGER_S` (30) without holders and at process exit.
- While the forward is down every capability call fails fast: `refused`, `evidence.reason_code: "transport_unavailable"`, `evidence.transport`; `safe_stop`/`cancel` return `failed` with the same evidence. A dropped rosbridge session is reopened on the next call once the forward is up.
- `transport_status()` on the adapter, the provider (`adapter_provider.transport_status`, `discover_resource_manifests.transport_status`), the process protocol (`transport_status`) and an opt-in manifest extension `transport`: `{state: connected|reconnecting|failed|stopped, since, attempts, last_error, ...}`. The presence watch notifies `transport_<state>` and retries the robot at once when the forward returns.
- Operator refresh: `GenericROS2Adapter.reconnect()` (process op `reconnect`, `adapter_provider.reconnect_resource`) safe stops actuating calls first and refuses (`safe_stop_unconfirmed`) when the stop is not confirmed, then rebuilds the forward and the rosbridge session and re-reads the graph and `served_identity()`, returning the new status. It returns a status instead of `None` and no longer raises; the old ROS-link-only reconnect is `reopen()` (the presence watch uses it).
- The motion preflight applies the 0.35 m floor along the path the motion sweeps (`flyto_robotics/path_clearance.py`), by the capability contract's new `motion_kind` (`advance`, `retreat`, `rotate`, `planned`): a straight drive's corridor in its direction of travel (`x > 0`, `|y| < floor + FLYTO_ROS2_ROBOT_RADIUS_M`), a rotation all around at `floor + radius` (0.45 m by default, stricter than before), a navigation left to Nav2, the inflation escape and the in-motion guard. Until now the nearest return in any direction refused every motion: live, a return 0.334 m behind refused a forward advance with 1.445 m of room ahead. A refusal carries `reason_code: "path_blocked"` and `path_clearance` with the limiting side and the nearest return per sector (`ahead`, `left`, `behind`, `right`). Without a sweep the old omnidirectional check applies.

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
- Every Generic ROS 2 motion result carries `evidence.motion_outcome` with a machine-readable `reason` (`obstacle_blocked`, `sensor_stale`, `timeout`, ...), start/final pose, distance travelled vs requested and the nearest LiDAR return at the stop; failed motions lead their detail with it.- Ship Generic ROS2/rosbridge, OpenRMF and vision-stream integrations as external adapter providers usable by the built-in AI Space host or optional process hosts such as Flyto2 Runtime, instead of Cloud-bundled transport implementations.
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
