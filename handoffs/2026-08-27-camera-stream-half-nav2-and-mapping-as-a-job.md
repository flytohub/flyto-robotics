# The camera's other half, Nav2 left disabled, and mapping as a dispatched job

Owner: claude
Branch: claude/camera-streams-and-nav2
Date: 2026-08-27

## What changed

**The gateway now serves both halves of the vision-gateway contract.**
`flyto_robotics/camera_observation.py` gains `CameraStreamCatalog`,
`validate_stream_url` and a second route, so
`GET /api/spaces/zone-camera/streams` answers
`flyto.vision.stream-catalog.v1` instead of 404. Four operator settings read in
`camera_sources.py`: `FLYTO_CAMERA_STREAM_URL`, `_PROTOCOL`, `_LABEL`,
`_TTL_SECONDS`.

**`deploy/systemd/flyto-nav2.service`** — the Nav2 stack, installed and
deliberately not enabled. `ConditionPathExists` points at
`/home/ubuntu/.flyto/maps/lab.yaml`, which does not exist.

**`deploy/systemd/flyto-slam.service`** and **`deploy/executors/`** — mapping as
a device executor on the non-`robotics.` dispatch path, module ids
`mapping.start` / `mapping.save` / `mapping.abort`. See
`deploy/executors/README.md`; it is not restated here.

**`deploy/make-map.sh`** — the same work as a shell script, for an office bench
where there is no flyto-cloud.

`deploy/robot-tunnel.sh` is **not** part of this. It landed in a6d99ab (PR #27)
and is already on main.

## Why

**The stream half existed on the wrong side of the boundary.** Both consumers
default to `http://127.0.0.1:9000` — this gateway's port —
`flyto-modules-vision` for the observation path and flyto-cloud's
`vision_stream_adapter` for the catalog. The gateway served one and 404'd the
other, and the gap had been worked around by putting `stream?topic=/camera/image_raw`
into flyto-cloud's `.env`. A ROS topic name in the platform's configuration is
the leak `ARCHITECTURE.md`'s Boundaries section forbids and that flyto-cloud has
already had to clean up once — `useSpaceVocabulary.js` records `ros2` having
"drifted into the wrong vocabulary" in `DecisionTimeline`. Serving the catalog
here puts the topic name back on the robot, where ROS is the subject rather than
a leak.

**Plaintext-off-loopback is refused on the platform side, not here.** flyto-cloud's
`services/space_tasks/streams.refuse_insecure_address` owns that rule and says
why: the gateway is the party that would be exposed, and a party cannot be
relied on to refuse its own misconfiguration. `validate_stream_url` checks only
that the value is a URL a browser could open. Two copies of a security rule
would drift, and the dangerous outcome is whichever is more permissive.

**Everything the robot serves binds to loopback, and the tunnel is the
transport rather than a workaround for one.** `validate_bind()` refuses any
address that is not literal IPv4 loopback, and flyto-cloud will not mint a
plaintext stream reference whose host is not loopback. So a camera served across
venue wi-fi would not merely be unsafe — **it would not appear in the operations
room at all.** Nobody arriving at a venue discovers that by reading the code.

**Nav2 was installed and had never been started.** `ros2_action_executor` is a
full `NavigateToPose` client that refuses any other interface type, so the whole
semantic-navigation path depended on an action server nothing was running, and
the LiDAR was reduced to the delivery gateway's obstacle-stop gate.

It ships **disabled** because the only map in this repo is
`maps/nav2_lab.pgm`: a 20×20-pixel synthetic 8 m square with a drawn-on
perimeter at 0.4 m per cell, present so the launch files load. AMCL against it
converges, the costmap looks plausible, and every goal is computed in a room
that does not exist. That fails silently, which is worse than not navigating.

**Mapping is commissioning, not a mission primitive.** Adding `start_slam` to
`workflow.PrimitiveKind` would put it in the vocabulary the AI planner composes
delivery plans from, and a plan that can emit "begin remapping the building"
halfway through carrying something would have to be *refused* rather than never
expressible. `flyto_job_runner` already routes any step not prefixed `robotics.`
to the installed device-executor registry, so the mission engine is untouched.
The driving in between stays on the approved `robotics.motion` capabilities with
their existing sensor gates; mapping does not need a second, unreviewed way to
move a robot.

## Verified

Camera, on the robot and from this workstation through the tunnel:

```
/api/spaces/zone-camera/observation  → 200  [{"kind":"zone.overview","zone":"robot-front",
                                             "source":{"provider":"ros_image",
                                             "source_id":"tb3-front-0"},"usable":true,
                                             "detail":"camera_frame_fresh"}]
/api/spaces/zone-camera/streams      → 200  {"configured":true,
                                             "contract_version":"flyto.vision.stream-catalog.v1",...}
/api/spaces/zone-camera/nope         → 404
```

`/camera/image_raw` publishes `rgb8`, `width 640`, `step 1920` (`accept_image`
admits rgb8/bgr8 only, and requires `step == width*3`). Rate measured between
10 and 28 Hz depending on load; the driver's YUYV→RGB8 conversion is the
bottleneck, not the device, which stays at 30 fps.

flyto-cloud's `vision_stream_adapter.describe()` reaches the robot **on its own
defaults**, no configuration, and returns `capability_id vision.stream`,
`resource_id robot-front`, `approval_status DISCOVERED` — the robot's catalog is
evidence, never its own approval.

The conformance kit, run against the live robot through the tunnel:
`[pass] stream — mjpeg at 500ms`. Two checks fail: `cancel`, which the vision
adapter's own docstring says is the point of running it, and `timeout`
("reported success for a call with no time to run"), which is a genuine finding
nobody had seen because the documented way to run the kit did not work until
today.

Nav2 comes up on this Pi 4: `amcl`, `map_server`,
`planner_server`, `controller_server`, `smoother_server`, `behavior_server`,
`bt_navigator`, both costmaps, `velocity_smoother`, and `/navigate_to_pose`
advertised. The LiDAR writes real obstacles into the local costmap — 1333
occupied and 1417 inflated cells in a 60×60 window at 5 cm.

Reboot: all three camera units and bringup return on their own. `flyto-nav2`
stays `inactive` with `Condition: start condition unmet`, and did so again on
2026-08-27 with `/home/ubuntu/.flyto/maps/` absent.

The mapping executor, driven by a **real `DeviceExecutorRegistry`** against the
installed manifest on the robot: `prepare` returns a handle and `execute`
returns a contract-valid result for all three module ids; `../../etc/passwd`,
fullwidth `ｌａｂ` and `café` are refused as `map_name_invalid`; `mapping.start`
refuses `battery_too_low` at a real 11.45 V. 66 unit tests, plus 14 in
`tests/test_deploy_units.py`.

The sudoers grant works (`systemctl start`/`stop flyto-slam.service` without a
password) and `visudo -c` parses the whole configuration.

## Not verified

**The motion loop has never run.** Nav2 has never been sent a goal. Nothing
here proves plan → move → arrive → evidence on this hardware. The stack coming
up is not the loop closing.

**There is no venue map.** `/home/ubuntu/.flyto/maps/` does not exist, so
`flyto-nav2.service` has never satisfied its own start condition and has never
localised against anything real.

**No mission consumes the camera's evidence.** It is produced, it is fetchable,
and nothing depends on it. `flyto-modules-vision` — whose `vision.observe` is
the designed consumer and whose defaults already point at this gateway — is not
installed in flyto-cloud's venv.

**Mapping has never been dispatched.** `grep -c device_executor
~/.flyto/device-events.jsonl` → 0. Two independent reasons: the executor's wire
protocol was wrong until today (it wrote a trailing newline, which the registry
rejects, and read `module_id` off the top level instead of from inside
`request`), and this branch has never been cut as a release —
`git cat-file -p 8c751d7:deploy/executors` reports the path does not exist, so
the deployed tree the job runner imports does not contain it.

**`flyto-slam.service` is `static`, deliberately.** It has no `[Install]`
section, so it can only ever be started by the executor. A robot that booted
into mapping mode would have thrown away the map it recorded yesterday.

**The camera is not calibrated.** `/camera/camera_info` publishes
`K = [0,0,0,0,0,0,0,0,0]` with an empty distortion model. `apriltag_ros` and
`image_proc` are installed and `apriltag_node` subscribes correctly, but
`/image_rect` never appears because rectification against a zero intrinsics
matrix is meaningless. `mission_gateway.py` already admits `apriltag` as a
calibration-marker source and requires `x`, `y` and `yaw` — a full pose — so
calibration is a prerequisite of that contract, not an optional refinement.

**The device-executor boundary's central property is not enforced below the
contract on this machine.** `/etc/sudoers.d/90-cloud-init-users` grants
`ubuntu ALL=(ALL) NOPASSWD:ALL` and `flyto-job-runner.service` runs as `ubuntu`,
so "a dispatched job must not become arbitrary host execution" is held up by the
contract's bounded JSON, fixed module-id set and timeout, and by nothing under
them. The narrow sudoers fragment this ships is correct and currently buys
nothing here. Left alone on purpose: narrowing a running robot's own
administrative access can lock out a machine with no console, `eth0` and `usb0`
are down, and wlan0 is the only route in.

**A workflow agent moved this checkout to `main` mid-session** and the branch
had to be recovered. Nothing was lost — the work was committed and pushed — but
if a tree looks wrong, check `git reflog` before concluding files were deleted.

## Follow-ups

- Charge the pack. Every supported path to a map refuses below 11.6 V and it has
  been between 11.03 and 11.45 V all day. The robot does not charge in place.
- Record a real venue map, then enable `flyto-nav2` and prove the motion loop.
  Both need a person driving the robot.
- Calibrate the camera (`ros-jazzy-camera-calibration` is installed;
  `v4l2_camera` already looks for `~/.ros/camera_info/`), then wire
  `apriltag_ros` to the `CalibrationMarker` contract that is already there.
- Install `flyto-modules-vision` in flyto-cloud's venv so a mission can require
  what the camera sees.
- Cut a release containing `deploy/executors/` so the job runner can actually
  reach the mapping executor.
- `docs/INSTALLATION.md` enumerates twelve `FLYTO_CAMERA_*` settings and none of
  the four stream ones, so an operator following it cannot configure the stream
  half at all.
