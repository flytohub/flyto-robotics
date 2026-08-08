# Running the demo

Gazebo, the reproducible container, the adversarial lab and the shortcut loop.

The supported reference environment is Ubuntu 24.04 with ROS 2 Jazzy and
Gazebo Harmonic:

```bash
sudo apt update
sudo apt install \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-sim \
  python3-colcon-common-extensions
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install
source install/setup.bash
ros2 launch flyto_robotics hospital_demo.launch.py
ros2 launch flyto_robotics atomic_ai_demo.launch.py
```

The launch file starts:

- `worlds/hospital-logistics.sdf`;
- the `flyto_rover` differential-drive model;
- `ros_gz_bridge` for `/clock`, command velocity, odometry, and lidar;
- `mission_controller`, using the bundled pharmacy-to-ward example.

Override the job or output file:

```bash
ros2 launch flyto_robotics hospital_demo.launch.py \
  job_file:=/absolute/path/job.json \
  result_file:=/absolute/path/result.json
```

For a server-only smoke, append `headless:=true`.

Run the semantic-location plan in the same Gazebo world:

```bash
ros2 launch flyto_robotics atomic_ai_demo.launch.py \
  plan_file:="$(ros2 pkg prefix flyto_robotics)/share/flyto_robotics/examples/plans/semantic-location-sequence.json" \
  semantic_map_file:="$(ros2 pkg prefix flyto_robotics)/share/flyto_robotics/examples/maps/atomic-color-route.json" \
  semantic_map_id:=gazebo.atomic-color-route.v1
```

#### Reproducible container

On macOS or another machine without ROS 2, build the reference environment:

```bash
docker build \
  -t flyto-robotics:jazzy-harmonic \
  -f docker/Dockerfile.jazzy .
```

The image contains ROS 2 Jazzy, Gazebo Harmonic's ROS integration, the bridge,
and ffmpeg for evidence-video encoding. Mount this repository, run
`colcon build`, and launch with `headless:=true`. The container supports both
ARM64 and AMD64 base images.

#### Adversarial lab and evidence

Run one strict lab or the default three-run cold-start matrix:

```bash
make gazebo-lab
make gazebo-matrix
make gazebo-video
make gazebo-shortcut
```

Each run injects a real dynamic obstacle, verifies a LiDAR stop, removes the
obstacle, publishes one valid signed approval, attempts eight nonce replays,
and requires an authorized resume and final safe stop. Gazebo's own world-pose
publisher independently proves the rover body moved at least 3.8 m; controller
odometry alone is not accepted.

`make gazebo-video` runs the same strict lab, continuously samples the Gazebo
overhead camera, and writes `gazebo-careflow.mp4`, `video-probe.json`, and an
MP4 SHA-256 file beside the normal JSON/Markdown/JUnit evidence. The output
timeline is calibrated to the measured simulated mission duration; repeated
presentation frames are used when the ROS sensor-data QoS drops camera frames,
without generating or interpolating new visual content. Raw frames and videos
remain ignored build evidence.

The complete plan, measured results, image inventory, and external evaluator
walkthrough are indexed in `docs/testing/README.md`.

#### Workflow-card shortcut closed loop

`make gazebo-shortcut` exercises the same boundary used by a Flyto2 AI Space
workflow card. It sends `press`, bounded `heartbeat`, `release`, and a second
`press` as versioned input events; it never sends velocity or motor fields. The
ROS adapter resolves `keyboard.main/ArrowUp` to the one validated workflow ID,
executes that immutable plan, and publishes deterministic velocity commands.

The evidence driver moves a real Gazebo obstacle into and out of the lidar
path. A run passes only when all of these assertions hold:

- the shortcut starts the reviewed workflow rather than a motor command;
- missing hold state or release cancels the first mission and publishes stop;
- lidar produces an obstacle stop followed by a path-clear recovery;
- the second start completes the workflow;
- the audit timeline, four labelled Gazebo captures, ground-truth displacement,
  and at least eight real camera frames are present.

The output directory contains `shortcut-result.json`, `report.json`,
`report.md`, labelled PNG captures, raw camera frames, an H.264 evidence video,
and SHA-256 files. The evaluator exits non-zero if any required artifact or
behavior is missing.

For a Cloud-connected ROS deployment, generate one short-lived local secret
outside the repositories and provide the same value to the Flyto2 local
backend and the Robotics process:

```bash
export FLYTO_ROBOTICS_INPUT_TOKEN="<at-least-32-random-bytes>"
export FLYTO_ROBOTICS_INPUT_URL="http://127.0.0.1:8765/v1/input-events"

ros2 run flyto_robotics shortcut_controller --ros-args \
  -p job_file:=/absolute/path/job.json \
  -p plan_file:=/absolute/path/validated-plan.json \
  -p result_file:=/absolute/path/shortcut-result.json
```

The gateway binds to literal loopback only and requires the bearer token for
health and input events. The Cloud browser talks only to its same-origin local
WebSocket; the backend keeps the secret off-wire and forwards the strict event
contract to Robotics. A press is shown as active only after Robotics confirms
that the exact reviewed workflow ID started. Unknown bindings, workflow
mismatches, control-thread acknowledgement timeouts, socket loss, stale
sensors, and dead-man expiry all fail closed.
