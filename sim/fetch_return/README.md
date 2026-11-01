# Two separate devices: fetch a red block and return (SIMULATION)

## The actual topology

```
HOME: user + TurtleBot3                      STATION: fixed AM-ARM200 + red block
  [mobile TurtleBot3]  ─── navigate ───>      [fixed mechanical arm + table]
  [empty rectangular cargo tray]               |  camera finds red block
                 ║                              |  arm picks red block
          robot docks and stops  <──────────────┘  arm puts it INTO robot tray
                 ║                    [verify placement while robot is parked]
      verify stationary arm has retracted
                 ║
  <── TurtleBot3 drives home with red block ──
                 ║
        MOBILE cargo sensor confirms block is still aboard
```

The AM-ARM200 **never rides on the TurtleBot3**. The arm is bolted to the
stationary workbench at the destination; the robot starts beside the user.
No arm actuation is allowed until the robot is docked and still. No return
is authorized until the stationary arm retracts and the handoff is verified.
The mobile robot owns its load from the handoff onward; the arm's stationary
camera or bin coordinates cannot prove the returning vehicle still has it.

## Reproduce on the Mac

Use the standalone isolated local checkout of flyto-robotics (no real robots).
The command interprets the whole Chinese goal; all ten-plus stages share a
single task id and a hash-linked evidence chain.

```
ARM_CONFIG=/Users/chester/flytohub/flyto-modules-lerobot/examples/am-arm200-mujoco.json
PY=/Users/chester/flytohub/flyto-modules-lerobot/.venv/bin/python
$PY sim/fetch_return/run.py '派機器人去把紅色積木拿回來' \
  --arm-config "$ARM_CONFIG" --out /tmp/flyto-fixed-arm-handoff.json
$PY -m pytest -q tests/test_fetch_return_sim.py
```

For the higher-fidelity Gazebo/Nav2 twin (ROS2 in Docker), bring up the
`sim/twin/compose.yaml` service from the flyto-robotics checkout; then run
with `--navigation gazebo --station-distance-m 0.4`. The twin is ROS2
simulation only and the navigation safety floor must NOT be disabled to pass.

## What these results DO and DO NOT prove

- **Implemented/tested:** deterministic simulated mobile navigation, docking
  gate, the existing MuJoCo stationary arm's camera/IK/contact-based physical
  grasp, MuJoCo drop into the docked robot's cargo-bin proxy, transfer gate,
  arm retraction, return routing and separate mobile cargo presence simulation.
- **Separate independent resources:** `turtlebot3-mobile-sim` and
  `am-arm200-fixed-station-sim` with one Task ID.
- **Not physically coupled:** the MuJoCo arm/bin proxy and Gazebo moving
  base are independent simulation worlds. Cargo is not simulated under
  acceleration during transit. Mobile cargo sensor is a simulator, not a
  measured real sensor. The prediction model is constant velocity from
  sourced visual observations, not a trained World Model.
- **Not integrated with Cloud task acceptance:** this is a runnable robotics
  co-simulation harness, not a verified `flyto-cloud` multi-resource task.
  The Cloud planner, space resource approvals and independent verdict need
  their own multi-device integration test before that claim is made.
- **No hardware authorization:** does not send commands to the real ROS2
  robot, to the physical AM-ARM200, or to production Cloud.

## Positioning the physical station (measure before fabricating)

- Put AM-ARM200 **on a rigid stationary rectangular tabletop**, not on the
  TurtleBot3. The table needs an anchored base and space for the red block.
- Put a **small, lightweight rectangular cargo tray on the TurtleBot3**.
  It needs low side walls so the block cannot roll off; verify chassis
  payload capacity, center of gravity, LiDAR/camera clearance and fastening.
- Mark a docking pose beside the table with the tray within the arm's
  verified reach and with clearance between gripper, tabletop, tray and robot.
  Teach/measure the arm-to-tray transform only after the robot docks.
- For the first physical demonstration, keep speed low, provide a reachable
  12 V arm cut-off, and do not let the robot leave before retraction + load
  verification. Exact dimensions depend on the real hardware measurements,
  so none are asserted by this simulated fixture.

## Cargo bridge for the live twin (SIMULATION)

The Gazebo/Nav2 twin has no tray sensor. `cargo_bridge.py` holds the
**symbolic** cargo state of the simulated tray and publishes it to ROS 2 so
the robot's adapter reads `sensing.cargo_present` from a topic, exactly as it
would from a real tray sensor. It is simulation state, not a measurement.

| What | Value |
| --- | --- |
| Topic | `$FLYTO_ROS2_CARGO_TOPIC`, default `/flyto/cargo_present` |
| Type | `std_msgs/msg/Bool` (`{"data": true}` = block in tray) |
| QoS | advertised with rosbridge `latch: true` (transient local), re-published at `--rate-hz` (default 2 Hz) |
| Robot pose | `/odom` (`$FLYTO_ROS2_ODOM_TOPIC`) over the same rosbridge, odometry frame |
| Arm signal | `GET http://127.0.0.1:$E2E_SIM_CONTROL_PORT/sim/truth` (default 7102), rising edge of `inside` |
| Control | `http://127.0.0.1:$FLYTO_SIM_CARGO_BRIDGE_PORT` (default 7110), loopback only |

Rules the bridge enforces (`cargo_tray.py`):

- The tray starts empty.
- A place loads the tray only while the robot is **parked at the station**:
  a fresh (<= 1 s) odometry pose within `--station-tolerance-m` (default
  0.10 m) of `--station-x/--station-y`, and speed <= 0.01 m/s, <= 0.02 rad/s.
  A missing, stale or malformed pose is unknown, and an unknown pose refuses
  the place (`robot_pose_unknown`); away from the station it is
  `robot_not_at_station`; still moving it is `robot_not_parked`.
- Once loaded, the cargo stays with the robot wherever it drives; the arm
  world being reset afterwards does not clear it. Only an explicit unload does.
- Every place and unload is recorded with its reason in `GET /sim/cargo`.

### Why the arm signal is the e2e host's `/sim/truth`

flyto-cloud's `scripts/e2e_closed_loop/e2e_host.py` already serves
`GET /sim/truth` on its loopback simulation control port. Its `inside` field
is MuJoCo ground truth that the block rests in `bin-blue`, the arm world's
proxy for the docked robot's tray. Polling it needs **no change to Cloud**
and no change to the arm pack. Only a False -> True transition counts as a
place: a block already inside when the bridge starts is never reported,
because nothing says the robot was parked when it got there. An unreachable
host or a reading without a boolean `inside` is no signal and keeps the last
value. Note that each `/sim/truth` read integrates the MuJoCo world to now,
as any reading of the arm does in that harness.

For glue without a truth port (for example `run.py`'s in-process arm), start
the bridge with `--arm-source http-only` and report the place yourself:
`POST /sim/cargo/place`; clear it with `POST /sim/cargo/unload`.

### Run

```
PY=/Users/chester/Projects/flytohub/flyto-robotics/.venv/bin/python
# 1. The twin is up on ws://127.0.0.1:19090 (sim/twin/compose.yaml).
# 2. Read where the station is in the odometry frame, e.g. the parked pose:
#      docker compose -f sim/twin/compose.yaml exec twin \
#        bash -lc 'ros2 topic echo --once /odom --field pose.pose.position'
# 3. Start the bridge (the e2e host serving /sim/truth on 7102):
E2E_SIM_CONTROL_PORT=7102 $PY sim/fetch_return/cargo_bridge.py \
  --station-x 2.0 --station-y 0.0 --station-tolerance-m 0.10
#    or without an arm host:
$PY sim/fetch_return/cargo_bridge.py --station-x 2.0 --station-y 0.0 --arm-source http-only
# 4. Inspect / drive it:
curl -s http://127.0.0.1:7110/sim/cargo
curl -s -X POST http://127.0.0.1:7110/sim/cargo/place
curl -s -X POST http://127.0.0.1:7110/sim/cargo/unload
#    and on the ROS side:
#      ros2 topic echo /flyto/cargo_present std_msgs/msg/Bool \
#        --qos-durability transient_local
# Tests (no Docker; fake rosbridge):
$PY -m pytest -q tests/test_cargo_bridge_sim.py
```

The bridge never commands motion and never touches the 0.35 m clearance
floor or any other safety limit.
