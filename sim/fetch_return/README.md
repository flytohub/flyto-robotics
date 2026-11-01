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
**symbolic** tray (`mobile_cargo.MobileCargoSimulator`, the same object
`run.py` uses) and publishes it so the robot's adapter reads
`sensing.cargo_present` from a topic. It is simulation state, not a
measurement, and it can only reach a simulation.

| What | Value |
| --- | --- |
| Topic | `$FLYTO_ROS2_CARGO_TOPIC`, else `/flyto/cargo_present` (the adapter's simulation default) |
| Type | `std_msgs/msg/Bool` (`{"data": true}` = block in tray) |
| QoS | volatile, re-published at `--rate-hz` (2-10, default 2); never latched. The rate contract is written once, in `flyto_robotics/cargo_sensor.py` |
| Publisher | the adapter's `RosbridgeROS2Backend.publish_simulated_state`, presence mode (odometry only, never motion) |
| Gate | refuses unless `FLYTO_ROS2_DEPLOYMENT_MODE=simulation`, the marker check is on, and the graph carries `/clock`; the process refuses to start otherwise |
| Robot pose | `/odom` over the same connection, odometry frame |
| Reports | `--reports FILE`, `flyto.sim-arm-report.v1` lines pushed by the arm's and mission's host (`run.py --reports FILE`); without it the tray never loads |

The tray loads only through `accept_station_handoff`, when all of these hold
for one place: a completed place, seen inside, released and not held; a
settled run of the arm's own tray readings (>= 3 readings over 0.5 s of
simulated time, inside 2 mm, not held); a completed retract to `ready`; and
odometry within `--station-tolerance-m` of the station, moving less than
1 cm, with no gap over 1.5 s, **from the place report's `started_at` (taken
before the place was commanded) until the moment the handoff is decided**.

It empties on an `unload` report (delivered at home), a `mission_end` report
(and nothing reported before it loads again), a `loss` report, an arm
reading showing the block out or held again before departure,
`--max-carry-s` (default 600) of carrying, and when the bridge stops (it
publishes `false` last).

```
PY=<repo>/.venv/bin/python
# 1. The twin is up (sim/twin/compose.yaml); the station is the parked pose in
#    the odometry frame.
# 2. FLYTO_ROS2_DEPLOYMENT_MODE=simulation $PY sim/fetch_return/cargo_bridge.py \
#      --station-x 2.0 --station-y 0.0 --reports /tmp/fetch-reports.jsonl
# 3. The arm/mission host appends to the same file, e.g.
#      $PY sim/fetch_return/run.py --arm-config <config> --reports /tmp/fetch-reports.jsonl
# Tests (no Docker; fake rosbridge):
$PY -m pytest -q tests/test_sim_cargo_bridge.py
```

The bridge never commands motion and never touches the 0.35 m clearance
floor or any other safety limit.
