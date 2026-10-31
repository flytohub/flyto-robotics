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
