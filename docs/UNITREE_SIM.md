# Unitree in simulation: feasibility and integration options

Status: feasibility recorded 2026-10-17. No adapter exists yet.

## Why Unitree

The UCC acceptance target is a new device joining through an adapter and a
capability contract, with no change to Cloud, the App or Runtime. TurtleBot3
cannot test the semantics a legged or humanoid robot needs:

- states a call requires and leaves (stand before walk);
- separate parts that can move at once (legs and arms);
- a stop that needs time and distance and ends in a stance.

flyto-core 2.41.0 (flytohub/flyto-core#139, unmerged) adds `establishes` and
`occupies` to the capability contract for exactly these. Simulation comes
first; hardware is not needed until a simulated run passes.

## Measured on the development Mac (Apple M5, macOS 26.6)

`scripts/unitree_mujoco_check.py` uses MuJoCo 3.16.0 and torch 2.14.1 with the
G1 policy published in unitree_rl_gym. It runs headless.

| Phase | Command | Result |
| --- | --- | --- |
| Walk 5 s | vx 0.5 m/s | +2.224 m forward, -0.099 m lateral; base height 0.77 m |
| Stop 3 s | zero velocity | 0.224 m further while decelerating; base height 0.78 m, upright |

What this shows:

- Locomotion under a velocity command runs on this machine, with no GPU and
  no DDS.
- A humanoid stop is not instant: it travels about 0.2 m after the command.
  This is why a contract has to state what a stop leaves the resource in and
  how long it takes.

What it does not show:

- Any Flyto2 path. Nothing here is registered, dispatched or judged.
- Unitree's Go2 or the official unitree_mujoco SDK2 bridge. The policy used
  here is the G1 one shipped with unitree_rl_gym, and Isaac Lab
  (unitree_sim_isaaclab) needs an NVIDIA GPU this Mac does not have.

## Integration options (to decide before building)

1. **Speak standard ROS 2.** A bridge runs the simulation behind rosbridge:
   `cmd_vel` in; odometry, a ray-cast LaserScan and a rendered camera out.
   The existing `generic_ros2_adapter` then drives it unchanged, which is the
   strongest proof that no adapter or core change was needed. Cost: ROS 2 and
   rosbridge in Docker on the Mac. The adapter's Nav2 actions (navigate,
   drive on heading) must be served too, either by real Nav2 against the
   simulated topics or by a small action server.
2. **Use Core's external adapter interface.** Implement the same capability
   IDs the TurtleBot3 pack exposes (`motion.advance`, `motion.rotate`, the
   resource's safe stop) as an external adapter, as the Home Assistant adapter
   does (flytohub/flyto-core#138), with `establishes`/`occupies` declared.
   This is less infrastructure, but it adds a second Python driver beside the
   ROS 2 one, so it proves the contract rather than the adapter.

Either way the robot must not get its own capability vocabulary: it reuses the
existing motion contracts so one goal runs on either robot.

## Licences

unitree_rl_gym, unitree_mujoco and unitree_sdk2 are BSD-3-Clause, and
unitree_sim_isaaclab is Apache-2.0. Models, USD assets and pretrained weights
can carry their own terms. Nothing from them is vendored here. If any is ever
redistributed, keep its notices, and never use Unitree's name to endorse
Flyto2.
