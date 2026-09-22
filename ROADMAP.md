# Roadmap

## Near term

1. **External Generic ROS 2 Adapter closure**
   - consume approved Flyto2 capability requests on an AI Space computer;
   - discover standard ROS 2 interfaces;
   - return bounded execution receipts and independent observations.

2. **Native TurtleBot3 readiness**
   - keep the Pi at zero Flyto2 runtime;
   - make cold-boot ROS discovery deterministic;
   - make silent OpenCR/odom/TF loss fail visibly or recover through a generic
     ROS/OS mechanism;
   - activate Nav2/SLAM without fabricated localization.

3. **Physical closed-loop acceptance**
   - bounded move;
   - observe;
   - interrupt/cancel;
   - safe stop;
   - recover/continue;
   - return odom/LiDAR/camera evidence;
   - Cloud independently verifies the objective.

4. **Retire appliance compatibility source**
   - remove old job-runner/lifecycle/recovery implementation after all
     historical tests/downstream consumers have migrated.

## Later

- MHS / vendor / VLA adapter compatibility through the same capability and
  evidence boundaries.
- Multi-robot/fleet adapters such as Open-RMF without changing Cloud authority.
- Stronger hardware-backed attestation where target hardware supports it.
- Long-duration physical soak and multi-host acceptance.

## Explicitly out of scope

- Joint/motor command APIs in Flyto2 Cloud.
- A proprietary robot OS layer.
- A second scheduler inside robotics.
- Treating model judgment or ROS action success as mission truth.
