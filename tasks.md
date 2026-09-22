# Tasks

## Current zero-runtime architecture

- [x] Remove Flyto2 runtime, credentials and Flyto-specific services from the
      physical TurtleBot3.
- [x] Prove upstream TurtleBot3 bringup, LiDAR, odometry, TF and camera without
      Flyto2 code on the Pi.
- [x] Fix cold-boot DDS ordering by waiting for a usable network route before
      native ROS startup.
- [x] Add upstream-only native ROS systemd examples under `deploy/native_ros2/`.
- [x] Stop shipping Pi job-runner/lifecycle/recovery console entry points.
- [x] Stop shipping the legacy lifecycle registry and `deploy/` package in the
      production wheel.
- [x] Reframe README / Architecture / Installation around an external AI Space
      computer and standard ROS 2 robot.
- [ ] Close silent OpenCR/odom/TF liveness loss without adding a Flyto-specific
      robot daemon.
- [ ] Make Nav2/SLAM lifecycle fully active with truthful localization/mapping.
- [ ] Run the external Generic ROS 2 Adapter against the real robot graph.
- [ ] After physical clearance is confirmed, run bounded
      move -> interrupt -> safe stop -> recover evidence acceptance.
- [ ] Verify Cloud objective completion only from bound independent evidence.
- [ ] Remove historical appliance source after migration dependencies are gone.

## Verification

Before landing source changes:

```bash
make verify
flyto-index task validate --test-path tests
flyto-index impact <changed symbol>
flyto-index verify . --strict
```

No source-only check may be reported as physical acceptance.
