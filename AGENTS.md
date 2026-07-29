# flyto-robotics Agent Rules

- Keep this repository independent from Flyto Cloud, Core, Code, and Engine
  source trees. Integrate through versioned JSON contracts or network APIs.
- Simulation and real hardware must consume the same mission contract.
- Keep the control loop deterministic and fail safe: stale odometry, invalid
  jobs, missing sensors, and nearby obstacles must stop the robot.
- Do not store credentials, tokens, patient data, or production endpoints in
  source, examples, logs, or generated evidence.
- Treat all example payloads as synthetic.
- Keep Gazebo assets self-contained. Do not require online model downloads at
  runtime.
- Before changing code, use flyto-indexer search and structure to explore the
  target, then run flyto-indexer task planning and impact analysis.
- Run `make verify` after behavior, contract, model, world, or launch changes.
- After changing code, run flyto-indexer task validation, unstaged impact
  analysis, and full strict verification before committing.
- Generated ROS, Gazebo, test, and evidence output must remain untracked.
- Use `git -C <repo> ...`; never use `cd <repo> && git ...`.
