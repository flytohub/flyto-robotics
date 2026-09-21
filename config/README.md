# Configuration

Configuration in this directory supports the external ROS 2 adapter, Gazebo
labs, and deterministic controller tests.

Production robots are not configured as Flyto2 appliances. Robot-side runtime
configuration stays in standard ROS 2 / Nav2 / driver files. Files here that use
`/flyto/*` names belong to simulation, compatibility, or external adapter
test surfaces unless a document explicitly says otherwise.

Do not put credentials, Cloud endpoints, execution-host identity, or robot-side
Flyto2 scheduler state in these files.
