# Delivery capability catalog

The loopback delivery gateway that already accepts authenticated
`POST /v1/plans` now serves authenticated `GET /v1/capabilities`. The response
is the existing `default_capability_registry().execution_catalog()` projection,
so `flyto.robotics.capability-catalog.v1`, capability ordering, argument
schemas and bounds, schema hashes, and the contract hash retain one authority.

The endpoint is read-only and returns `Cache-Control: no-store`. It contains no
timestamp, filesystem or URL path, host/device identity, credential, ROS
detail, or actuator command. Reading it does not create a delivery session,
invoke a runner, perform simulation, or move physical hardware. Authentication
failures remain the generic `401 unauthorized`, and unrelated paths remain
`404 not_found`.

## Pre-rework verification receipt

Governed route job `job_d10fd522e77144e99f3fe055` verified revision
`6cfeb457ddc371d2bd222d91a4c1cfd42c8ff327b01fe19043945634efaecf6a`.
The governed route verify passed, and Codex independently ran `make verify`
with exit 0: Ruff clean, 1038 passed, 4 skipped, with all asset, contract, and
dry-run gates completed. This is exact pre-rework evidence, not Codex audit
acceptance of the current rework revision.

The implementation worker's sandbox prohibited loopback and Unix socket
creation, so its direct verification retained that environment limitation.
No Gazebo run, physical robot run, deployment, publication, or hardware motion
was performed for this closure.

Real-server tests request the endpoint twice and compare both bodies exactly
with the registry projection while also checking authorization and cache
policy. The next bottom-up layer is to migrate `flyto-modules-robotics` to
consume this endpoint as its bounds/schema authority; that repository is not
changed by this closure.
