# Named places on the execution host

Owner: claude
Branch: claude/robot-places
Date: 2026-10-04

## What changed

- `flyto_robotics/places.py` (new): `PlacesStore`, schema `flyto.robot-places.v1`.
  One JSON file per robot per map: `FLYTO_ROS2_PLACES_FILE`, else
  `<data dir>/places/<resource id>/<FLYTO_ROS2_MAP_ID or "default">.json`,
  data dir `FLYTO_ROBOTICS_DATA_DIR`, else `%LOCALAPPDATA%/flyto-robotics`
  (Windows), else `$XDG_DATA_HOME/flyto-robotics` (`~/.local/share`). Entries
  `{name, frame: "map", x, y, yaw}`; names NFC, trimmed, 1..64 characters, no
  control/line-separator characters, unique per map ignoring case; at most 200
  places; atomic writes (`fsio.atomic_write`) under an `flock` lock file; any
  file not readable exactly as written raises `PlacesStoreError` and is never
  overwritten. Ids are sanitised (with a digest when changed) so a path never
  leaves the data dir.
- `generic_ros2_adapter.py`: `places.list` and `places.mark` are declared iff
  `motion.navigate` is on the graph (runtime name `host:places:flyto.robot-places.v1`).
  `places.list` returns `evidence.places`, `map_id` and a `places`
  `application/json` artifact. `places.mark` reads the same `map_pose` the
  arrival uses (waits for pose + map TF), refuses without it, and keeps its
  result by call id. `motion.navigate` takes `place` XOR `x`/`y` (yaw only with
  x/y); a place is resolved before any precondition or motion: unknown ->
  refused with `evidence.known_places`, unreadable file -> refused, both with no
  backend call. The Nav2 goal is the stored pose (heading included). Every
  navigate result carries `evidence.navigation_target`; a call by place carries
  `evidence.resolved_arguments` `{x, y, yaw_radians}`. A resumed call keeps its
  first resolution. `GenericROS2Adapter(places_store=...)` is optional.
- `adapter_contract.py`: metadata for `places.list` (read_only) and
  `places.mark` (controlled, no safe stop, not cancellable);
  `DeclaredArgument.max_length` (emitted in `to_dict` only when set, so
  declarations without text arguments keep their schema hash); JSON schema
  gives text `minLength`/`maxLength`.
- Version 0.3.0 (setup.py, package.xml, `__version__`); CHANGELOG (also
  absorbs the unreleased map_pose entry), README, DECISIONS, STATE.

## Why

Named places without Cloud storing a location list: the host that drives the
robot keeps them. The pack's arrival evidence reads `x`/`y` from the call's
arguments; a call by place has none, so the adapter (the one resolver, and the
one sending the Nav2 goal) reports the coordinates it resolved to. See
DECISIONS.md 2026-10-04 and flyto-modules-robotics DECISIONS.md 2026-10-04.
Note: navigate's declaration schema hash changes (x/y no longer individually
required, `place` added), so hosts that pin approval to the hash will see it as
a new revision.

## Verified

- `make verify PYTHON=<repo .venv>`: exit 0 — ruff 0.16.4 "All checks passed",
  pytest 1224 passed / 1 skipped, plus the asset and dry-run targets.
- `tests/test_places.py` (68 cases): round trip incl. Chinese names, case-
  insensitive uniqueness and re-mark, bad names, non-map/non-finite poses,
  17 corrupt-file shapes fail closed and stay byte-identical, oversize file,
  default/override paths and traversal, declaration only beside navigate,
  mark/list via the adapter, mark refused without map_pose, idempotent mark,
  unknown place refused with known names and zero backend calls, place/x/y
  exclusivity (6 cases) with zero backend calls, corrupt file refuses
  navigation, resolved goal equals what Nav2 is sent, resumed call keeps target.
- `flyto-index verify . --strict` (2.18.1): PASS, 20 pass / 0 warn / 0 fail.
- `flyto-index task validate`: pass (ruff pass, pytest pass) — run with the
  indexer's interpreter and the worktree on PYTHONPATH, because the shim
  replaces PYTHONPATH.
- `flyto-index pr-risk` on the uncommitted diff: "breaking" flags reviewed —
  `GenericROS2Adapter.__init__` only gained an optional keyword argument.

## Not verified

- No Gazebo twin and no physical robot: nothing here was run against a live
  ROS graph. `places.mark` against a real `map_pose` is untested live.
- Windows locking (`fcntl` absent): only the in-process lock applies there;
  not exercised.
- No host (Cloud, Desktop) judges a call by place against
  `resolved_arguments` yet; until one does, its arrival evidence is unprovable
  (fails closed).

## Follow-ups

- Hosts: judge a navigate call by place against the step's / adapter's
  `resolved_arguments` (flyto-modules-robotics 1.3.0 puts it in the step output).
- Rename/delete of a place is not offered; re-marking a name moves it.
- Map identity is configured (`FLYTO_ROS2_MAP_ID`); it is not derived from the
  robot's SLAM map, so a rebuilt map with the same id keeps stale places.
