"""SIMULATION cargo bridge: the twin's tray, loaded only by a settled handoff.

No Docker, no rosbridge server, no MuJoCo host: reports are written as the
arm's and the mission's hosts would push them, odometry is fed as wall-clock
samples, and the rosbridge connection is the adapter's own backend over a fake
socket. What is pinned:

* One tray (``mobile_cargo.MobileCargoSimulator``), shared with ``run.py``.
* It loads only on a completed, released, settled place with the arm
  retracted, while the robot was parked within the station tolerance from
  before the place started until the handoff was decided.
* It unloads on an explicit unload at home, on mission end, on a loss report,
  on a loss seen before departure, after the longest carry, and when the
  bridge stops: never true for the life of the process.
* The only publisher is the adapter backend's ``publish_simulated_state``,
  which refuses unless the configured deployment is a simulation and the
  graph carries the simulator marker; volatile, repeated at >= 2 Hz.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from test_cargo_sensor import FakeBridge

from flyto_robotics import cargo_sensor
from flyto_robotics.generic_ros2_adapter import (
    CLOCK_TYPE,
    RosbridgeROS2Backend,
    SimulationOnlyError,
)
from sim.fetch_return import arm_reports as reports
from sim.fetch_return import cargo_bridge
from sim.fetch_return.cargo_bridge import (
    DECIDE_AFTER_S,
    CargoBridge,
    CargoHandoff,
    Station,
)
from sim.fetch_return.mobile_cargo import MobileCargoSimulator

STATION = Station(0.4, 0.0, 0.10)
AT_STATION = {"x": 0.401, "y": 0.002, "yaw": 0.0}
HOME = {"x": 0.0, "y": 0.0, "yaw": 0.0}
BLOCK = [0.22, 0.10, 0.012]


def _place(at, *, started_at=None, **overrides):
    evidence = {
        "released": True,
        "object_held": False,
        "perception_after": {"object_in_container": True},
    }
    evidence.update(overrides.pop("evidence", {}))
    return reports.place_report(
        overrides.pop("outcome", "completed"),
        evidence,
        started_at=at - 0.5 if started_at is None else started_at,
        at=at,
    )


def _reading(at, sim_t, *, inside=True, held=False, block=BLOCK):
    return reports.tray_reading(inside=inside, held=held, block=block, sim_t=sim_t, at=at)


def _settled(start_at, start_sim=10.0, count=6, step=0.1):
    return [_reading(start_at + i * 0.05, start_sim + i * step) for i in range(count)]


def _feed(handoff, records):
    parsed = [reports.parse_report(r) for r in records]
    assert all(p is not None for p in parsed)
    handoff.observe_reports(parsed)


def _parked(handoff, start, end, pose=AT_STATION, every=0.5):
    t = start
    while t <= end + 1e-9:
        handoff.observe_pose(t, pose)
        t += every


def _full_handoff(handoff, *, t0=100.0):
    """Robot parked from t0; place 101.5..102; settle readings; retract at t0+3."""
    _parked(handoff, t0, t0 + 4.0)
    _feed(
        handoff,
        [_place(t0 + 2.0), *_settled(t0 + 2.1), reports.retract_report("completed", at=t0 + 3.0)],
    )
    handoff.evaluate(t0 + 4.0)


# --- one tray ------------------------------------------------------------------


def test_the_bridge_and_the_offline_mission_share_one_tray():
    from sim.fetch_return import run

    assert run.MobileCargoSimulator is MobileCargoSimulator
    assert isinstance(CargoHandoff(STATION).tray, MobileCargoSimulator)


def test_the_rejected_second_tray_and_rosbridge_client_are_gone():
    folder = Path(cargo_bridge.__file__).parent
    for name in ("cargo_tray.py", "rosbridge_link.py", "arm_place_signal.py"):
        assert not (folder / name).exists()


def test_the_bridge_holds_no_rosbridge_client_or_http_path_of_its_own():
    source = Path(cargo_bridge.__file__).read_text(encoding="utf-8")
    for forbidden in ("websocket", "urllib", "http.server", "/sim/truth", "requests"):
        assert forbidden not in source


# --- loading -------------------------------------------------------------------


def test_a_settled_released_place_with_the_arm_retracted_and_the_robot_parked_loads():
    handoff = CargoHandoff(STATION)
    _full_handoff(handoff)
    assert handoff.cargo_present is True
    assert handoff.events[-1]["accepted"] is True
    assert handoff.events[-1]["simulation"] is True
    assert handoff.events[-1]["simulation_only"] is True


def test_a_place_report_must_say_when_the_place_started():
    record = _place(102.0)
    assert record["started_at"] == pytest.approx(101.5)
    assert reports.parse_report({**record, "started_at": None}) is None
    assert reports.parse_report({**record, "started_at": 102.5}) is None  # after it ended
    with pytest.raises(ValueError):
        reports.append_report(Path("/nonexistent/never-written"), {**record, "started_at": None})


def test_a_robot_that_arrived_during_the_place_was_not_parked_before_it():
    # The place ran 101.5..102.4; the robot reached the station at 102.0.
    # Timed from the place's end, it looked parked; from its start it was not.
    handoff = CargoHandoff(STATION)
    handoff.observe_pose(101.0, {"x": 0.20, "y": 0.0})
    handoff.observe_pose(101.6, {"x": 0.32, "y": 0.0})
    _parked(handoff, 102.0, 105.0)
    _feed(
        handoff,
        [
            _place(102.4, started_at=101.5),
            *_settled(102.5),
            reports.retract_report("completed", at=103.0),
        ],
    )
    handoff.evaluate(105.0)
    assert handoff.cargo_present is False
    assert handoff.events[-1]["reason"] == "robot_not_docked"


def test_a_robot_that_left_before_the_handoff_was_decided_is_refused():
    # Every report arrives late, after the robot drove off at 104: the handoff
    # is decided at 105, so the robot was not parked until the handoff.
    handoff = CargoHandoff(STATION)
    _parked(handoff, 100.0, 103.5)
    handoff.observe_pose(104.0, {"x": 0.20, "y": 0.0})
    handoff.observe_pose(105.0, HOME)
    _feed(
        handoff,
        [_place(102.0), *_settled(102.1), reports.retract_report("completed", at=103.0)],
    )
    handoff.evaluate(105.0)
    assert handoff.cargo_present is False
    assert handoff.events[-1]["reason"] == "robot_not_docked"


def test_a_still_robot_outside_the_station_tolerance_is_refused():
    handoff = CargoHandoff(STATION)
    beside = {"x": 0.4 + 0.15, "y": 0.0, "yaw": 0.0}  # parked, still, 0.15 m off
    _parked(handoff, 100.0, 104.0, pose=beside)
    _feed(
        handoff,
        [_place(102.0), *_settled(102.1), reports.retract_report("completed", at=103.0)],
    )
    handoff.evaluate(104.0)
    assert handoff.cargo_present is False
    assert handoff.events[-1]["reason"] == "robot_not_docked"


def test_a_robot_creeping_inside_the_tolerance_is_not_parked():
    handoff = CargoHandoff(STATION)
    for i, t in enumerate(range(100, 105)):
        handoff.observe_pose(float(t), {"x": 0.37 + 0.008 * i, "y": 0.0})
    _feed(
        handoff,
        [_place(102.0), *_settled(102.1), reports.retract_report("completed", at=103.0)],
    )
    handoff.evaluate(104.0)
    assert handoff.cargo_present is False
    assert handoff.events[-1]["reason"] == "robot_not_docked"


def test_a_brief_inside_reading_never_loads_the_tray():
    handoff = CargoHandoff(STATION)
    _parked(handoff, 100.0, 110.0)
    _feed(
        handoff,
        [
            _place(102.0),
            _reading(102.1, 10.0),
            _reading(102.2, 10.1),
            _reading(102.3, 10.2, inside=False),
            reports.retract_report("completed", at=103.0),
        ],
    )
    handoff.evaluate(104.0)
    assert handoff.cargo_present is False
    assert handoff.events == []  # still waiting for it to settle
    handoff.evaluate(103.0 + DECIDE_AFTER_S + 1.0)
    assert handoff.cargo_present is False
    assert handoff.events[-1]["reason"] == "cargo_transfer_not_verified"


def test_a_block_still_held_by_the_jaws_never_loads():
    handoff = CargoHandoff(STATION)
    _parked(handoff, 100.0, 110.0)
    held = [_reading(102.1 + i * 0.05, 10.0 + i * 0.1, held=True) for i in range(8)]
    _feed(handoff, [_place(102.0), *held, reports.retract_report("completed", at=103.0)])
    handoff.evaluate(103.0 + DECIDE_AFTER_S + 1.0)
    assert handoff.cargo_present is False


def test_a_block_still_moving_is_not_settled():
    handoff = CargoHandoff(STATION)
    _parked(handoff, 100.0, 110.0)
    sliding = [
        _reading(102.1 + i * 0.05, 10.0 + i * 0.1, block=[0.22 + i * 0.002, 0.10, 0.012])
        for i in range(8)
    ]
    _feed(handoff, [_place(102.0), *sliding, reports.retract_report("completed", at=103.0)])
    handoff.evaluate(103.0 + DECIDE_AFTER_S + 1.0)
    assert handoff.cargo_present is False


@pytest.mark.parametrize(
    "place",
    [
        _place(102.0, outcome="failed"),
        _place(102.0, evidence={"released": False, "object_held": True}),
        _place(102.0, evidence={"perception_after": {"object_in_container": None}}),
    ],
)
def test_a_place_the_arm_did_not_complete_and_release_never_loads(place):
    handoff = CargoHandoff(STATION)
    _parked(handoff, 100.0, 105.0)
    _feed(handoff, [place, *_settled(102.1), reports.retract_report("completed", at=103.0)])
    handoff.evaluate(105.0)
    assert handoff.cargo_present is False
    assert handoff.events[-1]["reason"] == "cargo_transfer_not_verified"


def test_no_load_until_the_arm_reports_it_retracted():
    handoff = CargoHandoff(STATION)
    _parked(handoff, 100.0, 120.0)
    _feed(handoff, [_place(102.0), *_settled(102.1)])
    handoff.evaluate(120.0)
    assert handoff.cargo_present is False
    _feed(handoff, [reports.retract_report("failed", at=103.0)])
    handoff.evaluate(120.0)
    assert handoff.cargo_present is False
    assert handoff.events[-1]["reason"] == "stationary_arm_not_retracted"


def test_an_odometry_gap_during_the_handoff_refuses():
    handoff = CargoHandoff(STATION)
    _parked(handoff, 100.0, 102.0)
    handoff.observe_pose(102.5, None)  # pose unknown while the arm worked
    _parked(handoff, 103.0, 105.0)
    _feed(
        handoff,
        [_place(102.2), *_settled(102.3), reports.retract_report("completed", at=103.5)],
    )
    handoff.evaluate(105.0)
    assert handoff.cargo_present is False
    assert handoff.events[-1]["reason"] == "robot_not_docked"


def test_silent_odometry_during_the_handoff_refuses():
    # No message at all for 2 s while the arm worked: the robot is unaccounted for.
    handoff = CargoHandoff(STATION)
    _parked(handoff, 100.0, 101.5)
    _parked(handoff, 103.5, 105.0)
    _feed(
        handoff,
        [_place(102.2), *_settled(102.3), reports.retract_report("completed", at=103.0)],
    )
    handoff.evaluate(105.0)
    assert handoff.cargo_present is False
    assert handoff.events[-1]["reason"] == "robot_not_docked"


def test_a_place_older_than_the_odometry_history_refuses():
    handoff = CargoHandoff(STATION)
    _parked(handoff, 200.0, 205.0)
    _feed(
        handoff,
        [_place(102.0), *_settled(102.1), reports.retract_report("completed", at=103.0)],
    )
    handoff.evaluate(205.0)
    assert handoff.cargo_present is False
    assert handoff.events[-1]["reason"] == "robot_not_docked"


# --- unloading: never true for the life of the process ---------------------------


def test_an_unload_at_home_empties_the_tray():
    handoff = CargoHandoff(STATION)
    _full_handoff(handoff)
    handoff.observe_pose(106.0, HOME)
    _feed(handoff, [reports.unload_report("completed", at=106.0)])
    handoff.evaluate(106.0)
    assert handoff.cargo_present is False
    assert handoff.events[-1]["kind"] == "unloaded"


def test_mission_end_empties_the_tray_and_retires_its_place():
    handoff = CargoHandoff(STATION)
    _full_handoff(handoff)
    _feed(handoff, [reports.mission_end_report("failed", at=106.0)])
    handoff.evaluate(106.0)
    assert handoff.cargo_present is False
    assert handoff.events[-1]["kind"] == "mission_end"
    # A place of the ended mission never loads it again.
    _parked(handoff, 106.0, 110.0)
    handoff.evaluate(110.0)
    assert handoff.cargo_present is False


def test_a_mission_ending_before_its_handoff_never_loads():
    handoff = CargoHandoff(STATION)
    _parked(handoff, 100.0, 105.0)
    _feed(
        handoff,
        [
            _place(102.0),
            *_settled(102.1),
            reports.mission_end_report("failed", at=102.6),
            reports.retract_report("completed", at=103.0),
        ],
    )
    handoff.evaluate(105.0)
    assert handoff.cargo_present is False


def test_a_reported_loss_empties_the_tray():
    handoff = CargoHandoff(STATION)
    _full_handoff(handoff)
    _feed(handoff, [reports.loss_report("payload_lost_in_transit", at=105.0)])
    handoff.evaluate(105.0)
    assert handoff.cargo_present is False
    assert handoff.events[-1]["kind"] == "lost"


def test_the_longest_carry_empties_the_tray():
    handoff = CargoHandoff(STATION, max_carry_s=30.0)
    _full_handoff(handoff)
    handoff.evaluate(120.0)
    assert handoff.cargo_present is True
    handoff.evaluate(134.5)
    assert handoff.cargo_present is False
    assert handoff.events[-1]["kind"] == "carry_expired"


def test_a_block_lost_before_departure_clears_the_tray():
    handoff = CargoHandoff(STATION)
    _full_handoff(handoff)
    assert handoff.cargo_present is True
    _feed(handoff, [_reading(104.5, 12.0, inside=False)])
    handoff.observe_pose(104.5, AT_STATION)
    handoff.evaluate(104.6)
    assert handoff.cargo_present is False
    assert handoff.events[-1]["kind"] == "lost_before_departure"
    # The same place cannot load it again; only a new place can.
    _parked(handoff, 105.0, 110.0)
    _feed(handoff, [*_settled(105.0, start_sim=13.0)])
    handoff.evaluate(110.0)
    assert handoff.cargo_present is False
    _feed(
        handoff,
        [
            _place(111.0),
            *_settled(111.1, start_sim=20.0),
            reports.retract_report("completed", at=111.5),
        ],
    )
    _parked(handoff, 110.5, 113.0)
    handoff.evaluate(113.0)
    assert handoff.cargo_present is True


def test_the_block_regrasped_before_departure_clears_the_tray():
    handoff = CargoHandoff(STATION)
    _full_handoff(handoff)
    _feed(handoff, [_reading(104.5, 12.0, inside=True, held=True)])
    handoff.evaluate(104.6)
    assert handoff.cargo_present is False


def test_after_departure_the_arm_proxy_no_longer_speaks_for_the_tray():
    handoff = CargoHandoff(STATION)
    _full_handoff(handoff)
    handoff.observe_pose(104.5, {"x": 0.20, "y": 0.0})  # driving home
    handoff.evaluate(104.5)
    assert handoff.events[-1]["kind"] == "departed"
    _feed(handoff, [_reading(105.0, 13.0, inside=False)])  # the proxy bin is empty now
    handoff.observe_pose(105.0, HOME)
    handoff.evaluate(105.0)
    assert handoff.cargo_present is True


# --- report files -----------------------------------------------------------------


def test_reports_written_before_the_bridge_started_are_never_read(tmp_path):
    path = tmp_path / "reports.jsonl"
    for record in [_place(102.0), *_settled(102.1), reports.retract_report("completed", at=103.0)]:
        reports.append_report(path, record)
    tail = reports.ArmReportTail(path)
    assert tail.read_new() == []
    reports.append_report(path, reports.retract_report("completed", at=104.0))
    assert [r.kind for r in tail.read_new()] == ["retract"]


def test_malformed_reports_are_rejected_and_counted(tmp_path):
    path = tmp_path / "reports.jsonl"
    path.write_text("")
    tail = reports.ArmReportTail(path)
    bad = [
        {**_place(1.0), "simulation": False},
        {**_place(1.0), "schema": "other"},
        {**_place(1.0), "released": "yes"},
        {**_reading(1.0, 1.0), "block": [0.1, 0.2]},
        {**_reading(1.0, 1.0), "sim_t": float("nan")},
        {"schema": reports.SCHEMA, "simulation": True, "at": 1.0, "kind": "load"},
        {**reports.unload_report("completed", at=1.0), "outcome": 1},
    ]
    with path.open("a") as handle:
        for record in bad:
            handle.write(json.dumps(record) + "\n")
        handle.write("{not json\n")
        handle.write(json.dumps(_place(2.0))[:20])  # a line still being written
    assert tail.read_new() == []
    assert tail.rejected == len(bad) + 1
    with pytest.raises(ValueError):
        reports.append_report(path, bad[0])


# --- the bridge loop ----------------------------------------------------------------


class FakeLink:
    """Only what the bridge may use: a pose read, the simulated publish, reconnect."""

    def __init__(self, pose=AT_STATION):
        self.pose = pose
        self.published: list[tuple[str, bool]] = []
        self.fail: Exception | None = None
        self.reconnects = 0

    def held_observation(self):
        return {"pose": self.pose}

    def publish_simulated_state(self, topic, value):
        if self.fail is not None:
            raise self.fail
        self.published.append((topic, value))

    def reconnect(self):
        self.reconnects += 1


def test_without_a_report_source_the_bridge_publishes_an_empty_tray():
    link = FakeLink()
    clock = iter(float(t) for t in range(100, 200))
    bridge = CargoBridge(link=link, handoff=CargoHandoff(STATION), clock=lambda: next(clock))
    for _ in range(5):
        assert bridge.tick() is False
    assert link.published == [("/flyto/cargo_present", False)] * 5
    state = bridge.state()
    assert state["simulation"] is True
    assert state["deployment_mode"] == "simulation"


def test_the_bridge_publishes_the_tray_once_the_pushed_handoff_holds(tmp_path):
    path = tmp_path / "reports.jsonl"
    link = FakeLink()
    now = [100.0]
    bridge = CargoBridge(
        link=link,
        handoff=CargoHandoff(STATION),
        reports=reports.ArmReportTail(path),
        clock=lambda: now[0],
    )

    def tick_until(t):
        while now[0] < t:
            bridge.tick()
            now[0] += 0.5

    tick_until(102.0)
    for record in [_place(102.0), *_settled(102.1)]:
        reports.append_report(path, record)
    tick_until(103.0)
    assert link.published[-1] == ("/flyto/cargo_present", False)
    reports.append_report(path, reports.retract_report("completed", at=103.0))
    tick_until(104.0)
    assert link.published[-1] == ("/flyto/cargo_present", True)
    link.pose = HOME
    tick_until(106.0)
    assert link.published[-1] == ("/flyto/cargo_present", True)
    reports.append_report(path, reports.unload_report("completed", at=106.0))
    tick_until(107.0)
    assert link.published[-1] == ("/flyto/cargo_present", False)


def test_stopping_the_bridge_publishes_an_empty_tray_last():
    link = FakeLink()
    handoff = CargoHandoff(STATION)
    _full_handoff(handoff)
    bridge = CargoBridge(link=link, handoff=handoff, clock=lambda: 104.0)
    assert bridge.tick() is True
    bridge.stop()
    assert handoff.cargo_present is False
    assert link.published[-1] == ("/flyto/cargo_present", False)


def test_a_refused_publish_is_reported_and_never_retried_as_a_reconnect():
    link = FakeLink()
    link.fail = SimulationOnlyError("the graph has no /clock")
    bridge = CargoBridge(link=link, handoff=CargoHandoff(STATION), clock=lambda: 100.0)
    bridge.tick()
    assert link.reconnects == 0
    assert "SimulationOnlyError" in bridge.state()["publish_error"]


def test_a_failed_publish_reconnects_the_one_connection():
    link = FakeLink()
    link.fail = RuntimeError("rosbridge adapter disconnected")
    bridge = CargoBridge(link=link, handoff=CargoHandoff(STATION), clock=lambda: 100.0)
    bridge.tick()
    assert link.reconnects == 1
    assert "disconnected" in bridge.state()["publish_error"]


@pytest.mark.parametrize("rate", [0.5, 1.0, 1.9])
def test_the_bridge_never_publishes_slower_than_the_consumer_contract(rate):
    bridge = CargoBridge(link=FakeLink(), handoff=CargoHandoff(STATION))
    bridge.stop()  # so a run that skipped the check would return, not loop
    with pytest.raises(ValueError, match="Hz"):
        bridge.run(rate_hz=rate)


@pytest.fixture
def no_rosbridge(monkeypatch):
    """main() must never reach a real endpoint from a test (19090 is the twin's)."""
    monkeypatch.setenv("FLYTO_ROSBRIDGE_URL", "ws://127.0.0.1:1")

    def forbidden(*_args, **_kwargs):
        raise AssertionError("the bridge opened a rosbridge connection")

    monkeypatch.setattr(cargo_bridge, "RosbridgeROS2Backend", forbidden)


def test_the_bridge_refuses_to_start_unless_configured_as_a_simulation(
    monkeypatch, capsys, no_rosbridge
):
    monkeypatch.setenv("FLYTO_ROS2_DEPLOYMENT_MODE", "hardware")
    assert cargo_bridge.main(["--station-x", "0.4", "--station-y", "0.0"]) == 2
    assert "simulation" in capsys.readouterr().err
    monkeypatch.setenv("FLYTO_ROS2_DEPLOYMENT_MODE", "simulation")
    monkeypatch.setenv("FLYTO_ROS2_SIM_MARKER_TOPIC", "")
    assert cargo_bridge.main(["--station-x", "0.4", "--station-y", "0.0"]) == 2


def test_the_bridge_rate_option_is_bounded_by_the_contract(monkeypatch, no_rosbridge):
    monkeypatch.setenv("FLYTO_ROS2_DEPLOYMENT_MODE", "simulation")
    with pytest.raises(SystemExit):
        cargo_bridge.main(["--station-x", "0.4", "--station-y", "0.0", "--rate-hz", "1"])


# --- the one publisher, behind the simulation gate ------------------------------------

SIM_GRAPH = {"/odom": "nav_msgs/msg/Odometry", "/clock": CLOCK_TYPE}
HARDWARE_GRAPH = {"/odom": "nav_msgs/msg/Odometry"}


@pytest.fixture
def backend_on(monkeypatch):
    made: list[tuple[RosbridgeROS2Backend, FakeBridge]] = []

    def build(topics, mode="simulation"):
        monkeypatch.setenv("FLYTO_ROS2_DEPLOYMENT_MODE", mode)
        monkeypatch.setenv("FLYTO_ROS2_SIM_MARKER_TOPIC", "/clock")
        bridge = FakeBridge(topics)
        sent: list[dict] = []
        original = bridge.send

        def send(payload):
            sent.append(json.loads(payload))
            original(payload)

        bridge.send = send
        backend = RosbridgeROS2Backend(
            url="ws://127.0.0.1:1", connection_factory=lambda _url: bridge, presence_only=True
        )
        made.append((backend, bridge))
        return backend, sent

    yield build
    for backend, _ in made:
        backend.disconnect()


def published(sent):
    return [m for m in sent if m.get("op") in ("advertise", "publish")]


def test_the_simulated_publish_is_a_volatile_bool_repeated_by_the_caller(backend_on):
    backend, sent = backend_on(SIM_GRAPH)
    backend.publish_simulated_state("/flyto/cargo_present", True)
    backend.publish_simulated_state("/flyto/cargo_present", False)
    advertised = [m for m in sent if m["op"] == "advertise"]
    assert advertised == [
        {
            "op": "advertise",
            "topic": "/flyto/cargo_present",
            "type": "std_msgs/msg/Bool",
            "qos": {
                "history": "keep_last",
                "depth": 1,
                "reliability": "reliable",
                "durability": "volatile",
            },
        }
    ]
    assert [m["msg"] for m in sent if m["op"] == "publish"] == [{"data": True}, {"data": False}]


def test_the_simulated_publish_refuses_a_hardware_configuration(backend_on):
    backend, sent = backend_on(SIM_GRAPH, mode="hardware")
    with pytest.raises(SimulationOnlyError, match="simulation"):
        backend.publish_simulated_state("/flyto/cargo_present", True)
    assert published(sent) == []


def test_the_simulated_publish_refuses_a_graph_without_the_marker(backend_on):
    backend, sent = backend_on(HARDWARE_GRAPH)
    with pytest.raises(SimulationOnlyError, match="/clock"):
        backend.publish_simulated_state("/flyto/cargo_present", True)
    assert published(sent) == []


def test_the_simulated_publish_refuses_with_the_marker_check_disabled(backend_on, monkeypatch):
    backend, sent = backend_on(SIM_GRAPH)
    monkeypatch.setenv("FLYTO_ROS2_SIM_MARKER_TOPIC", "")
    with pytest.raises(SimulationOnlyError):
        backend.publish_simulated_state("/flyto/cargo_present", True)
    assert published(sent) == []


@pytest.mark.parametrize(
    ("topic", "value"),
    [("/cmd_vel", True), ("/flyto/cargo_present", 1), ("flyto/cargo_present", True)],
)
def test_the_simulated_publish_sends_only_a_boolean_state(backend_on, topic, value):
    backend, sent = backend_on(SIM_GRAPH)
    with pytest.raises((ValueError, SimulationOnlyError)):
        backend.publish_simulated_state(topic, value)
    assert published(sent) == []


def test_the_bridge_publishes_on_the_topic_the_adapter_reads(monkeypatch):
    monkeypatch.delenv(cargo_sensor.TOPIC_ENV, raising=False)
    assert cargo_bridge.bridge_topic() == cargo_sensor.CONVENTIONAL_TOPIC
    monkeypatch.setenv(cargo_sensor.TOPIC_ENV, "/robot/tray")
    assert cargo_bridge.bridge_topic() == "/robot/tray"


# --- the offline mission pushes the trail -----------------------------------------------


def test_the_offline_mission_pushes_a_report_trail_that_loads_and_unloads(tmp_path):
    pytest.importorskip("mujoco")
    pytest.importorskip("flyto_modules_lerobot")
    from test_fetch_return_sim import CONFIG

    if not CONFIG.is_file():
        pytest.skip("external LeRobot config not installed")
    from sim.fetch_return.run import run_mission

    path = tmp_path / "reports.jsonl"
    path.write_text("")
    tail = reports.ArmReportTail(path)
    report = run_mission("派機器人去把紅色積木拿回來", config_path=CONFIG, reports_path=path)
    assert report["state"] == "done"
    trail = tail.read_new()
    assert tail.rejected == 0
    kinds = [r.kind for r in trail]
    assert kinds[0] == "place"
    assert kinds[-3:] == ["retract", "unload", "mission_end"]
    assert reports.settled_run([r for r in trail if r.kind == "tray_reading"]) is not None
    place = trail[0]
    handoff = CargoHandoff(STATION)
    decided = trail[-3].at + 1.0
    _parked(handoff, place.fields["started_at"] - 1.0, decided, every=0.25)
    handoff.observe_pose(decided, AT_STATION)
    handoff.observe_reports(trail[:-2])
    handoff.evaluate(decided)
    assert handoff.cargo_present is True
    handoff.observe_reports(trail[-2:])
    handoff.evaluate(trail[-1].at + 1.0)
    assert handoff.cargo_present is False
