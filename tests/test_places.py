"""Named places: a host-side store, two capabilities, and navigation by name.

Every guarantee here is one an operator relies on without seeing it: an
unknown place never moves the robot, a place and coordinates are never mixed,
a saved place reads back exactly, the navigation is sent to (and reports) the
coordinates the place stood for, and a places file that cannot be read is
refused rather than guessed at or overwritten.
"""

from __future__ import annotations

import base64
import json
import math
import os
from pathlib import Path

import pytest

from flyto_robotics import adapter_contract as decl
from flyto_robotics import places
from flyto_robotics.adapter_contract import (
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_REFUSED,
    OUTCOME_TIMEOUT,
    CallRequest,
)
from flyto_robotics.generic_ros2_adapter import (
    ARGUMENTS,
    PLACES_LIST,
    PLACES_MARK,
    GenericROS2Adapter,
)
from flyto_robotics.places import PlacesError, PlacesStore, PlacesStoreError, UnknownPlace
from tests.test_generic_ros2_adapter import FakeROS2Backend

LOBBY_POSE = {"frame": "map", "x": 1.5, "y": -2.25, "yaw": 0.5}


def store(tmp_path: Path, map_name: str = "default") -> PlacesStore:
    return PlacesStore(tmp_path / "places" / f"{map_name}.json", map_name=map_name)


def adapter(tmp_path: Path, capabilities=None) -> GenericROS2Adapter:
    device = GenericROS2Adapter(
        backend=FakeROS2Backend(capabilities),
        resource_id="standard-turtlebot3",
        places_store=store(tmp_path),
    )
    device.describe()
    return device


def at(device: GenericROS2Adapter, pose) -> None:
    device.backend.observation_payload["map_pose"] = dict(pose)


# --- the store ------------------------------------------------------------------


def test_mark_then_list_round_trips_free_text_names(tmp_path):
    places_store = store(tmp_path)
    saved, replaced = places_store.mark("  一樓大廳  ", LOBBY_POSE)

    assert replaced is None
    assert saved.to_dict() == {"name": "一樓大廳", "frame": "map", "x": 1.5, "y": -2.25, "yaw": 0.5}
    assert [place.to_dict() for place in places_store.list()] == [saved.to_dict()]
    assert places_store.resolve("一樓大廳") == saved
    document = json.loads(places_store.path.read_text(encoding="utf-8"))
    assert document["schema"] == places.PLACES_SCHEMA
    assert document["map_id"] == "default"
    # Atomic: nothing but the file and its lock is left behind.
    assert sorted(item.name for item in places_store.path.parent.iterdir()) == [
        ".default.json.lock",
        "default.json",
    ]


def test_names_are_unique_per_map_ignoring_case_and_a_second_mark_moves_it(tmp_path):
    places_store = store(tmp_path)
    places_store.mark("Lobby", LOBBY_POSE)
    places_store.mark("Dock", {"frame": "map", "x": 0.0, "y": 0.0, "yaw": 0.0})

    moved, replaced = places_store.mark("lobby", {"frame": "map", "x": 3.0, "y": 4.0, "yaw": -1.0})

    assert replaced is not None and replaced.x == 1.5
    assert [place.name for place in places_store.list()] == ["lobby", "Dock"]
    assert places_store.resolve("LOBBY") == moved


def test_another_map_keeps_its_own_places(tmp_path):
    first = store(tmp_path, "floor-1")
    second = store(tmp_path, "floor-2")
    first.mark("Lobby", LOBBY_POSE)

    assert second.list() == []
    with pytest.raises(UnknownPlace):
        second.resolve("Lobby")


@pytest.mark.parametrize(
    "name",
    [
        "", "   ", "x" * (places.MAX_NAME_LENGTH + 1), "line\nbreak", "tab\there",
        "nul\x00", "c1\x85in", "sep\u2028x", 7, None,
    ],
)
def test_bad_names_are_refused(tmp_path, name):
    with pytest.raises(PlacesError):
        store(tmp_path).mark(name, LOBBY_POSE)
    assert not store(tmp_path).path.exists()


def test_the_longest_name_counts_characters_not_bytes(tmp_path):
    name = "廳" * places.MAX_NAME_LENGTH
    saved, _ = store(tmp_path).mark(name, LOBBY_POSE)
    assert saved.name == name


@pytest.mark.parametrize(
    "pose",
    [
        {"frame": "odom", "x": 0.0, "y": 0.0, "yaw": 0.0},
        {"x": 0.0, "y": 0.0, "yaw": 0.0},
        {"frame": "map", "x": float("nan"), "y": 0.0, "yaw": 0.0},
        {"frame": "map", "x": 0.0, "y": 0.0},
        {"frame": "map", "x": 1001.0, "y": 0.0, "yaw": 0.0},
        {"frame": "map", "x": True, "y": 0.0, "yaw": 0.0},
    ],
)
def test_only_a_finite_map_frame_pose_is_saved(tmp_path, pose):
    with pytest.raises(PlacesError):
        store(tmp_path).mark("Lobby", pose)


def test_a_heading_is_kept_within_a_half_turn(tmp_path):
    turned = {"frame": "map", "x": 0.0, "y": 0.0, "yaw": 3 * math.pi / 2}
    saved, _ = store(tmp_path).mark("Turned", turned)
    assert saved.yaw == pytest.approx(-math.pi / 2)
    assert store(tmp_path).resolve("Turned").yaw == saved.yaw


def test_unknown_place_names_the_ones_there_are(tmp_path):
    places_store = store(tmp_path)
    places_store.mark("Lobby", LOBBY_POSE)
    places_store.mark("Dock", LOBBY_POSE)

    with pytest.raises(UnknownPlace) as raised:
        places_store.resolve("Kitchen")
    assert raised.value.known == ("Lobby", "Dock")
    assert "Lobby, Dock" in str(raised.value)


def test_a_map_holds_a_bounded_number_of_places(tmp_path, monkeypatch):
    monkeypatch.setattr(places, "MAX_PLACES", 2)
    places_store = store(tmp_path)
    places_store.mark("a", LOBBY_POSE)
    places_store.mark("b", LOBBY_POSE)
    with pytest.raises(PlacesError):
        places_store.mark("c", LOBBY_POSE)
    # Moving one that exists is still allowed.
    places_store.mark("A", LOBBY_POSE)


def _valid_document(**changes):
    document = {
        "schema": places.PLACES_SCHEMA,
        "map_id": "default",
        "places": [{"name": "Lobby", "frame": "map", "x": 1.0, "y": 2.0, "yaw": 0.0}],
    }
    document.update(changes)
    return json.dumps(document)


def _entry(**changes):
    entry = {"name": "Lobby", "frame": "map", "x": 1.0, "y": 2.0, "yaw": 0.0}
    entry.update(changes)
    return entry


CORRUPT_FILES = {
    "not json": "{not json",
    "truncated": _valid_document()[:-7],
    "not utf-8": b"\xff\xfe\x00".decode("latin-1"),
    "a list": "[]",
    "another schema": _valid_document(schema="flyto.robot-places.v0"),
    "another map": _valid_document(map_id="floor-9"),
    "an extra key": json.dumps({**json.loads(_valid_document()), "owner": "x"}),
    "places not a list": _valid_document(places={"Lobby": [1, 2]}),
    "an entry missing yaw": _valid_document(
        places=[{"name": "Lobby", "frame": "map", "x": 1.0, "y": 2.0}]
    ),
    "an entry with an extra key": _valid_document(places=[_entry(z=0.0)]),
    "an entry in odom": _valid_document(places=[_entry(frame="odom")]),
    "a NaN coordinate": _valid_document(places=[_entry(x=float("nan"))]),
    "a text coordinate": _valid_document(places=[_entry(x="1.0")]),
    "an untrimmed name": _valid_document(places=[_entry(name=" Lobby")]),
    "an empty name": _valid_document(places=[_entry(name="")]),
    "a heading past a half turn": _valid_document(places=[_entry(yaw=4.0)]),
    "a name twice": _valid_document(places=[_entry(), _entry(name="LOBBY")]),
}


@pytest.mark.parametrize("label", sorted(CORRUPT_FILES))
def test_a_corrupt_file_fails_closed_and_is_never_overwritten(tmp_path, label):
    places_store = store(tmp_path)
    places_store.path.parent.mkdir(parents=True)
    raw = CORRUPT_FILES[label]
    if label == "not utf-8":
        places_store.path.write_bytes(b"\xff\xfe\x00")
    else:
        places_store.path.write_text(raw, encoding="utf-8")
    before = places_store.path.read_bytes()

    with pytest.raises(PlacesStoreError):
        places_store.list()
    with pytest.raises(PlacesStoreError):
        places_store.resolve("Lobby")
    with pytest.raises(PlacesStoreError):
        places_store.mark("Dock", LOBBY_POSE)
    assert places_store.path.read_bytes() == before


def test_an_oversized_file_fails_closed(tmp_path):
    places_store = store(tmp_path)
    places_store.path.parent.mkdir(parents=True)
    places_store.path.write_text(" " * (places.MAX_FILE_BYTES + 1), encoding="utf-8")
    with pytest.raises(PlacesStoreError):
        places_store.list()


def test_a_missing_file_is_an_empty_map(tmp_path):
    assert store(tmp_path).list() == []


# --- where the file lives --------------------------------------------------------


def test_the_file_is_named_outright_when_configured(tmp_path, monkeypatch):
    monkeypatch.setenv(places.PLACES_FILE_ENV, str(tmp_path / "site-places.json"))
    monkeypatch.setenv(places.MAP_ID_ENV, "floor-2")
    places_store = PlacesStore.for_resource("robot-1")
    assert places_store.path == tmp_path / "site-places.json"
    assert places_store.map_id == "floor-2"


def test_the_default_file_is_per_robot_and_map_under_the_data_dir(tmp_path, monkeypatch):
    monkeypatch.delenv(places.PLACES_FILE_ENV, raising=False)
    monkeypatch.delenv(places.MAP_ID_ENV, raising=False)
    monkeypatch.setenv(places.DATA_DIR_ENV, str(tmp_path))
    expected = tmp_path / "places" / "robot-1" / "default.json"
    assert PlacesStore.for_resource("robot-1").path == expected

    monkeypatch.delenv(places.DATA_DIR_ENV)
    monkeypatch.setenv("XDG_DATA_HOME", str(tmp_path / "xdg"))
    if os.name != "nt":
        assert PlacesStore.for_resource("robot-1").path == (
            tmp_path / "xdg" / "flyto-robotics" / "places" / "robot-1" / "default.json"
        )


def test_an_id_never_escapes_the_data_dir(tmp_path, monkeypatch):
    monkeypatch.delenv(places.PLACES_FILE_ENV, raising=False)
    monkeypatch.setenv(places.DATA_DIR_ENV, str(tmp_path))
    monkeypatch.setenv(places.MAP_ID_ENV, "../../etc")
    path = PlacesStore.for_resource("../../robot").path
    assert path.resolve().is_relative_to(tmp_path.resolve())
    # Two ids that sanitise alike do not share a file.
    assert places.default_path("a/b") != places.default_path("a-b")


# --- declared beside navigation ---------------------------------------------------


def test_places_are_declared_with_navigation_and_only_then(tmp_path):
    declared = {item.capability_id for item in adapter(tmp_path).describe()}
    assert {PLACES_LIST, PLACES_MARK} <= declared

    without = {item.capability_id for item in adapter(tmp_path, {"motion.rotate"}).describe()}
    assert without.isdisjoint({PLACES_LIST, PLACES_MARK})


def test_declared_metadata_and_text_argument(tmp_path):
    by_id = {item.capability_id: item for item in adapter(tmp_path).describe()}
    assert by_id[PLACES_LIST].safety_class == "read_only"
    assert by_id[PLACES_MARK].safety_class == "controlled"
    assert not by_id[PLACES_MARK].requires_safe_stop and not by_id[PLACES_MARK].cancellable

    schema = decl.arguments_to_json_schema(by_id["motion.navigate"].arguments)
    assert schema["properties"]["place"] == {
        "type": "string",
        "description": "A named place on this map, instead of x and y",
        "minLength": 1,
        "maxLength": places.MAX_NAME_LENGTH,
    }
    # Either target is allowed, so neither is required by the schema itself.
    assert "required" not in schema
    mark = decl.arguments_to_json_schema(by_id[PLACES_MARK].arguments)
    assert mark["required"] == ["place"]


def test_a_numeric_argument_declares_exactly_what_it_did_before():
    """No max_length key, so declarations without text keep their schema hash."""
    advance = ARGUMENTS["motion.advance"][0].to_dict()
    assert set(advance) == {"name", "type", "required", "description", "minimum", "maximum", "unit"}


# --- places.mark and places.list --------------------------------------------------


def test_mark_saves_the_map_pose_and_list_returns_it(tmp_path):
    device = adapter(tmp_path)
    at(device, LOBBY_POSE)

    marked = device.invoke(CallRequest("m1", PLACES_MARK, {"place": "一樓大廳"}))
    listed = device.invoke(CallRequest("l1", PLACES_LIST, {}))

    assert marked.outcome == OUTCOME_COMPLETED
    assert marked.evidence["place"] == {"name": "一樓大廳", **LOBBY_POSE}
    assert marked.evidence["replaced"] is None
    assert listed.outcome == OUTCOME_COMPLETED
    assert listed.evidence["places"] == [{"name": "一樓大廳", **LOBBY_POSE}]
    assert listed.evidence["map_id"] == "default"
    artifact = listed.evidence["artifacts"][0]
    assert (artifact["kind"], artifact["media_type"]) == ("places", "application/json")
    assert json.loads(base64.b64decode(artifact["data_base64"])) == listed.evidence["places"]
    # Neither call commands the robot.
    assert device.backend.calls == []


def test_mark_waits_for_the_map_transform_and_refuses_without_a_map_pose(tmp_path):
    device = adapter(tmp_path)
    device.backend.observation_payload.pop("map_pose", None)

    result = device.invoke(CallRequest("m1", PLACES_MARK, {"place": "Lobby"}))

    assert result.outcome == OUTCOME_REFUSED
    assert "map-frame pose" in result.detail
    assert set(device.backend.required) == {"pose", "map_tf"}
    assert not device.places_store.path.exists()


def test_the_same_mark_call_again_does_not_save_the_pose_it_is_at_now(tmp_path):
    device = adapter(tmp_path)
    at(device, LOBBY_POSE)
    first = device.invoke(CallRequest("m1", PLACES_MARK, {"place": "Lobby"}))
    at(device, {"frame": "map", "x": 9.0, "y": 9.0, "yaw": 0.0})

    again = device.invoke(CallRequest("m1", PLACES_MARK, {"place": "Lobby"}))

    assert again == first
    assert device.places_store.resolve("Lobby").x == 1.5


@pytest.mark.parametrize(
    "arguments", [{}, {"place": ""}, {"place": "a\nb"}, {"place": 3}, {"name": "x"}]
)
def test_mark_refuses_a_bad_name(tmp_path, arguments):
    device = adapter(tmp_path)
    at(device, LOBBY_POSE)
    assert device.invoke(CallRequest("m", PLACES_MARK, arguments)).outcome == OUTCOME_REFUSED
    assert not device.places_store.path.exists()


def test_list_and_mark_fail_closed_on_a_corrupt_file(tmp_path):
    device = adapter(tmp_path)
    at(device, LOBBY_POSE)
    device.places_store.path.parent.mkdir(parents=True)
    device.places_store.path.write_text("{oops", encoding="utf-8")

    listed = device.invoke(CallRequest("l", PLACES_LIST, {}))
    marked = device.invoke(CallRequest("m", PLACES_MARK, {"place": "Lobby"}))

    assert listed.outcome == OUTCOME_FAILED and "unreadable" in listed.detail
    assert marked.outcome == OUTCOME_FAILED
    assert device.places_store.path.read_text(encoding="utf-8") == "{oops"


# --- navigate by place --------------------------------------------------------------


def test_navigate_by_place_sends_the_resolved_pose_and_reports_it(tmp_path):
    device = adapter(tmp_path)
    device.places_store.mark("Lobby", LOBBY_POSE)

    result = device.invoke(CallRequest("n1", "motion.navigate", {"place": "lobby"}, 30.0))

    assert result.outcome == OUTCOME_COMPLETED
    # The goal handed to Nav2 is the stored pose, heading included.
    sent = {"x": 1.5, "y": -2.25, "yaw_radians": 0.5}
    assert device.backend.calls == [("n1", "motion.navigate", sent)]
    assert result.evidence["navigation_target"] == {
        "frame": "map", "x": 1.5, "y": -2.25, "yaw_radians": 0.5, "place": "Lobby",
    }
    # What the arrival evidence (distance_to x/y, angle_to yaw_radians) is
    # judged against: exactly the coordinates the robot was sent to.
    assert result.evidence["resolved_arguments"] == {"x": 1.5, "y": -2.25, "yaw_radians": 0.5}


def test_navigate_by_coordinates_reports_its_target_and_resolves_nothing(tmp_path):
    device = adapter(tmp_path)
    result = device.invoke(CallRequest("n1", "motion.navigate", {"x": 1.0, "y": 2.0}, 30.0))

    assert result.outcome == OUTCOME_COMPLETED
    assert result.evidence["navigation_target"] == {"frame": "map", "x": 1.0, "y": 2.0}
    assert "resolved_arguments" not in result.evidence


def test_an_unknown_place_refuses_with_the_known_names_and_no_motion(tmp_path):
    device = adapter(tmp_path)
    device.places_store.mark("Lobby", LOBBY_POSE)
    device.places_store.mark("Dock", LOBBY_POSE)

    result = device.invoke(CallRequest("n1", "motion.navigate", {"place": "Kitchen"}, 30.0))

    assert result.outcome == OUTCOME_REFUSED
    assert result.evidence["known_places"] == ["Lobby", "Dock"]
    assert "Kitchen" in result.detail and "Lobby, Dock" in result.detail
    assert device.backend.calls == []
    assert device.backend.counts == {}


@pytest.mark.parametrize(
    "arguments",
    [
        {"place": "Lobby", "x": 1.0, "y": 2.0},
        {"place": "Lobby", "x": 1.0},
        {"place": "Lobby", "yaw_radians": 0.3},
        {"x": 1.0},
        {"yaw_radians": 0.3},
        {},
    ],
)
def test_place_and_coordinates_are_exclusive(tmp_path, arguments):
    device = adapter(tmp_path)
    device.places_store.mark("Lobby", LOBBY_POSE)

    result = device.invoke(CallRequest("n1", "motion.navigate", arguments, 30.0))

    assert result.outcome == OUTCOME_REFUSED
    assert device.backend.calls == []


def test_navigate_by_place_refuses_with_no_motion_when_the_file_is_corrupt(tmp_path):
    device = adapter(tmp_path)
    device.places_store.path.parent.mkdir(parents=True)
    device.places_store.path.write_text(_valid_document(map_id="floor-9"), encoding="utf-8")

    result = device.invoke(CallRequest("n1", "motion.navigate", {"place": "Lobby"}, 30.0))

    assert result.outcome == OUTCOME_REFUSED
    assert "unreadable" in result.detail
    assert device.backend.calls == []


def test_a_resumed_navigation_keeps_the_target_it_was_sent_to(tmp_path):
    device = adapter(tmp_path)
    device.places_store.mark("Lobby", LOBBY_POSE)
    first = device.invoke(CallRequest("n1", "motion.navigate", {"place": "Lobby"}, 0.001))
    assert first.outcome == OUTCOME_TIMEOUT
    device.places_store.mark("Lobby", {"frame": "map", "x": 8.0, "y": 8.0, "yaw": 0.0})
    device.backend.active.discard("n1")

    resumed = device.invoke(CallRequest("n1", "motion.navigate", {"place": "Lobby"}, 30.0))

    assert resumed.evidence["resolved_arguments"]["x"] == 1.5
    assert len(device.backend.calls) == 1
