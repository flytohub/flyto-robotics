"""Clearance readings, and the refusal to let "I cannot see" look like room."""

from __future__ import annotations

import math
import os
import re
from pathlib import Path

import pytest

from flyto_robotics.scan_clearance import (
    SECTORS,
    UNREADABLE,
    describe,
    is_clear,
    sector_clearance,
)

FAR = 5.0


def sweep(beam_count: int = 360, **near_beams: float):
    """A clear sweep with specific beams brought close.

    Keys are ``b<index>`` so they survive being keyword arguments.
    """
    ranges = [FAR] * beam_count
    for key, value in near_beams.items():
        ranges[int(key[1:]) % beam_count] = value
    return ranges


class TestReadingASector:
    def test_finds_the_closest_thing_ahead(self):
        assert sector_clearance(sweep(b5=0.4), "front") == pytest.approx(0.4)

    def test_a_sweep_wraps_around_zero(self):
        """Straight ahead is index 0, so its wedge spans the end of the array."""
        assert sector_clearance(sweep(b355=0.3), "front") == pytest.approx(0.3)

    @pytest.mark.parametrize(
        ("sector", "index"),
        [("front", 0), ("left", 90), ("rear", 180), ("right", 270)],
    )
    def test_each_sector_looks_where_it_says(self, sector, index):
        ranges = sweep(**{f"b{index}": 0.5})
        assert sector_clearance(ranges, sector) == pytest.approx(0.5)
        others = [other for other in SECTORS if other != sector]
        for other in others:
            assert sector_clearance(ranges, other) == pytest.approx(FAR)

    def test_ignores_things_outside_the_wedge(self):
        assert sector_clearance(sweep(b90=0.2), "front") == pytest.approx(FAR)

    def test_works_on_a_sweep_that_is_not_360_beams(self):
        """The lab robot publishes 399."""
        assert sector_clearance(sweep(399, b2=0.36), "front") == pytest.approx(0.36)

    def test_rejects_an_unknown_sector(self):
        with pytest.raises(ValueError, match="sector must be one of"):
            sector_clearance(sweep(), "up")


class TestWhatCountsAsUnreadable:
    def test_an_empty_sweep_is_unreadable(self):
        assert sector_clearance([], "front") is UNREADABLE

    def test_all_beams_dropped_is_unreadable(self):
        assert sector_clearance([math.nan] * 360, "front") is UNREADABLE

    def test_beams_below_the_trusted_band_are_unreadable(self):
        """Zeros are how this lidar reports no return, not a wall at 0 m."""
        assert sector_clearance([0.0] * 360, "front") is UNREADABLE

    def test_beams_past_the_trusted_band_are_unreadable(self):
        assert sector_clearance([99.0] * 360, "front") is UNREADABLE

    def test_one_good_beam_among_dropped_ones_is_a_reading(self):
        ranges = [math.nan] * 360
        ranges[3] = 0.8
        assert sector_clearance(ranges, "front") == pytest.approx(0.8)


class TestRefusingToDriveBlind:
    def test_enough_room_is_clear(self):
        assert is_clear(1.2, 0.70) is True

    def test_exactly_the_required_room_is_clear(self):
        assert is_clear(0.70, 0.70) is True

    def test_too_little_room_is_not_clear(self):
        assert is_clear(0.36, 0.70) is False

    def test_unreadable_is_not_clear(self):
        """The defect this module was written for.

        An operator script treated an unreadable scan as 99 m and drove on. The
        robot was 0.36 m from something at the time. Not knowing must refuse.
        """
        assert is_clear(UNREADABLE, 0.70) is False

    def test_unreadable_is_refused_however_small_the_requirement(self):
        assert is_clear(UNREADABLE, 0.0) is False


class TestDescribe:
    def test_a_reading_is_reported_in_metres(self):
        assert describe(0.36) == "0.36 m"

    def test_ignorance_is_never_reported_as_a_number(self):
        text = describe(UNREADABLE)
        assert "unreadable" in text
        assert not any(character.isdigit() for character in text)


class TestOperatorScript:
    """The move script is the surface the defect actually appeared on."""

    SCRIPT = Path(__file__).resolve().parents[1] / "scripts/move-robot.sh"

    def code(self) -> str:
        """The script with comment lines dropped.

        The header explains the defect by quoting it, and a guard that reads
        its own explanation as a violation would force the explanation out —
        losing the one thing that stops someone reintroducing the bug on
        purpose. Comments are not code; match against what runs.
        """
        return "\n".join(
            line
            for line in self.SCRIPT.read_text().splitlines()
            if not line.lstrip().startswith("#")
        )

    def test_the_script_exists_and_is_executable(self):
        assert self.SCRIPT.is_file()
        assert os.access(self.SCRIPT, os.X_OK), "operators run this directly"

    def test_it_asks_this_module_rather_than_parsing_the_scan_itself(self):
        assert "from flyto_robotics.scan_clearance import" in self.code()

    def test_it_never_substitutes_a_number_for_an_unreadable_scan(self):
        """The regression guard.

        The version this replaced parsed the sweep with a bare
        ``except: print('99')``. A blind robot reported 99 m of room and the
        safety check waved it through. Nothing here may swallow a parse failure
        into a distance again.
        """
        body = self.code()
        assert re.search(r"except[^\n]*:\s*print", body) is None
        assert "99" not in body
