"""How much room a lidar sweep says there is, decided without importing ROS.

Reading a sector minimum out of a ranges array is arithmetic, but it kept being
written inline next to the code that fetches the scan — and the two failure
modes then look identical. "There is nothing within 12 m" and "I could not read
the sensor" are both an empty list of usable beams, and an operator script that
collapsed them printed a clearance of 99 m for a robot that was blind.

So the answer here is either a distance or :data:`UNREADABLE`, and it is never a
number standing in for ignorance. The caller decides what to do about not
knowing; it does not get to mistake it for room.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Literal

Sector = Literal["front", "rear", "left", "right"]

#: The sector could not be measured. Distinct from "measured, and it is far".
UNREADABLE = None

#: Beams outside this band are dropped: below is the sensor seeing itself or
#: dropping out, above is past anything a range finder should be trusted on.
MIN_VALID_RANGE = 0.02
MAX_VALID_RANGE = 12.0

#: Half-width of a sector, in degrees either side of its centre.
DEFAULT_HALF_WIDTH_DEGREES = 20.0

#: Where each sector's centre sits, as a fraction of a full sweep. A TurtleBot3
#: publishes /scan starting at straight ahead and going counter-clockwise.
_SECTOR_CENTRE_FRACTION: dict[str, float] = {
    "front": 0.0,
    "left": 0.25,
    "rear": 0.5,
    "right": 0.75,
}

SECTORS: tuple[str, ...] = tuple(_SECTOR_CENTRE_FRACTION)


def sector_clearance(
    ranges: Sequence[float],
    sector: Sector,
    *,
    half_width_degrees: float = DEFAULT_HALF_WIDTH_DEGREES,
) -> float | None:
    """Closest valid return in ``sector``, or :data:`UNREADABLE`.

    :param ranges: one full lidar sweep, in metres, in publication order.
    :param sector: which way to look.
    :param half_width_degrees: how wide a wedge to consider, either side of the
        sector's centre.
    :returns: the distance to the nearest thing in that wedge, or ``None`` when
        the sweep carries no usable beam there — no reading, all beams dropped
        out, or every beam out of the trusted band.
    :raises ValueError: if ``sector`` is not one of :data:`SECTORS`.
    """
    if sector not in _SECTOR_CENTRE_FRACTION:
        raise ValueError(f"sector must be one of {', '.join(SECTORS)}")

    beam_count = len(ranges)
    if beam_count == 0:
        return UNREADABLE

    half_width = max(1, round(beam_count * half_width_degrees / 360.0))
    centre = round(beam_count * _SECTOR_CENTRE_FRACTION[sector])
    wedge = (
        ranges[(centre + offset) % beam_count]
        for offset in range(-half_width, half_width + 1)
    )

    usable = [
        beam
        for beam in wedge
        # NaN fails every comparison, which is how a dropped beam arrives.
        if not math.isnan(beam) and MIN_VALID_RANGE < beam < MAX_VALID_RANGE
    ]
    if not usable:
        return UNREADABLE
    return min(usable)


def is_clear(
    clearance: float | None,
    required_metres: float,
) -> bool:
    """Whether it is safe to drive, refusing when the answer is not known.

    An unreadable sector is not clear. This is the whole reason the module
    exists: a robot that cannot see must not be treated as a robot with room.
    """
    if clearance is UNREADABLE:
        return False
    return clearance >= required_metres


def describe(clearance: float | None) -> str:
    """A one-line report that never dresses ignorance up as a measurement."""
    if clearance is UNREADABLE:
        return "unreadable (no usable lidar return)"
    return f"{clearance:.2f} m"
