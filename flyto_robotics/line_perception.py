"""Transport-neutral color-line perception for cameras and deterministic tests."""

from __future__ import annotations

from dataclasses import dataclass

SUPPORTED_COLORS = ("black", "blue", "green", "purple", "red", "white", "yellow")


@dataclass(frozen=True)
class LineObservation:
    """Centroid observation for one colored route segment."""

    color: str
    visible: bool
    confidence: float
    lateral_error: float
    pixel_count: int


@dataclass(frozen=True)
class LineScene:
    """All route colors found in one camera frame."""

    detections: tuple[LineObservation, ...]

    def get(self, color: str) -> LineObservation | None:
        return next((item for item in self.detections if item.color == color), None)


def _matches(color: str, red: int, green: int, blue: int) -> bool:
    high = max(red, green, blue)
    low = min(red, green, blue)
    if color == "black":
        return high < 55
    if color == "white":
        return low > 175 and high - low < 55
    if color == "red":
        return red > 90 and red > green * 1.35 and red > blue * 1.25
    if color == "green":
        return green > 75 and green > red * 1.25 and green > blue * 1.15
    if color == "blue":
        return blue > 80 and blue > red * 1.35 and blue > green * 1.12
    if color == "yellow":
        return red > 95 and green > 85 and blue < min(red, green) * 0.72
    if color == "purple":
        return (
            red > 65
            and blue > 75
            and green < min(red, blue) * 0.78
            and abs(red - blue) < 150
        )
    return False


def detect_line_scene(
    data: bytes | bytearray | memoryview,
    *,
    width: int,
    height: int,
    encoding: str,
    step: int = 0,
    colors: tuple[str, ...] = SUPPORTED_COLORS,
) -> LineScene:
    """Detect colored route centroids in the lower camera region.

    This intentionally uses a small deterministic classifier. A learned
    segmentation adapter can later emit the same ``LineScene`` contract.
    """
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")
    normalized_encoding = encoding.lower()
    if normalized_encoding not in {"rgb8", "bgr8"}:
        raise ValueError("line perception supports rgb8 or bgr8 images")
    unsupported = sorted(set(colors) - set(SUPPORTED_COLORS))
    if unsupported:
        raise ValueError(f"unsupported route colors: {', '.join(unsupported)}")

    row_stride = step or width * 3
    minimum_bytes = row_stride * height
    view = memoryview(data)
    if len(view) < minimum_bytes:
        raise ValueError("image payload is shorter than width, height, and step require")

    start_y = max(0, int(height * 0.42))
    end_y = max(start_y + 1, int(height * 0.90))
    sample_stride = 2 if width >= 80 else 1
    x_totals = {color: 0 for color in colors}
    counts = {color: 0 for color in colors}
    sampled_pixels = 0
    for y in range(start_y, end_y, sample_stride):
        row_offset = y * row_stride
        for x in range(0, width, sample_stride):
            offset = row_offset + x * 3
            first, green, third = view[offset : offset + 3]
            if normalized_encoding == "rgb8":
                red, blue = first, third
            else:
                blue, red = first, third
            sampled_pixels += 1
            for color in colors:
                if _matches(color, red, green, blue):
                    counts[color] += 1
                    x_totals[color] += x

    minimum_count = max(6, int(sampled_pixels * 0.0025))
    confidence_denominator = max(1.0, sampled_pixels * 0.025)
    center = max(1.0, (width - 1) / 2.0)
    detections: list[LineObservation] = []
    for color in colors:
        count = counts[color]
        visible = count >= minimum_count
        centroid = x_totals[color] / count if count else center
        detections.append(
            LineObservation(
                color=color,
                visible=visible,
                confidence=min(1.0, count / confidence_denominator),
                lateral_error=max(-1.0, min(1.0, (centroid - center) / center)),
                pixel_count=count,
            )
        )
    return LineScene(tuple(detections))
