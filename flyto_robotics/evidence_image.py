"""Dependency-free PNG encoding for Gazebo camera evidence."""

from __future__ import annotations

import binascii
import struct
import tempfile
import zlib
from pathlib import Path

MAX_IMAGE_DIMENSION = 4096
SUPPORTED_ENCODINGS = frozenset({"rgb8", "bgr8"})
MAX_VIDEO_FRAMES = 3600


def _chunk(kind: bytes, payload: bytes) -> bytes:
    checksum = binascii.crc32(kind)
    checksum = binascii.crc32(payload, checksum) & 0xFFFFFFFF
    return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", checksum)


def encode_rgb_png(
    *,
    width: int,
    height: int,
    encoding: str,
    step: int,
    data: bytes,
    compression_level: int = 9,
) -> bytes:
    """Encode one bounded ROS RGB/BGR image as a standards-compliant PNG."""
    if not isinstance(width, int) or not isinstance(height, int):
        raise ValueError("image width and height must be integers")
    if not 1 <= width <= MAX_IMAGE_DIMENSION or not 1 <= height <= MAX_IMAGE_DIMENSION:
        raise ValueError("image dimensions are outside the supported range")
    normalized_encoding = encoding.lower()
    if normalized_encoding not in SUPPORTED_ENCODINGS:
        raise ValueError(f"unsupported image encoding: {encoding}")
    row_bytes = width * 3
    if step < row_bytes:
        raise ValueError("image step is smaller than the RGB row width")
    required = step * height
    if len(data) < required:
        raise ValueError("image payload is shorter than step * height")
    if (
        not isinstance(compression_level, int)
        or not 0 <= compression_level <= 9
    ):
        raise ValueError("PNG compression level must be between 0 and 9")

    rows = bytearray()
    for row_index in range(height):
        offset = row_index * step
        row = bytearray(data[offset : offset + row_bytes])
        if normalized_encoding == "bgr8":
            for pixel in range(0, len(row), 3):
                row[pixel], row[pixel + 2] = row[pixel + 2], row[pixel]
        rows.append(0)
        rows.extend(row)

    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    return b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", header) + _chunk(
        b"IDAT", zlib.compress(bytes(rows), level=compression_level)
    ) + _chunk(b"IEND", b"")


def write_rgb_png_atomic(
    destination: Path,
    *,
    width: int,
    height: int,
    encoding: str,
    step: int,
    data: bytes,
    compression_level: int = 9,
) -> None:
    """Atomically write camera evidence without leaving partial image files."""
    encoded = encode_rgb_png(
        width=width,
        height=height,
        encoding=encoding,
        step=step,
        data=data,
        compression_level=compression_level,
    )
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb",
            prefix=f".{destination.name}.",
            suffix=".tmp",
            dir=destination.parent,
            delete=False,
        ) as handle:
            handle.write(encoded)
            handle.flush()
            temporary_path = Path(handle.name)
        temporary_path.replace(destination)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


class VideoFrameSequence:
    """Write a bounded, contiguous PNG sequence for later video encoding."""

    def __init__(self, directory: Path, *, max_frames: int = 600) -> None:
        if not isinstance(max_frames, int) or not 1 <= max_frames <= MAX_VIDEO_FRAMES:
            raise ValueError(
                f"max_frames must be between 1 and {MAX_VIDEO_FRAMES}"
            )
        self.directory = Path(directory)
        self.max_frames = max_frames
        self.frame_count = 0
        self.dropped_frames = 0

    def write(
        self,
        *,
        width: int,
        height: int,
        encoding: str,
        step: int,
        data: bytes,
    ) -> Path | None:
        """Write the next frame, or count it as dropped after the hard limit."""
        if self.frame_count >= self.max_frames:
            self.dropped_frames += 1
            return None
        sequence = self.frame_count + 1
        destination = self.directory / f"frame-{sequence:06d}.png"
        write_rgb_png_atomic(
            destination,
            width=width,
            height=height,
            encoding=encoding,
            step=step,
            data=data,
            compression_level=1,
        )
        self.frame_count = sequence
        return destination
