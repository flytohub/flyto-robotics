"""Durable, atomic, mode-correct file writes.

Two modules need this (the lifecycle state/units and the support bundle) and a
third will. Reaching into another module's private ``_atomic_write`` is how a
refactor turns into a ``NameError`` on a customer's device, so the primitive
lives here, public and tested.

Two properties, both load-bearing:

* **Atomic.** Write a temporary file, ``fsync`` it, ``os.replace`` it into
  place, then ``fsync`` the parent directory. A reader sees the old bytes or the
  new bytes, never a truncated file, and a power cut cannot lose a rename whose
  contents were already flushed. Directory ``fsync`` is the step people skip;
  without it "atomic" only means "atomic until the machine loses power".
* **Mode-correct from birth.** The file is created with its final permissions.
  Writing 0644 and chmod-ing to 0600 afterwards leaves a window in which a file
  that may hold secrets is world readable, and that window is precisely when a
  support tool is running in a shared directory.
"""

from __future__ import annotations

import os
from pathlib import Path

__all__ = ["atomic_write", "fsync_dir", "fsync_path"]


def fsync_path(path: Path, *, directory: bool = False) -> None:
    """``fsync`` a file or directory, tolerating filesystems that cannot.

    Best effort by design: tmpfs and some network filesystems refuse to sync a
    directory handle. Refusing to write at all on those would break the test
    suite and container images for no safety gain, so the sync is attempted and
    a refusal is not fatal.
    """

    flags = os.O_RDONLY | (getattr(os, "O_DIRECTORY", 0) if directory else 0)
    try:
        handle = os.open(path, flags)
    except OSError:
        return
    try:
        os.fsync(handle)
    except OSError:
        pass
    finally:
        os.close(handle)


def fsync_dir(path: Path) -> None:
    fsync_path(path, directory=True)


def atomic_write(path: Path, text: str, mode: int = 0o644) -> Path:
    """Write ``text`` to ``path`` durably and with exactly ``mode``."""

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, mode)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    # umask can clear bits that O_CREAT requested, so the mode is asserted
    # rather than assumed. A 0600 bundle that lands at 0644 is a silent leak.
    os.chmod(temporary, mode)
    os.replace(temporary, path)
    fsync_dir(path.parent)
    return path
