"""Data root and exclusive name claims (the Perception / Control pattern; CM-7).

Root: explicit -> ``$BLUEBOAT_DATA_DIR`` -> parent of the first ``$COLCON_PREFIX_PATH`` entry
(normally ``~/ros2_ws``) -> the working directory. Belief writes only under
``<root>/data/Belief_data/``; names are claimed with ``O_EXCL`` and suffixed ``-2``, ``-3``…
"""

from __future__ import annotations

import os
from pathlib import Path


def data_root(explicit: str = "") -> Path:
    if explicit:
        return Path(explicit).expanduser()
    env = os.environ.get("BLUEBOAT_DATA_DIR", "")
    if env:
        return Path(env).expanduser()
    prefix = os.environ.get("COLCON_PREFIX_PATH", "")
    if prefix:
        return Path(prefix.split(os.pathsep)[0]).expanduser().resolve().parent
    return Path.cwd()


def belief_dir(explicit: str = "") -> Path:
    return data_root(explicit) / "data" / "Belief_data"


def claim_dir(parent: Path, stem: str) -> Path:
    """Create ``parent/stem`` (or ``stem-2``, ``stem-3``…) exclusively and return it."""
    parent.mkdir(parents=True, exist_ok=True)
    k = 1
    while True:
        p = parent / (stem if k == 1 else f"{stem}-{k}")
        try:
            p.mkdir()
            return p
        except FileExistsError:
            k += 1


def claim_file(parent: Path, stem: str, suffix: str) -> Path:
    """Create an empty ``parent/stem[-k]suffix`` with O_EXCL and return its path."""
    parent.mkdir(parents=True, exist_ok=True)
    k = 1
    while True:
        p = parent / (f"{stem}{suffix}" if k == 1 else f"{stem}-{k}{suffix}")
        try:
            fd = os.open(p, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
            os.close(fd)
            return p
        except FileExistsError:
            k += 1
