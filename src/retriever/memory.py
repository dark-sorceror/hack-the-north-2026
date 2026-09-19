"""What the robot remembers: objects it was taught, and where it last saw things.

SQLite because it ships with Python, survives a crash mid-write, and keeps a whole run's
memory in one file that is easy to inspect. People ask "where are my keys?" long after the
keys left the camera's view, so `last_seen` answers from the newest sighting, and None is
a real answer ("I have not seen them") rather than an error to paper over. Labels come
from people and from an LLM, which never agree on capital letters, so every lookup is
case-insensitive. Embeddings are stored as little-endian float32: that is all the
precision an embedding carries, and the file reads the same on the laptop and the Pi.
"""

from __future__ import annotations

import math
import sqlite3
import struct
import time
import uuid
from dataclasses import dataclass
from pathlib import Path

from retriever.types import Pose

_SCHEMA = """
CREATE TABLE IF NOT EXISTS taught_objects (
    id TEXT PRIMARY KEY,
    name TEXT NOT NULL UNIQUE COLLATE NOCASE,
    embedding BLOB NOT NULL,
    taught_at REAL NOT NULL
);
CREATE TABLE IF NOT EXISTS sightings (
    label TEXT NOT NULL COLLATE NOCASE,
    x REAL NOT NULL,
    y REAL NOT NULL,
    theta REAL NOT NULL,
    confidence REAL NOT NULL,
    object_id TEXT,
    seen_at REAL NOT NULL
);
CREATE INDEX IF NOT EXISTS sightings_by_label ON sightings (label, seen_at);
"""

_SIGHTING_COLUMNS = "label, x, y, theta, confidence, seen_at, object_id"


@dataclass(frozen=True)
class Sighting:
    """One time the robot saw something, and where it was."""

    label: str
    pose: Pose
    confidence: float
    seen_at: float  # unix seconds
    object_id: str | None = None

    def age_s(self, now: float | None = None) -> float:
        """Seconds since the sighting; `now` defaults to the wall clock."""
        return (time.time() if now is None else now) - self.seen_at


class Memory:
    """Taught objects and sightings in one SQLite database (a file, or RAM by default)."""

    def __init__(self, path: Path | str = ":memory:") -> None:
        if str(path) != ":memory:":
            Path(path).parent.mkdir(parents=True, exist_ok=True)
        self._db = sqlite3.connect(str(path))
        self._db.executescript(_SCHEMA)

    def teach(self, name: str, embedding: list[float]) -> str:
        """Remember an object by name and return its id; re-teaching a name replaces it."""
        name = name.strip()
        if not name:
            raise ValueError("a taught object needs a name")
        if not embedding or not all(math.isfinite(v) for v in embedding):
            raise ValueError(f"embedding for {name!r} must be non-empty and finite")
        blob = struct.pack(f"<{len(embedding)}f", *embedding)
        with self._db:
            row = self._db.execute(
                "SELECT id FROM taught_objects WHERE name = ?", (name,)
            ).fetchone()
            # Keep the id across re-teaching so earlier sightings still point at the object.
            object_id = row[0] if row else uuid.uuid4().hex
            self._db.execute(
                "INSERT OR REPLACE INTO taught_objects (id, name, embedding, taught_at)"
                " VALUES (?, ?, ?, ?)",
                (object_id, name, blob, time.time()),
            )
        return object_id

    def taught(self) -> list[tuple[str, str, list[float]]]:
        """Every taught object as (id, name, embedding), ordered by name."""
        rows = self._db.execute(
            "SELECT id, name, embedding FROM taught_objects ORDER BY name"
        ).fetchall()
        return [
            (object_id, name, list(struct.unpack(f"<{len(blob) // 4}f", blob)))
            for object_id, name, blob in rows
        ]

    def saw(
        self,
        label: str,
        pose: Pose,
        confidence: float,
        object_id: str | None = None,
        at: float | None = None,
    ) -> None:
        """Record that `label` was seen at `pose`; `at` defaults to now (unix seconds)."""
        label = label.strip()
        if not label:
            raise ValueError("a sighting needs a label")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError(f"confidence must be within [0, 1], got {confidence!r}")
        seen_at = time.time() if at is None else at
        with self._db:
            self._db.execute(
                f"INSERT INTO sightings ({_SIGHTING_COLUMNS}) VALUES (?, ?, ?, ?, ?, ?, ?)",
                (label, pose.x, pose.y, pose.theta, confidence, seen_at, object_id),
            )

    def last_seen(self, label: str) -> Sighting | None:
        """The most recent sighting of `label`, or None if it has never been seen."""
        row = self._db.execute(
            f"SELECT {_SIGHTING_COLUMNS} FROM sightings WHERE label = ?"
            " ORDER BY seen_at DESC, rowid DESC LIMIT 1",
            (label.strip(),),
        ).fetchone()
        return _sighting(row) if row else None

    def summary(self, limit: int = 12) -> str:
        """A compact digest for an LLM prompt: the newest sighting per label, newest first."""
        if limit < 1:
            raise ValueError(f"limit must be at least 1, got {limit}")
        rows = self._db.execute(
            f"SELECT {_SIGHTING_COLUMNS} FROM ("
            " SELECT *, ROW_NUMBER() OVER"
            " (PARTITION BY label ORDER BY seen_at DESC, rowid DESC) AS recency"
            " FROM sightings"
            ") WHERE recency = 1 ORDER BY seen_at DESC LIMIT ?",
            (limit,),
        ).fetchall()
        if not rows:
            return "Memory is empty: nothing has been seen yet."
        now = time.time()
        return "\n".join(
            f"{s.label}: ({s.pose.x:.2f}, {s.pose.y:.2f}) {_human_age(s.age_s(now))},"
            f" conf {s.confidence:.2f}"
            for s in map(_sighting, rows)
        )

    def close(self) -> None:
        """Close the database; safe to call twice."""
        self._db.close()


def _sighting(row: tuple) -> Sighting:
    label, x, y, theta, confidence, seen_at, object_id = row
    return Sighting(label, Pose(x, y, theta), confidence, seen_at, object_id)


def _human_age(seconds: float) -> str:
    if seconds < 10:
        return "just now"
    if seconds < 60:
        return f"{int(seconds)} s ago"
    if seconds < 3600:
        return f"{int(seconds // 60)} min ago"
    if seconds < 86400:
        return f"{int(seconds // 3600)} h ago"
    return f"{int(seconds // 86400)} d ago"
