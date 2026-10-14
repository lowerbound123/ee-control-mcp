from __future__ import annotations

import csv
import json
import logging
import threading
from collections.abc import Iterable
from datetime import datetime, timedelta
from pathlib import Path

from .observations import EncodedImage

logger = logging.getLogger(__name__)

_TIMESTAMP_FORMAT = "%Y%m%d-%H%M%S-%f"
_CAMERA_NAMES = {"head": "head", "left_wrist": "left", "right_wrist": "right"}
_VECTORS = ("qpos", "qvel", "effort", "eef")
_STATE_FIELDS = (
    "logged_at",
    "timestamp",
    "seq",
    "state_ready",
    "left_seq",
    "right_seq",
    "left_timestamp",
    "right_timestamp",
    *(f"{name}_{index}" for name in _VECTORS for index in range(14)),
    "state_json",
)


class ObservationRecorder:
    """Save requested observations once per receipt sequence, shared by all MCP clients."""

    def __init__(self):
        self._root = Path.cwd() / "logs"
        self._started_at = datetime.now().astimezone()
        self.directory = self._root / f"system-{self._started_at.strftime(_TIMESTAMP_FORMAT)}"
        self._directory_created = False
        self._lock = threading.Lock()
        self._saved_states: set[int] = set()
        self._saved_frames: set[tuple[str, int]] = set()
        self._last_logged_at: datetime | None = None

    def _ensure_directory(self) -> None:
        while not self._directory_created:
            try:
                self.directory.mkdir(parents=True)
            except FileExistsError:
                # Even simultaneous server instances must keep separate run directories.
                self._started_at += timedelta(microseconds=1)
                self.directory = self._root / (
                    f"system-{self._started_at.strftime(_TIMESTAMP_FORMAT)}"
                )
            else:
                self._directory_created = True

    def _next_timestamp(self) -> datetime:
        now = datetime.now().astimezone()
        if self._last_logged_at is not None and now <= self._last_logged_at:
            now = self._last_logged_at + timedelta(microseconds=1)
        self._last_logged_at = now
        return now

    def _append_state(self, payload: dict, logged_at: datetime) -> None:
        row = {
            "logged_at": logged_at.isoformat(timespec="microseconds"),
            "timestamp": payload["timestamp"],
            "seq": payload["seq"],
            "state_ready": payload["state_ready"],
            "state_json": json.dumps(payload, ensure_ascii=False, allow_nan=False),
        }
        for arm in ("left", "right"):
            observation = payload["arms"][arm]
            row[f"{arm}_seq"] = observation.get("seq")
            row[f"{arm}_timestamp"] = observation["timestamp"]
        for name in _VECTORS:
            values = payload[name]
            row.update(
                {
                    f"{name}_{index}": None if values is None else values[index]
                    for index in range(14)
                }
            )
        with (self.directory / "state.csv").open("a", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=_STATE_FIELDS)
            if output.tell() == 0:
                writer.writeheader()
            writer.writerow(row)

    def record(self, payload: dict, images: Iterable[EncodedImage] = ()) -> dict[str, str]:
        errors: dict[str, str] = {}

        def report_error(source, exc):
            errors[source] = str(exc)
            logger.warning("Could not save %s in %s: %s", source, self.directory, exc)

        with self._lock:
            seq = payload.get("seq")
            save_state = seq is not None and seq not in self._saved_states
            metadata = {item["camera"]: item for item in payload.get("images", [])}
            pending_images = [
                (frame, (frame.name, metadata[frame.name]["seq"]))
                for frame in images
                if (frame.name, metadata[frame.name]["seq"]) not in self._saved_frames
            ]
            if not save_state and not pending_images:
                return errors
            try:
                self._ensure_directory()
            except OSError as exc:
                report_error("directory", exc)
                return errors
            logged_at = self._next_timestamp()
            if save_state:
                try:
                    self._append_state(payload, logged_at)
                except OSError as exc:
                    report_error("state", exc)
                else:
                    self._saved_states.add(seq)
            for frame, identity in pending_images:
                name = _CAMERA_NAMES[frame.name]
                path = self.directory / f"{name}-{logged_at.strftime(_TIMESTAMP_FORMAT)}.jpg"
                try:
                    path.write_bytes(frame.data)
                except OSError as exc:
                    report_error(frame.name, exc)
                else:
                    self._saved_frames.add(identity)
        return errors
