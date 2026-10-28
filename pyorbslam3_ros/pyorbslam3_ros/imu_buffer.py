"""Thread-safe IMU buffer feeding ORB-SLAM3's per-frame IMU windows."""

from collections import deque
import threading
from typing import Optional, Tuple

import numpy as np

ImuRow = Tuple[float, float, float, float, float, float, float]  # t, gx, gy, gz, ax, ay, az


class ImuBuffer:
    """Time-ordered IMU samples, trimmed to `max_duration` seconds of history.

    `add` runs on the IMU callback thread; `wait_for` / `take_window` run on
    the tracking thread. Samples that arrive out of order are dropped (and
    counted) because ORB-SLAM3's preintegration assumes monotonic time.
    """

    def __init__(self, max_duration: float = 5.0):
        self.max_duration = float(max_duration)
        self._rows: deque = deque()
        self._cond = threading.Condition()
        self.dropped_out_of_order = 0
        self.trimmed_until: Optional[float] = None  # newest timestamp discarded by trimming

    def add(self, row: ImuRow) -> bool:
        with self._cond:
            if self._rows and row[0] <= self._rows[-1][0]:
                self.dropped_out_of_order += 1
                return False
            self._rows.append(tuple(row))
            newest = row[0]
            while self._rows and newest - self._rows[0][0] > self.max_duration:
                self.trimmed_until = self._rows.popleft()[0]
            self._cond.notify_all()
            return True

    def __len__(self) -> int:
        with self._cond:
            return len(self._rows)

    def span(self) -> Tuple[Optional[float], Optional[float]]:
        """(oldest, newest) timestamps, or (None, None) when empty."""
        with self._cond:
            if not self._rows:
                return None, None
            return self._rows[0][0], self._rows[-1][0]

    def wait_for(self, t: float, timeout: float) -> bool:
        """Block until a sample with timestamp >= t is buffered, or timeout."""
        with self._cond:
            return self._cond.wait_for(
                lambda: bool(self._rows) and self._rows[-1][0] >= t, timeout=max(0.0, timeout))

    def take_window(self, t_from: Optional[float], t_to: float) -> np.ndarray:
        """Pop samples with t_from < t <= t_to as an (N, 7) float64 array.

        Samples at or before t_from are discarded; samples after t_to stay
        buffered for the next frame. t_from=None keeps everything up to t_to.
        """
        rows = []
        with self._cond:
            while self._rows and self._rows[0][0] <= t_to:
                r = self._rows.popleft()
                if t_from is None or r[0] > t_from:
                    rows.append(r)
        if not rows:
            return np.empty((0, 7), dtype=np.float64)
        return np.asarray(rows, dtype=np.float64)

    def clear(self) -> None:
        with self._cond:
            self._rows.clear()
            self.trimmed_until = None
