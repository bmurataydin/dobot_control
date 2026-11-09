"""
Veri kaydı: CSV kaydedici ve arka planda poz örnekleyici.

CsvLogger  : her satırda zaman damgası + poz + (önceki satıra göre) hız.
             Her satır anında diske yazılır (flush) — program çökse bile veri kalır.
PoseSampler: ayrı bir thread'de get_pose()'u sabit frekansta okur; sürekli
             (CP) hareketler sırasında yüksek çözünürlüklü yörünge kaydı için.

    with CsvLogger("data/run1.csv", extra_fields=["step", "target_x"]) as log:
        log.log(bot.get_pose(), step=0, target_x=220)

    with PoseSampler(bot, hz=30, csv_path="data/stream.csv"):
        bot.follow_path(points)        # hareket boyunca 30 Hz poz kaydı
"""
import csv
import os
import threading
import time
from typing import Iterable, Optional

POSE_FIELDS = ["x", "y", "z", "r", "j1", "j2", "j3", "j4"]
VEL_FIELDS = ["vx", "vy", "vz", "vr", "vj1", "vj2", "vj3", "vj4"]
BASE_FIELDS = ["t_unix", "t_rel"] + POSE_FIELDS + VEL_FIELDS


class CsvLogger:
    def __init__(self, path: str, extra_fields: Iterable[str] = (), compute_velocity: bool = True):
        self.path = path
        self.extra_fields = list(extra_fields)
        self.fields = BASE_FIELDS + self.extra_fields
        self.compute_velocity = compute_velocity
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self._f = open(path, "w", newline="", encoding="utf-8")
        self._w = csv.DictWriter(self._f, fieldnames=self.fields, extrasaction="ignore")
        self._w.writeheader()
        self._t0 = None
        self._prev = None        # (t, pose)
        self.rows = 0
        self._lock = threading.Lock()

    def log(self, pose, t: Optional[float] = None, **extra):
        """pose: Pose veya 8 elemanlı dizi. t: zaman damgası (varsayılan time.time())."""
        t = time.time() if t is None else t
        vals = [float(v) for v in pose[:8]]
        with self._lock:
            if self._t0 is None:
                self._t0 = t
            row = {"t_unix": f"{t:.6f}", "t_rel": f"{t - self._t0:.6f}"}
            row.update({k: f"{v:.4f}" for k, v in zip(POSE_FIELDS, vals)})
            if self.compute_velocity and self._prev is not None and t > self._prev[0]:
                dt = t - self._prev[0]
                row.update({k: f"{(v - pv) / dt:.4f}"
                            for k, v, pv in zip(VEL_FIELDS, vals, self._prev[1])})
            else:
                row.update({k: "0.0000" for k in VEL_FIELDS})
            row.update(extra)
            self._w.writerow(row)
            self._f.flush()
            self._prev = (t, vals)
            self.rows += 1

    def close(self):
        if not self._f.closed:
            self._f.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


class PoseSampler:
    """
    Arka planda sabit frekansta poz okur. Gerçek seri hatta ulaşılabilen hız
    tipik olarak ~20-50 Hz'dir (her get_pose bir istek/yanıt döngüsüdür).
    """

    def __init__(self, bot, hz: float = 20.0, csv_path: Optional[str] = None,
                 extra: Optional[dict] = None):
        self.bot = bot
        self.period = 1.0 / float(hz)
        self.samples = []                 # [(t_unix, Pose), ...]
        self.logger = CsvLogger(csv_path, extra_fields=list((extra or {}).keys())) if csv_path else None
        self.extra = extra or {}
        self.errors = 0
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="PoseSampler", daemon=True)
        self._thread.start()
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self.logger is not None:
            self.logger.close()

    def _run(self):
        next_t = time.monotonic()
        while not self._stop.is_set():
            try:
                pose = self.bot.get_pose()
                t = time.time()
                self.samples.append((t, pose))
                if self.logger is not None:
                    self.logger.log(pose, t=t, **self.extra)
            except Exception:
                self.errors += 1
            next_t += self.period
            delay = next_t - time.monotonic()
            if delay > 0:
                self._stop.wait(delay)
            else:
                next_t = time.monotonic()      # gecikmeyi biriktirme

    @property
    def achieved_hz(self) -> float:
        if len(self.samples) < 2:
            return 0.0
        span = self.samples[-1][0] - self.samples[0][0]
        return (len(self.samples) - 1) / span if span > 0 else 0.0

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()
        return False
