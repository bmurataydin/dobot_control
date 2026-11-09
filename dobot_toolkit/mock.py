"""
Donanımsız test için sahte (mock) Dobot arayüzü.

`MockInterface`, gerçek `Interface` ile aynı metotları sunar; böylece üst
seviye `Dobot` sınıfı HİÇ DEĞİŞMEDEN robot olmadan çalıştırılabilir:

    from dobot_toolkit import Dobot
    from dobot_toolkit.mock import MockInterface
    bot = Dobot(interface=MockInterface(realtime=True))

realtime=True  : hareketler hız parametresine göre gerçek zamanlı sürer
                 (bekleme, örnekleme, kamera senkronu gibi kodları test etmek için)
realtime=False : her hareket anında tamamlanır (birim testleri için hızlı)

Kuyruk semantiği firmware ile aynıdır: "mevcut indeks" = yürütülmekte olan
veya en son başlayan komutun indeksi.
"""
import threading
import time

import numpy as np

from .kinematics import DobotKinematics, UnreachableError

HOME_JOINTS = (0.0, 45.0, 45.0, 0.0)   # gerçek robotun home pozu: x=259.1, z=-8.5


class _MockSerial:
    is_open = True

    def reset_input_buffer(self):
        pass

    def close(self):
        self.is_open = False


class _Job:
    __slots__ = ("idx", "kind", "start", "end", "j_from", "j_to", "x_from", "x_to", "t0", "t1")

    def __init__(self, idx, kind="noop", duration=0.0):
        self.idx = idx
        self.kind = kind
        self.start = None      # planlanan başlangıç/bitiş (sn, monotonic)
        self.end = None
        self.j_from = self.j_to = None
        self.x_from = self.x_to = None
        self.t0 = 0.0
        self.t1 = duration


class MockInterface:
    def __init__(self, realtime: bool = False, kinematics: DobotKinematics = None,
                 time_scale: float = 1.0):
        self.serial = _MockSerial()
        self.kin = kinematics or DobotKinematics()
        self.realtime = realtime
        self.time_scale = time_scale
        self.lock = threading.RLock()

        self._idx = 0
        self._jobs = []                     # planlanmış işler (zaman sırasıyla)
        self._pending = []                  # kuyruk durdurulmuşken eklenenler
        self._running = True
        self._planned = np.array(HOME_JOINTS, dtype=float)   # son planlanan eklem durumu
        self._settled = np.array(HOME_JOINTS, dtype=float)   # biten işlerin eklem durumu
        self._settled_idx = 0
        self._alarms = set()
        self._ptp_speed = 50.0              # mm/s ve °/s (basit model)
        self._ratio = 50.0
        self.sent = []                      # gönderilen komutların kaydı (test için)
        self.effectors = {"suction": (0, 0), "gripper": (0, 0), "laser": (0, 0)}

    # ------------------------------------------------------------------ yardımcılar
    def _now(self):
        return time.monotonic()

    def _speed(self):
        return max(1.0, self._ptp_speed * self._ratio / 100.0) * self.time_scale

    def _fk(self, j):
        x, y, z = self.kin.forward(j[0], j[1], j[2])
        return np.array([x, y, z, j[0] + j[3]])     # r = j1 + j4 (Dobot sözleşmesi)

    def _ik(self, x, y, z, r):
        j1, j2, j3 = self.kin.inverse(x, y, z)
        return np.array([j1, j2, j3, r - j1])

    def _enqueue(self, job):
        self._idx += 1
        job.idx = self._idx
        if self._running:
            self._schedule(job)
        else:
            self._pending.append(job)
        return self._idx

    def _schedule(self, job):
        now = self._now()
        prev_end = self._jobs[-1].end if self._jobs else now
        job.start = max(now, prev_end)
        job.end = job.start + (job.t1 if self.realtime else 0.0)
        self._jobs.append(job)

    def _advance(self):
        """Bitmiş işleri 'settled' duruma taşır; yürütülen işi döndürür."""
        now = self._now()
        while self._jobs and self._jobs[0].end <= now:
            job = self._jobs.pop(0)
            if job.j_to is not None:
                self._settled = job.j_to.copy()
            self._settled_idx = job.idx
        if self._jobs and self._jobs[0].start <= now:
            return self._jobs[0], now
        return None, now

    def _current_joints(self):
        job, now = self._advance()
        if job is None or job.j_to is None:
            return self._settled.copy()
        span = max(job.end - job.start, 1e-9)
        s = min(max((now - job.start) / span, 0.0), 1.0)
        if job.kind == "linear":
            p = job.x_from + s * (job.x_to - job.x_from)
            try:
                return self._ik(*p)
            except UnreachableError:
                return job.j_from + s * (job.j_to - job.j_from)
        return job.j_from + s * (job.j_to - job.j_from)

    def _motion_job(self, target_joints, linear):
        job = _Job(0, "linear" if linear else "joint")
        job.j_from = self._planned.copy()
        job.j_to = np.array(target_joints, dtype=float)
        job.x_from, job.x_to = self._fk(job.j_from), self._fk(job.j_to)
        dist = float(np.linalg.norm(job.x_to[:3] - job.x_from[:3]))
        jdist = float(np.max(np.abs(job.j_to - job.j_from)))
        job.t1 = max(dist, jdist) / self._speed()
        self._planned = job.j_to.copy()
        return self._enqueue(job)

    def _noop(self, duration=0.0):
        return self._enqueue(_Job(0, "noop", duration))

    # ------------------------------------------------------------------ bağlantı
    def connected(self):
        return self.serial.is_open

    def close(self):
        self.serial.close()

    # ------------------------------------------------------------------ bilgi
    def get_device_name(self):
        return "MockDobot"

    def get_device_serial_number(self):
        return "MOCK-0000"

    def get_device_version(self):
        return (3, 7, 0)

    def get_device_time(self):
        return int(time.monotonic() * 1000) & 0xFFFFFFFF

    def get_pose(self):
        with self.lock:
            j = self._current_joints()
            p = self._fk(j)
            return tuple(float(v) for v in (*p, *j))

    # ------------------------------------------------------------------ alarm
    def get_alarms_state(self):
        b = [0] * 16
        for code in self._alarms:
            b[code // 8] |= 1 << (code % 8)
        return tuple(b)

    def clear_alarms_state(self):
        self._alarms.clear()
        return []

    def raise_alarm(self, code):
        """Test için: alarm oluştur."""
        self._alarms.add(code)

    # ------------------------------------------------------------------ parametreler
    def set_point_to_point_joint_params(self, velocity, acceleration, queue=True):
        self._ptp_speed = float(velocity[0])
        return self._noop()

    def set_point_to_point_coordinate_params(self, v, rv, a, ra, queue=True):
        self._ptp_speed = float(v)
        return self._noop()

    def set_point_to_point_common_params(self, velocity_ratio, acceleration_ratio, queue=True):
        self._ratio = float(velocity_ratio)
        return self._noop()

    def set_point_to_point_jump_params(self, jump_height, z_limit, queue=True):
        return self._noop()

    def set_continuous_trajectory_params(self, *args, queue=True):
        return self._noop()

    def set_end_effector_params(self, *args, queue=False):
        return []

    def set_homing_parameters(self, *args, queue=True):
        return self._noop()

    # ------------------------------------------------------------------ hareket
    def set_homing_command(self, command=0, queue=True):
        with self.lock:
            return self._motion_job(HOME_JOINTS, linear=False)

    def set_point_to_point_command(self, mode, x, y, z, r, queue=True):
        with self.lock:
            mode = int(mode)
            cur = self._planned
            try:
                if mode in (0, 1, 2, 9):                         # mutlak XYZ
                    target, linear = self._ik(x, y, z, r), mode == 2
                elif mode in (3, 4, 5):                          # mutlak açı
                    target, linear = np.array([x, y, z, r], float), mode == 5
                elif mode == 6:                                  # eklem artışı
                    target, linear = cur + np.array([x, y, z, r]), False
                elif mode in (7, 8):                             # kartezyen artış
                    p = self._fk(cur) + np.array([x, y, z, r])
                    target, linear = self._ik(*p), mode == 7
                else:
                    raise ValueError(f"Geçersiz PTP modu: {mode}")
            except UnreachableError:
                self._alarms.add(0x11)
                return self._noop()
            return self._motion_job(target, linear)

    def set_continuous_trajectory_command(self, mode, x, y, z, velocity, queue=True):
        with self.lock:
            cur = self._fk(self._planned)
            p = np.array([x, y, z, cur[3]]) if mode == 1 else cur + np.array([x, y, z, 0.0])
            try:
                target = self._ik(*p)
            except UnreachableError:
                self._alarms.add(0x11)
                return self._noop()
            old = self._ptp_speed, self._ratio
            self._ptp_speed, self._ratio = float(velocity), 100.0
            idx = self._motion_job(target, linear=True)
            self._ptp_speed, self._ratio = old
            return idx

    def set_arc_command(self, via, end, queue=True):
        with self.lock:
            try:
                target = self._ik(*end[:4])
            except UnreachableError:
                self._alarms.add(0x11)
                return self._noop()
            return self._motion_job(target, linear=False)

    # orijinal lib/interface.py yazımı ("continous") ile aynı adlar
    set_continous_trajectory_params = set_continuous_trajectory_params
    set_continous_trajectory_command = set_continuous_trajectory_command

    def set_jog_command(self, jog_type, command, queue=False):
        return []

    def wait(self, milliseconds, queue=True):
        with self.lock:
            return self._noop(milliseconds / 1000.0)

    # ------------------------------------------------------------------ uç efektör
    def set_end_effector_suction_cup(self, enable_control, enable_suction, queue=True):
        self.effectors["suction"] = (int(enable_control), int(enable_suction))
        return self._noop()

    def set_end_effector_gripper(self, enable_control, enable_grip, queue=True):
        self.effectors["gripper"] = (int(enable_control), int(enable_grip))
        return self._noop()

    def set_end_effector_laser(self, enable_control, enable_laser, queue=True):
        self.effectors["laser"] = (int(enable_control), int(enable_laser))
        return self._noop()

    # ------------------------------------------------------------------ IO
    def set_io_multiplexing(self, address, multiplex, queue=True):
        return self._noop()

    def set_io_do(self, address, level, queue=True):
        return self._noop()

    def get_io_di(self, address):
        return (address, 0)

    def get_io_adc(self, address):
        return (address, 0)

    # ------------------------------------------------------------------ kuyruk
    def start_queue(self):
        with self.lock:
            self._running = True
            for job in self._pending:
                self._schedule(job)
            self._pending.clear()
            return []

    def stop_queue(self, force=False):
        with self.lock:
            if force:
                j = self._current_joints()
                self._jobs.clear()
                self._pending.clear()
                self._settled = self._planned = j
                self._settled_idx = self._idx
            self._running = False
            return []

    def clear_queue(self):
        with self.lock:
            self._pending.clear()
            return []

    def get_current_queue_index(self):
        with self.lock:
            job, _ = self._advance()
            return job.idx if job is not None else self._settled_idx
