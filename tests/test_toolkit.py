"""
Donanım gerektirmeyen birim testleri (sahte robot ile).

    python -m pytest tests -v
    python -m unittest discover tests -v
"""
import csv
import os
import struct
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from dobot_toolkit import (CsvLogger, DobotAlarmError, DobotKinematics, PoseSampler, PTPMode,
                           UnreachableError, Workspace, WorkspaceError, connect, decode_alarms,
                           trajectories as tr)
from lib.message import Message


class TestProtocol(unittest.TestCase):
    def test_ptp_packet_layout_and_checksum(self):
        msg = Message([0xAA, 0xAA], 2, 84, True, True, [int(PTPMode.MOVJ_XYZ), 200.0, 0.0, 10.0, 0.0],
                      direction="out")
        pkt = msg.package()
        self.assertEqual(pkt[:2], b"\xaa\xaa")
        self.assertEqual(pkt[2], 2 + 17)                 # ID + ctrl + (1 + 4*4) bayt
        self.assertEqual(pkt[3], 84)
        self.assertEqual(pkt[4], 0b11)                   # rw=1, queued=1
        self.assertEqual(sum(pkt[3:]) % 256, 0)          # checksum doğru
        self.assertEqual(struct.unpack("<Bffff", pkt[5:-1])[1], 200.0)

    def test_parse_get_pose_response(self):
        vals = [259.1, 0.0, -8.5, 90.0, 0.0, 45.0, 45.0, 90.0]
        params = list(struct.pack("<8f", *vals))
        body = [10, 0] + params
        pkt = bytes([0xAA, 0xAA, len(body)] + body + [Message.calculate_checksum(body)])
        resp = Message.parse(pkt)
        self.assertIsNotNone(resp)
        np.testing.assert_allclose(resp.params, vals, atol=1e-4)

    def test_bad_checksum_rejected(self):
        body = [10, 0] + [0] * 32
        pkt = bytes([0xAA, 0xAA, len(body)] + body + [123])
        self.assertIsNone(Message.parse(pkt))


class TestKinematics(unittest.TestCase):
    def test_home_pose_matches_robot(self):
        # Gerçek robot home: j2=j3=45° -> x=259.1, z=-8.5
        x, y, z = DobotKinematics().forward(0, 45, 45)
        self.assertAlmostEqual(x, 259.1, delta=0.1)
        self.assertAlmostEqual(z, -8.5, delta=0.1)

    def test_roundtrip(self):
        kin = DobotKinematics()
        for p in [(220, 0, 20), (200, -80, 0), (250, 60, -50), (180, 30, 60)]:
            q = kin.inverse(*p)
            np.testing.assert_allclose(kin.forward(*q), p, atol=1e-6)

    def test_unreachable(self):
        with self.assertRaises(UnreachableError):
            DobotKinematics().inverse(600, 0, 0)


class TestSafety(unittest.TestCase):
    def test_workspace(self):
        ws = Workspace()
        self.assertTrue(ws.contains(220, 0, 0))
        self.assertFalse(ws.contains(400, 0, 0))
        with self.assertRaises(WorkspaceError):
            ws.check(50, 0, 0)
        x, y, z, clipped = ws.clamp(400, 0, -200)
        self.assertTrue(clipped)
        self.assertAlmostEqual(np.hypot(x, y), ws.r_max)
        self.assertEqual(z, ws.z_min)

    def test_alarm_decode(self):
        b = [0] * 16
        b[8] = 0b00000001     # 0x40
        b[2] = 0b00000010     # 0x11
        self.assertEqual(decode_alarms(b), [0x11, 0x40])


class TestMockRobot(unittest.TestCase):
    def setUp(self):
        self.bot = connect(dry_run=True, realtime=False)

    def test_moves(self):
        b = self.bot
        b.home()
        b.move_xyz(220, 0, 20)
        np.testing.assert_allclose(b.get_pose().xyz, (220, 0, 20), atol=1e-6)
        b.move_relative(dz=10)
        np.testing.assert_allclose(b.get_pose().xyz, (220, 0, 30), atol=1e-6)
        b.move_joints(10, 30, 40)
        self.assertAlmostEqual(b.get_pose().j1, 10)
        b.move_joints_relative(dj1=-10)
        self.assertAlmostEqual(b.get_pose().j1, 0)

    def test_follow_path(self):
        pts = tr.circle((220, 0), 20, 0, 50)
        self.bot.follow_path(pts, speed=50)
        np.testing.assert_allclose(self.bot.get_pose().xyz, pts[-1], atol=1e-6)

    def test_workspace_blocks_move(self):
        with self.assertRaises(WorkspaceError):
            self.bot.move_xyz(400, 0, 0)

    def test_alarm_raises_in_wait(self):
        bot = connect(dry_run=True, realtime=True, wait_timeout=5)
        bot.interface.raise_alarm(0x40)
        bot.interface.stop_queue()          # kuyruk ilerlemesin (alarmda olduğu gibi)
        with self.assertRaises(DobotAlarmError):
            bot.move_xyz(220, 0, 20)

    def test_sampler_and_logger(self):
        bot = connect(dry_run=True, realtime=True, speed=200, speed_ratio=100)
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "s.csv")
            with PoseSampler(bot, hz=20, csv_path=path) as s:
                bot.move_xyz(220, 50, 20)
            self.assertGreater(len(s.samples), 3)
            with open(path, encoding="utf-8") as f:
                rows = list(csv.DictReader(f))
            self.assertEqual(len(rows), len(s.samples))
            self.assertIn("vx", rows[0])

    def test_csv_logger_velocity(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "l.csv")
            with CsvLogger(path, extra_fields=["step"]) as log:
                log.log([0, 0, 0, 0, 0, 0, 0, 0], t=100.0, step=0)
                log.log([10, 0, 0, 0, 0, 0, 0, 0], t=102.0, step=1)
            with open(path, encoding="utf-8") as f:
                rows = list(csv.DictReader(f))
            self.assertAlmostEqual(float(rows[1]["vx"]), 5.0)
            self.assertAlmostEqual(float(rows[1]["t_rel"]), 2.0)


class TestTrajectories(unittest.TestCase):
    def test_shapes(self):
        for name in tr.SHAPES:
            pts = tr.make_shape(name, (220, 0), 30, -50, 80)
            self.assertEqual(pts.shape, (80, 3))
            self.assertTrue(np.all(pts[:, 2] == -50))

    def test_resample_spacing(self):
        pts = tr.resample(tr.line((0, 0, 0), (100, 0, 0), 3), 10)
        self.assertEqual(len(pts), 11)


if __name__ == "__main__":
    unittest.main()
