"""
dobot_toolkit — Dobot Magician için Python kontrol, veri toplama ve test araçları.

Hızlı başlangıç:
    from dobot_toolkit import connect
    bot = connect()                 # gerçek robot (port otomatik)
    bot = connect(dry_run=True)     # robot olmadan sahte robot
"""
from .alarms import ALARM_CODES, decode_alarms, describe_alarm, format_alarms
from .dobot import Dobot, DobotAlarmError, DobotTimeoutError, Pose, PTPMode
from .kinematics import DobotKinematics, UnreachableError
from .logger import CsvLogger, PoseSampler
from lib.interface import Interface   # orijinal, sahada test edilmiş protokol katmanı
from .safety import JOINT_LIMITS, Workspace, WorkspaceError
from . import trajectories

__version__ = "0.1.0"


def connect(port=None, dry_run=False, realtime=True, **kwargs) -> Dobot:
    """
    Robota bağlanır ve kullanıma hazır bir Dobot nesnesi döndürür.

    port    : 'COM3', '/dev/ttyUSB0' ... None -> otomatik bul
    dry_run : True ise robot yerine MockInterface kullanılır (donanım gerekmez)
    realtime: dry_run'da hareketlerin gerçek zamanlı sürmesi
    kwargs  : Dobot(...) parametreleri (speed, workspace, wait_timeout ...)
    """
    if dry_run:
        from .mock import MockInterface
        return Dobot(port="MOCK", interface=MockInterface(realtime=realtime), **kwargs)
    return Dobot(port, **kwargs)


__all__ = [
    "connect", "Dobot", "Pose", "PTPMode", "DobotAlarmError", "DobotTimeoutError",
    "Interface", "DobotKinematics", "UnreachableError",
    "Workspace", "WorkspaceError", "JOINT_LIMITS", "CsvLogger", "PoseSampler",
    "ALARM_CODES", "decode_alarms", "describe_alarm", "format_alarms", "trajectories",
]
