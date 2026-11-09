"""
Örneklerin ortak yardımcıları: komut satırı argümanları ve robota bağlanma.

Her örnek şu argümanları kabul eder:
    --port COM3        seri port (verilmezse otomatik bulunur)
    --dry-run          robot olmadan sahte robotla çalıştır
    --speed 50         hız parametresi
    --no-home          başta home hareketi yapma
"""
import argparse
import os
import sys
from datetime import datetime

# Paket pip ile kurulmamışsa repo kökünü yola ekle
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from dobot_toolkit import connect  # noqa: E402


def base_parser(description: str) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=description)
    p.add_argument("--port", default=None, help="Seri port (ör. COM3, /dev/ttyUSB0). Boş: otomatik.")
    p.add_argument("--dry-run", action="store_true", help="Robot olmadan sahte robotla çalıştır.")
    p.add_argument("--speed", type=float, default=50.0, help="Hız parametresi (varsayılan 50).")
    p.add_argument("--ratio", type=float, default=50.0, help="Hız oranı %% (0-100, varsayılan 50).")
    p.add_argument("--no-home", action="store_true", help="Başlangıçta home yapma.")
    return p


def get_robot(args, home: bool = True):
    mode = "SAHTE ROBOT (dry-run)" if args.dry_run else f"port={args.port or 'otomatik'}"
    print(f"-> Bağlanılıyor: {mode}")
    bot = connect(port=args.port, dry_run=args.dry_run, speed=args.speed, speed_ratio=args.ratio)
    print(f"-> Bağlandı ({bot.port}). Alarmlar: {bot.describe_alarms()}")
    if home and not args.no_home:
        print("-> Home hareketi (birkaç saniye sürebilir)...")
        bot.home()
    print(f"-> Poz: {bot.get_pose()}")
    return bot


def session_dir(root: str = "data", name: str = "session") -> str:
    """data/<name>_YYYYmmdd_HHMMSS klasörünü oluşturur."""
    path = os.path.join(root, f"{name}_{datetime.now():%Y%m%d_%H%M%S}")
    os.makedirs(path, exist_ok=True)
    return path


def confirm(msg: str, args) -> bool:
    """Gerçek robotta hareketten önce onay ister (dry-run'da otomatik evet)."""
    if args.dry_run:
        return True
    return input(f"{msg} [e/H]: ").strip().lower() in ("e", "evet", "y", "yes")
