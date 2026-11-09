"""
01 — Bağlan, cihaz bilgisini ve pozu oku (robot HAREKET ETMEZ).

İlk kurulumda bağlantıyı test etmek için bunu çalıştırın:
    python examples/01_connect_read_pose.py --port COM3
    python examples/01_connect_read_pose.py --dry-run

Robotu elle hareket ettirirken (koldaki kilit açma düğmesine basılı tutarak)
poz değerlerinin değiştiğini canlı izleyebilirsiniz: --watch
"""
import time

from _common import base_parser, get_robot
from dobot_toolkit.ports import list_serial_ports


def main():
    p = base_parser(__doc__)
    p.add_argument("--watch", action="store_true", help="Pozu 10 Hz ile sürekli yazdır (Ctrl+C ile çık).")
    args = p.parse_args()

    print("Sistemdeki seri portlar:")
    for dev, desc, _ in list_serial_ports():
        print(f"   {dev:10s} {desc}")

    bot = get_robot(args, home=False)       # bu örnek robotu hareket ettirmez
    try:
        for k, v in bot.info().items():
            print(f"   {k:14s}: {v}")
        print(f"   alarmlar      : {bot.describe_alarms()}")
        print(f"   kuyruk indeksi: {bot.current_queue_index()}")

        if args.watch:
            print("\nCanlı poz (Ctrl+C ile çıkış):")
            while True:
                print(f"\r{bot.get_pose()}", end="", flush=True)
                time.sleep(0.1)
    except KeyboardInterrupt:
        print()
    finally:
        bot.close()


if __name__ == "__main__":
    main()
