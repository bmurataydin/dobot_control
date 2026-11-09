"""
04 — Uç efektörler: vakum (suction cup), pnömatik tutucu (gripper), lazer.

Basit al-bırak (pick & place) senaryosu: A noktasındaki nesneyi vakumla alıp
B noktasına bırakır. Z değerlerini nesne yüksekliğinize göre ayarlayın!

    python examples/04_end_effectors.py --port COM3 --tool suction --pick-z -45
"""
import time

from _common import base_parser, confirm, get_robot


def main():
    p = base_parser(__doc__)
    p.add_argument("--tool", choices=["suction", "gripper", "laser"], default="suction")
    p.add_argument("--pick", type=float, nargs=2, default=[220, -60], metavar=("X", "Y"))
    p.add_argument("--place", type=float, nargs=2, default=[220, 60], metavar=("X", "Y"))
    p.add_argument("--pick-z", type=float, default=-40.0, help="Nesneyi alma yüksekliği (mm)")
    p.add_argument("--safe-z", type=float, default=40.0, help="Taşıma yüksekliği (mm)")
    args = p.parse_args()

    bot = get_robot(args)
    try:
        if args.tool == "laser":
            if not confirm("Lazer 1 sn açılacak. Koruyucu gözlük takılı mı?", args):
                return
            bot.laser(True); time.sleep(1.0); bot.laser(False)
            return

        if not confirm(f"{args.tool} ile {args.pick} -> {args.place} al-bırak yapılacak. Devam?", args):
            return
        grab = (lambda: bot.suction(True)) if args.tool == "suction" else (lambda: bot.gripper("close"))
        release = (lambda: bot.suction(False)) if args.tool == "suction" else (lambda: bot.gripper("open"))

        # JUMP modu: güvenli yüksekliğe kalk -> hedefin üstüne git -> in
        bot.set_jump_params(height=args.safe_z - args.pick_z, z_limit=args.safe_z + 20)
        if args.tool == "gripper":
            bot.gripper("open")
        bot.jump_to(*args.pick, args.pick_z)
        grab()
        bot.wait_ms(300)                     # vakumun oturması için robot tarafında bekle
        bot.jump_to(*args.place, args.pick_z)
        release()
        bot.wait_ms(300)
        bot.move_relative(dz=args.safe_z - args.pick_z)
        bot.end_effector_off()               # pompayı kapat
        print("Al-bırak tamamlandı:", bot.get_pose())
    finally:
        try:
            bot.end_effector_off()
        except Exception:
            pass
        bot.close()


if __name__ == "__main__":
    main()
