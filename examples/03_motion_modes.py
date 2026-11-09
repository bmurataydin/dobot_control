"""
03 — Hareket modlarının karşılaştırması: MOVJ, MOVL, JUMP, eklem hareketleri, JOG.

Her modda aynı A->B hareketi yapılır ve arka planda poz örneklenip
data/motion_modes_*/<mod>.csv dosyalarına yazılır. Böylece yol farkını
(eklem-uzayı eğrisi vs. düz çizgi vs. kapı hareketi) grafikte görebilirsiniz.

    python examples/03_motion_modes.py --port COM3 --z 20
"""
import os

from _common import base_parser, confirm, get_robot, session_dir
from dobot_toolkit import PoseSampler


def main():
    p = base_parser(__doc__)
    p.add_argument("--z", type=float, default=20.0)
    p.add_argument("--hz", type=float, default=25.0, help="Örnekleme frekansı")
    p.add_argument("--plot", action="store_true", help="Sonunda XY yollarını çiz (matplotlib)")
    args = p.parse_args()

    bot = get_robot(args)
    out = session_dir(name="motion_modes")
    A, B = (200, -80, args.z), (200, 80, args.z)
    paths = {}
    try:
        if not confirm("A(200,-80) <-> B(200,80) arasında farklı modlarla hareket edilecek. Devam?", args):
            return
        bot.set_jump_params(height=30, z_limit=args.z + 60)

        for mode in ("movj", "movl", "jump"):
            bot.move_xyz(*A)                                  # başlangıca dön
            with PoseSampler(bot, hz=args.hz, csv_path=os.path.join(out, f"{mode}.csv")) as s:
                bot.move_xyz(*B, mode=mode)
            paths[mode] = [pose for _, pose in s.samples]
            print(f"{mode:5s}: {len(s.samples)} örnek, {s.achieved_hz:.1f} Hz")

        # Eklem açılarıyla hareket (derece)
        pose = bot.get_pose()
        bot.move_joints(pose.j1 - 20, pose.j2, pose.j3)       # tabanı 20° döndür
        bot.move_joints_relative(dj1=20)                      # geri al

        # JOG: z eksenini 0.3 sn yukarı sür
        bot.jog("z", positive=True, duration=0.3)
        print("JOG sonrası:", bot.get_pose())
        print(f"CSV'ler: {out}")

        if args.plot:
            import matplotlib.pyplot as plt
            for mode, poses in paths.items():
                plt.plot([q.y for q in poses], [q.x for q in poses], ".-", label=mode)
            plt.xlabel("y (mm)"); plt.ylabel("x (mm)"); plt.legend(); plt.axis("equal")
            plt.title("Hareket modları — XY yolu"); plt.show()
    finally:
        bot.close()


if __name__ == "__main__":
    main()
