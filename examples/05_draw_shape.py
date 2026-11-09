"""
05 — Kalemle şekil çizme.

İki yöntem:
  --method cp  : sürekli yörünge (CP) — noktalar arasında durmadan, akıcı çizgi
  --method ptp : her noktaya ayrı MOVL — daha yavaş, her noktada kısa duruş

ÖNEMLİ: --draw-z, kalemin kağıda DEĞDİĞİ yüksekliktir ve düzeneğe göre değişir.
Önce 01 örneğinin --watch moduyla kolu elle kağıda indirip z değerini okuyun,
sonra 1-2 mm yukarısıyla başlayın.

    python examples/05_draw_shape.py --port COM3 --shape star --draw-z -62
    python examples/05_draw_shape.py --dry-run --shape heart --plot
"""
import os

import numpy as np

from _common import base_parser, confirm, get_robot, session_dir
from dobot_toolkit import PoseSampler, trajectories as tr


def main():
    p = base_parser(__doc__)
    p.add_argument("--shape", default="circle", choices=sorted(tr.SHAPES))
    p.add_argument("--center", type=float, nargs=2, default=[220, 0], metavar=("X", "Y"))
    p.add_argument("--size", type=float, default=30.0, help="Şekil boyutu/yarıçapı (mm)")
    p.add_argument("--points", type=int, default=120)
    p.add_argument("--draw-z", type=float, default=None, help="Kalemin kağıda değdiği z (mm)")
    p.add_argument("--lift", type=float, default=15.0, help="Kalem kaldırma yüksekliği (mm)")
    p.add_argument("--method", choices=["cp", "ptp"], default="cp")
    p.add_argument("--cp-speed", type=float, default=40.0, help="CP çizim hızı (mm/s)")
    p.add_argument("--hz", type=float, default=25.0, help="Kayıt frekansı")
    p.add_argument("--plot", action="store_true")
    args = p.parse_args()

    if args.draw_z is None:
        if not args.dry_run:
            p.error("--draw-z gerçek robotta zorunlu (kalemin kağıda değdiği z).")
        args.draw_z = -50.0

    pts = tr.make_shape(args.shape, args.center, args.size, args.draw_z, args.points)
    bot = get_robot(args)
    out = session_dir(name=f"draw_{args.shape}")
    try:
        if not confirm(f"'{args.shape}' z={args.draw_z} mm'de çizilecek. Devam?", args):
            return
        x0, y0, _ = pts[0]
        bot.move_xyz(x0, y0, args.draw_z + args.lift)   # başlangıcın üstüne git
        bot.move_linear(x0, y0, args.draw_z)            # kalemi indir

        with PoseSampler(bot, hz=args.hz, csv_path=os.path.join(out, "executed.csv")) as s:
            if args.method == "cp":
                bot.follow_path(pts, speed=args.cp_speed)
            else:
                for x, y, z in pts:
                    bot.move_linear(x, y, z)

        bot.move_relative(dz=args.lift)                 # kalemi kaldır
        np.savetxt(os.path.join(out, "target.csv"), pts, delimiter=",", header="x,y,z", comments="")
        print(f"{len(s.samples)} örnek ({s.achieved_hz:.1f} Hz) -> {out}")

        if args.plot:
            import matplotlib.pyplot as plt
            ex = np.array([q.xyz for _, q in s.samples])
            plt.plot(pts[:, 1], pts[:, 0], "b--", label="hedef")
            plt.plot(ex[:, 1], ex[:, 0], "r-", label="gerçekleşen (get_pose)")
            plt.axis("equal"); plt.legend(); plt.xlabel("y (mm)"); plt.ylabel("x (mm)")
            plt.title(f"{args.shape} — {args.method}"); plt.show()
    finally:
        bot.close()


if __name__ == "__main__":
    main()
