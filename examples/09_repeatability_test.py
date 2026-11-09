"""
09 — Tekrarlanabilirlik / konumlama testi.

Bir ızgaradaki her hedefe, araya bir "uzak nokta" koyarak N kez gidilir ve
her varışta poz kaydedilir. Hedef başına ortalama hata, standart sapma ve
maksimum sapma raporlanır (results.csv + summary.json).

DİKKAT: get_pose(), kontrolcünün HESAPLADIĞI pozdur (adım motoru sayımı).
Bu test komut/IK tutarlılığını ve adım kaçırmayı (büyük sıçramalar) gösterir;
gerçek mekanik tekrarlanabilirlik için hedeflere bir komparatör (dial gauge)
veya kamera ile harici ölçüm ekleyin (bkz. 08 örneği, docs/VERI_TOPLAMA.md).

    python examples/09_repeatability_test.py --port COM3 --cycles 10 --z 0
"""
import json
import os

import numpy as np

from _common import base_parser, confirm, get_robot, session_dir
from dobot_toolkit import CsvLogger, trajectories as tr


def main():
    p = base_parser(__doc__)
    p.add_argument("--cycles", type=int, default=5)
    p.add_argument("--z", type=float, default=0.0)
    p.add_argument("--grid", type=int, default=3, help="Izgara boyutu (grid x grid)")
    p.add_argument("--x-range", type=float, nargs=2, default=[180, 260])
    p.add_argument("--y-range", type=float, nargs=2, default=[-60, 60])
    p.add_argument("--away", type=float, nargs=3, default=[200, 0, 60], help="Ara (uzak) nokta")
    p.add_argument("--mode", choices=["movj", "movl", "jump"], default="movj")
    args = p.parse_args()

    targets = tr.grid(args.x_range, args.y_range, args.z, args.grid, args.grid)
    bot = get_robot(args)
    out = session_dir(name="repeatability")
    errs = {i: [] for i in range(len(targets))}
    try:
        if not confirm(f"{len(targets)} hedef x {args.cycles} tur test edilecek. Devam?", args):
            return
        extra = ["cycle", "target_id", "target_x", "target_y", "target_z", "ex", "ey", "ez", "err_mm"]
        with CsvLogger(os.path.join(out, "results.csv"), extra_fields=extra,
                       compute_velocity=False) as log:
            for c in range(args.cycles):
                for i, t in enumerate(targets):
                    bot.move_xyz(*args.away, mode=args.mode)       # her seferinde uzaktan yaklaş
                    bot.move_xyz(*t, mode=args.mode)
                    pose = bot.get_pose()
                    e = np.array(pose.xyz) - t
                    err = float(np.linalg.norm(e))
                    errs[i].append(e)
                    log.log(pose, cycle=c, target_id=i, target_x=t[0], target_y=t[1], target_z=t[2],
                            ex=f"{e[0]:.4f}", ey=f"{e[1]:.4f}", ez=f"{e[2]:.4f}", err_mm=f"{err:.4f}")
                print(f"tur {c + 1}/{args.cycles} bitti")

        summary = []
        print(f"\n{'id':>3} {'hedef':>22} {'ort.hata':>9} {'std':>7} {'maks':>7}  (mm)")
        for i, t in enumerate(targets):
            e = np.array(errs[i])
            n = np.linalg.norm(e, axis=1)
            spread = np.linalg.norm(e - e.mean(axis=0), axis=1)     # tekrarlanabilirlik
            row = {"target_id": i, "target": t.tolist(), "mean_err": float(n.mean()),
                   "std_err": float(n.std()), "max_err": float(n.max()),
                   "repeatability_max": float(spread.max())}
            summary.append(row)
            print(f"{i:>3} ({t[0]:6.1f},{t[1]:6.1f},{t[2]:6.1f}) {row['mean_err']:9.3f} "
                  f"{row['std_err']:7.3f} {row['max_err']:7.3f}")
        with open(os.path.join(out, "summary.json"), "w", encoding="utf-8") as f:
            json.dump({"args": vars(args), "targets": summary}, f, indent=2)
        print(f"-> {out}")
    finally:
        bot.close()


if __name__ == "__main__":
    main()
