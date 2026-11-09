"""
06 — Veri toplama: hedef yörüngeyi adım adım uygula, her adımı kaydet.

Her adımda kaydedilenler (steps.csv):
    step, rep, hedef (target_x/y/z), robotun bildirdiği poz (x..j4),
    hızlar (önceki adıma göre), komut süresi (move_time_s), hedef hatası (err_mm)
İsteğe bağlı --stream ile hareket boyunca yüksek frekanslı akış (stream.csv).
meta.json: tüm ayarlar + cihaz bilgisi (deneyin tekrarlanabilmesi için).

    python examples/06_record_trajectory.py --port COM3 --shape square --z -55 --repeat 3
    python examples/06_record_trajectory.py --dry-run --shape circle --stream
"""
import json
import os
import time

import numpy as np

from _common import base_parser, confirm, get_robot, session_dir
from dobot_toolkit import CsvLogger, PoseSampler, trajectories as tr


def main():
    p = base_parser(__doc__)
    p.add_argument("--shape", default="circle", choices=sorted(tr.SHAPES))
    p.add_argument("--center", type=float, nargs=2, default=[220, 0], metavar=("X", "Y"))
    p.add_argument("--size", type=float, default=30.0)
    p.add_argument("--points", type=int, default=50)
    p.add_argument("--z", type=float, default=0.0, help="Yörünge yüksekliği (mm)")
    p.add_argument("--repeat", type=int, default=1, help="Aynı yörüngeyi kaç kez tekrarla")
    p.add_argument("--mode", choices=["movl", "movj"], default="movl")
    p.add_argument("--stream", action="store_true", help="Hareket boyunca sürekli poz akışı kaydet")
    p.add_argument("--hz", type=float, default=30.0)
    args = p.parse_args()

    target = tr.make_shape(args.shape, args.center, args.size, args.z, args.points)
    bot = get_robot(args)
    out = session_dir(name=f"record_{args.shape}")

    meta = {"args": vars(args), "device": {k: str(v) for k, v in bot.info().items()},
            "start_time": time.strftime("%Y-%m-%d %H:%M:%S"), "n_points": len(target)}
    with open(os.path.join(out, "meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)
    np.savetxt(os.path.join(out, "target.csv"), target, delimiter=",", header="x,y,z", comments="")

    sampler = None
    try:
        if not confirm(f"{args.shape} yörüngesi {args.repeat} kez uygulanacak (z={args.z}). Devam?", args):
            return
        bot.move_xyz(*target[0])
        if args.stream:
            sampler = PoseSampler(bot, hz=args.hz, csv_path=os.path.join(out, "stream.csv")).start()

        extra = ["rep", "step", "target_x", "target_y", "target_z", "move_time_s", "err_mm"]
        with CsvLogger(os.path.join(out, "steps.csv"), extra_fields=extra) as log:
            for rep in range(args.repeat):
                for i, (tx, ty, tz) in enumerate(target):
                    t0 = time.time()
                    bot.move_xyz(tx, ty, tz, mode=args.mode)
                    pose = bot.get_pose()
                    err = float(np.linalg.norm(np.array(pose.xyz) - (tx, ty, tz)))
                    log.log(pose, rep=rep, step=i, target_x=f"{tx:.3f}", target_y=f"{ty:.3f}",
                            target_z=f"{tz:.3f}", move_time_s=f"{time.time() - t0:.4f}",
                            err_mm=f"{err:.4f}")
                print(f"tekrar {rep + 1}/{args.repeat} bitti")
        print(f"{log.rows} adım kaydedildi -> {out}")
    except KeyboardInterrupt:
        print("\nKullanıcı durdurdu; o ana kadarki veri kaydedildi.")
        bot.stop()
    finally:
        if sampler is not None:
            sampler.stop()
            print(f"akış: {len(sampler.samples)} örnek, {sampler.achieved_hz:.1f} Hz")
        bot.close()


if __name__ == "__main__":
    main()
