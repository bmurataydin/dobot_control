"""
10 — Kinematik model kontrolü ve L4 / z_offset kalibrasyonu.

Robotun bildirdiği (x, y, z) ile bizim ileri kinematik modelimizin eklem
açılarından hesapladığı (x, y, z) karşılaştırılır. Uyuşmazlık varsa uç
efektör ofseti (L4) ve z_offset en küçük kareler ile kestirilir. Bulunan
değerleri DobotKinematics(L4=..., z_offset=...) olarak kullanın.

Mod 1 (varsayılan): robot birkaç eklem konfigürasyonuna kendisi gider.
Mod 2 (--manual)  : kolu elle gezdirip Enter ile örnek alırsınız (hareket yok).

    python examples/10_kinematics_check.py --port COM3
    python examples/10_kinematics_check.py --port COM3 --manual --no-home
"""
import numpy as np

from _common import base_parser, confirm, get_robot
from dobot_toolkit import DobotKinematics

CONFIGS = [  # (j1, j2, j3) derece — masaya çarpmayan, orta bölge açıları
    (0, 20, 20), (15, 30, 30), (-15, 40, 35), (25, 25, 45), (-25, 45, 50), (0, 50, 40),
]


def main():
    p = base_parser(__doc__)
    p.add_argument("--manual", action="store_true")
    args = p.parse_args()

    bot = get_robot(args, home=not args.manual)
    samples = []
    try:
        if args.manual:
            print("Kolu elle konumlayın; Enter: örnek al | q + Enter: bitir")
            while input(f"[{len(samples)}] > ").strip().lower() != "q":
                samples.append(bot.get_pose())
        else:
            if not confirm("Robot 6 farklı eklem konfigürasyonuna gidecek. Devam?", args):
                return
            for j in CONFIGS:
                bot.move_joints(*j)
                samples.append(bot.get_pose())
                print("  ", samples[-1])
    finally:
        bot.close()

    if len(samples) < 2:
        print("Yeterli örnek yok.")
        return

    kin = DobotKinematics()
    P = np.array([s.xyz for s in samples])
    J = np.deg2rad(np.array([s.joints[:3] for s in samples]))
    model = np.array([kin.forward(*s.joints[:3]) for s in samples])
    err = np.linalg.norm(P - model, axis=1)
    print(f"\nVarsayılan model (L4={kin.L4}, z_offset={kin.z_offset}): "
          f"ort. hata {err.mean():.2f} mm, maks {err.max():.2f} mm")

    # Kestirim: yatay uzanım ve z, L4 ve z_offset'e doğrusal bağlıdır
    lx0 = kin.L2 * np.sin(J[:, 1]) + kin.L3 * np.cos(J[:, 2])
    z0 = kin.L2 * np.cos(J[:, 1]) - kin.L3 * np.sin(J[:, 2])
    L4 = float(np.mean(np.hypot(P[:, 0], P[:, 1]) - lx0))
    z_off = float(np.mean(P[:, 2] - z0))
    fitted = DobotKinematics(L4=L4, z_offset=z_off)
    model2 = np.array([fitted.forward(*s.joints[:3]) for s in samples])
    err2 = np.linalg.norm(P - model2, axis=1)
    print(f"Kestirilen model (L4={L4:.2f}, z_offset={z_off:.2f}): "
          f"ort. hata {err2.mean():.2f} mm, maks {err2.max():.2f} mm")
    print(f"\nKullanım: DobotKinematics(L4={L4:.2f}, z_offset={z_off:.2f})")


if __name__ == "__main__":
    main()
