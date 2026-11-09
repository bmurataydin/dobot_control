"""
02 — Home + temel hareketler.

Robot önünde ~25 cm yarıçaplı, masa üstünde en az 5 cm boşluk olan bir alan
gerektirir. Z değerleri masaya göre değişir: --z ile güvenli bir yükseklik verin.

    python examples/02_home_and_basic_moves.py --port COM3 --z 30
"""
from _common import base_parser, confirm, get_robot


def main():
    p = base_parser(__doc__)
    p.add_argument("--z", type=float, default=30.0, help="Hareket yüksekliği (mm), masadan güvenli olmalı.")
    args = p.parse_args()

    bot = get_robot(args)
    try:
        if not confirm(f"Robot z={args.z} mm yükseklikte bir kare çizecek. Devam?", args):
            return
        z = args.z
        # 1) Mutlak kartezyen hedef (MOVJ: eklem-uzayında hızlı yol)
        bot.move_xyz(220, 0, z)
        print("merkez     :", bot.get_pose())

        # 2) Düz çizgilerle (MOVL) bir kare
        for x, y in [(250, -30), (250, 30), (190, 30), (190, -30), (250, -30)]:
            bot.move_linear(x, y, z)
            print(f"köşe ({x},{y}):", bot.get_pose())

        # 3) Göreli hareket: 20 mm yukarı, sonra geri
        bot.move_relative(dz=20)
        bot.move_relative(dz=-20)

        # 4) Uç efektörü döndür (r ekseni, servo takılıysa)
        bot.move_xyz(220, 0, z, r=30)
        bot.move_xyz(220, 0, z, r=0)

        # 5) wait=False: komutları kuyruğa at, sonra hepsini bekle
        for y in (-40, 40, 0):
            bot.move_xyz(220, y, z, wait=False)
        bot.wait()
        print("son        :", bot.get_pose())
    finally:
        bot.close()


if __name__ == "__main__":
    main()
