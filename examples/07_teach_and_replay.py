"""
07 — Elle öğretme (teach) ve tekrar oynatma (replay).

KAYIT: Ön koldaki yuvarlak "kilit açma" düğmesine basılı tutarak kolu elle
istediğiniz noktaya götürün, düğmeyi bırakın ve Enter'a basın -> nokta kaydedilir.
'q' + Enter ile kayıt biter, noktalar JSON dosyasına yazılır.

    python examples/07_teach_and_replay.py --port COM3 --record waypoints.json
    python examples/07_teach_and_replay.py --port COM3 --replay waypoints.json --repeat 3

--continuous: Enter beklemeden, Ctrl+C'ye kadar --hz frekansında sürekli kayıt
(kolu elle gezdirerek demonstrasyon / imitation-learning verisi toplamak için).
"""
import json
import time

from _common import base_parser, confirm, get_robot


def record(bot, path, continuous=False, hz=10.0):
    points = []
    if continuous:
        print(f"Sürekli kayıt {hz} Hz — Ctrl+C ile bitirin.")
        t0 = time.time()
        try:
            while True:
                pose = bot.get_pose()
                points.append({"t": round(time.time() - t0, 4), **pose._asdict()})
                print(f"\r{len(points):5d}  {pose}", end="", flush=True)
                time.sleep(1.0 / hz)
        except KeyboardInterrupt:
            print()
    else:
        print("Kolu elle konumlayın, Enter: nokta kaydet | q + Enter: bitir")
        while True:
            cmd = input(f"[{len(points)} nokta] > ").strip().lower()
            if cmd == "q":
                break
            pose = bot.get_pose()
            points.append(pose._asdict())
            print(f"   kaydedildi: {pose}")
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"continuous": continuous, "points": points}, f, indent=2)
    print(f"{len(points)} nokta -> {path}")


def replay(bot, path, repeat=1, mode="movj", dry_run=False):
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    pts = data["points"]
    if data.get("continuous"):
        pts = pts[::max(1, len(pts) // 200)]        # çok yoğun kayıtları seyrelt
    print(f"{len(pts)} nokta, {repeat} tekrar, mod={mode}")
    for rep in range(repeat):
        for i, q in enumerate(pts):
            bot.move_xyz(q["x"], q["y"], q["z"], r=q["r"], mode=mode)
            print(f"\rtekrar {rep + 1} nokta {i + 1}/{len(pts)}", end="", flush=True)
        print()


def main():
    p = base_parser(__doc__)
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--record", metavar="JSON")
    g.add_argument("--replay", metavar="JSON")
    p.add_argument("--continuous", action="store_true")
    p.add_argument("--hz", type=float, default=10.0)
    p.add_argument("--repeat", type=int, default=1)
    p.add_argument("--mode", choices=["movj", "movl", "jump"], default="movj")
    args = p.parse_args()

    bot = get_robot(args, home=bool(args.replay))
    try:
        if args.record:
            if args.dry_run:
                print("(dry-run: sahte robot elle hareket ettirilemez; sabit poz kaydedilir)")
            record(bot, args.record, args.continuous, args.hz)
        else:
            if confirm(f"{args.replay} oynatılacak. Devam?", args):
                replay(bot, args.replay, args.repeat, args.mode, args.dry_run)
    finally:
        bot.close()


if __name__ == "__main__":
    main()
