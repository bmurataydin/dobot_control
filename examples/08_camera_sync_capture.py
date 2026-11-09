"""
08 — Robot + kamera senkron veri toplama (görüntü / poz eşleştirme).

Her adımda:  hareket et (bekle) -> titreşim için kısa bekle -> pozu oku ->
             pozdan SONRA çekilen ilk kareyi al -> kareyi PNG olarak kaydet.
Çıktı: frames/000000.png ... ve frames.csv (kare dosyası, poz, zaman damgaları,
poz-kare zaman farkı dt_ms). Görüntü tabanlı öğrenme veri setleri için uygundur.

    python examples/08_camera_sync_capture.py --port COM3 --camera 0 --shape circle --z -20
    python examples/08_camera_sync_capture.py --dry-run --fake-camera
"""
import os
import time

import numpy as np

from _common import base_parser, confirm, get_robot, session_dir
from dobot_toolkit import CsvLogger, trajectories as tr


class FakeCamera:
    """Kamera yokken hattı test etmek için sentetik kare üretir."""
    def __init__(self, w=640, h=480):
        self.w, self.h, self.n = w, h, 0

    def grab_fresh(self, after=None, timeout=1.0):
        self.n += 1
        img = np.full((self.h, self.w, 3), 40, np.uint8)
        img[:, (self.n * 10) % self.w] = 255
        return img, time.time()

    def close(self):
        pass


def main():
    p = base_parser(__doc__)
    p.add_argument("--camera", type=int, default=0, help="Kamera indeksi")
    p.add_argument("--fake-camera", action="store_true", help="Gerçek kamera yerine sentetik kare")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--shape", default="circle", choices=sorted(tr.SHAPES))
    p.add_argument("--center", type=float, nargs=2, default=[220, 0], metavar=("X", "Y"))
    p.add_argument("--size", type=float, default=30.0)
    p.add_argument("--points", type=int, default=40)
    p.add_argument("--z", type=float, default=0.0)
    p.add_argument("--settle", type=float, default=0.10, help="Durduktan sonra bekleme (sn)")
    args = p.parse_args()

    import cv2  # PNG yazmak için gerekli (pip install opencv-python)
    if args.fake_camera:
        cam = FakeCamera(args.width, args.height)
    else:
        from dobot_toolkit.camera import CameraGrabber
        cam = CameraGrabber(args.camera, args.width, args.height)

    target = tr.make_shape(args.shape, args.center, args.size, args.z, args.points)
    out = session_dir(name="camera_capture")
    frames_dir = os.path.join(out, "frames")
    os.makedirs(frames_dir, exist_ok=True)
    bot = get_robot(args)
    try:
        if not confirm(f"{len(target)} noktada kare alınacak. Devam?", args):
            return
        extra = ["step", "frame", "t_frame", "dt_ms", "target_x", "target_y", "target_z"]
        with CsvLogger(os.path.join(out, "frames.csv"), extra_fields=extra) as log:
            for i, (tx, ty, tz) in enumerate(target):
                bot.move_xyz(tx, ty, tz, mode="movl")
                time.sleep(args.settle)
                t_pose = time.time()
                pose = bot.get_pose()
                frame, t_frame = cam.grab_fresh(after=t_pose)
                name = f"{i:06d}.png"
                cv2.imwrite(os.path.join(frames_dir, name), frame)
                log.log(pose, t=t_pose, step=i, frame=name, t_frame=f"{t_frame:.6f}",
                        dt_ms=f"{(t_frame - t_pose) * 1000:.1f}", target_x=f"{tx:.3f}",
                        target_y=f"{ty:.3f}", target_z=f"{tz:.3f}")
                print(f"\r{i + 1}/{len(target)} kare", end="", flush=True)
        print(f"\n-> {out}")
    finally:
        cam.close()
        bot.close()


if __name__ == "__main__":
    main()
