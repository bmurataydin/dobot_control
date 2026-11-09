"""
Robotla senkron görüntü almak için arka plan kamera okuyucu (OpenCV gerekir).

Neden thread? OpenCV kamera tamponunda birkaç eski kare biriktirir; "o anda"
çekilen kareyi almak için kameradan SÜREKLİ okuyup en yeni kareyi zaman
damgasıyla tutmak gerekir. grab_fresh(), çağrıldığı andan SONRA gelen ilk
kareyi döndürür — böylece kare, robotun o anki pozuyla eşleşir.

    with CameraGrabber(0, width=640, height=480) as cam:
        pose = bot.get_pose()
        frame, t = cam.grab_fresh()
"""
import sys
import threading
import time
from typing import Optional, Tuple

try:
    import cv2
except ImportError:          # opsiyonel bağımlılık
    cv2 = None


def list_cameras(max_index: int = 5):
    """Açılabilen kamera indekslerini döndürür."""
    _require_cv2()
    found = []
    for i in range(max_index):
        cap = cv2.VideoCapture(i, _backend())
        if cap.isOpened():
            ok, _ = cap.read()
            if ok:
                found.append(i)
        cap.release()
    return found


def _require_cv2():
    if cv2 is None:
        raise ImportError("Kamera için OpenCV gerekli: pip install opencv-python")


def _backend():
    return cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_ANY


class CameraGrabber:
    def __init__(self, index: int = 0, width: int = 640, height: int = 480, fps: int = 30,
                 autofocus: Optional[bool] = False, warmup: float = 1.0):
        _require_cv2()
        self.cap = cv2.VideoCapture(index, _backend())
        if not self.cap.isOpened():
            raise RuntimeError(f"Kamera {index} açılamadı. list_cameras() ile indeksleri deneyin.")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        if autofocus is not None:
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 1 if autofocus else 0)

        self._cond = threading.Condition()
        self._frame = None
        self._t = 0.0
        self._count = 0
        self.read_failures = 0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, name="CameraGrabber", daemon=True)
        self._thread.start()
        time.sleep(warmup)        # pozlama / beyaz dengesi otursun

    def _loop(self):
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            t = time.time()
            if not ok:
                self.read_failures += 1
                time.sleep(0.01)
                continue
            with self._cond:
                self._frame, self._t = frame, t
                self._count += 1
                self._cond.notify_all()

    @property
    def actual_size(self) -> Tuple[int, int]:
        return (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

    @property
    def frames_read(self) -> int:
        return self._count

    def latest(self):
        """En son kare ve zaman damgası (beklemeden)."""
        with self._cond:
            return (None if self._frame is None else self._frame.copy()), self._t

    def grab_fresh(self, after: Optional[float] = None, timeout: float = 1.0):
        """`after` (varsayılan: şimdi) zamanından SONRA yakalanmış ilk kareyi döndürür."""
        after = time.time() if after is None else after
        deadline = time.monotonic() + timeout
        with self._cond:
            while self._frame is None or self._t <= after:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("Kameradan zamanında yeni kare gelmedi.")
                self._cond.wait(remaining)
            return self._frame.copy(), self._t

    def close(self):
        self._stop.set()
        self._thread.join(timeout=2.0)
        self.cap.release()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False
