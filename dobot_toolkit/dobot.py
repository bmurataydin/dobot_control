"""
Üst seviye Dobot Magician kontrol sınıfı.

    from dobot_toolkit import Dobot

    with Dobot("COM3") as bot:          # port=None -> otomatik bul
        bot.home()
        print(bot.get_pose())
        bot.move_xyz(220, 0, 30)          # mutlak kartezyen (MOVJ)
        bot.move_linear(220, 60, 30)      # düz çizgi (MOVL)
        bot.move_relative(dz=-20)         # göreli kartezyen
        bot.move_joints(10, 30, 40)       # eklem açıları (derece)

Tüm hareket metotları varsayılan olarak hareket FİZİKSEL OLARAK bitene kadar
bekler (wait=True). wait=False verilirse komut kuyruğa eklenir ve kuyruk
indeksi döner; daha sonra bot.wait() ile beklenebilir.

Seri protokol katmanı, sahada test edilmiş orijinal `lib/` paketidir
(lib/interface.py, lib/message.py, lib/parsers.py). Bu sınıf yalnızca onun
üzerine okunabilir isimli metotlar, güvenlik kontrolleri ve alarm yönetimi ekler.
"""
import time
from enum import IntEnum
from typing import Iterable, NamedTuple, Optional

from lib.interface import Interface

from .alarms import decode_alarms, format_alarms
from .safety import Workspace

_DEFAULT = object()


class PTPMode(IntEnum):
    """SetPTPCmd hareket modları (Dobot protokolü)."""
    JUMP_XYZ = 0        # kartezyen, kapı (kalk-git-in) hareketi
    MOVJ_XYZ = 1        # kartezyen hedef, eklem-uzayında en hızlı yol
    MOVL_XYZ = 2        # kartezyen hedef, DÜZ ÇİZGİ
    JUMP_ANGLE = 3      # eklem açısı hedefi, kapı hareketi
    MOVJ_ANGLE = 4      # eklem açısı hedefi, eklem-uzayında
    MOVL_ANGLE = 5      # eklem açısı hedefi, düz çizgi
    MOVJ_INC = 6        # EKLEM açılarına göreli artış
    MOVL_INC = 7        # KARTEZYEN göreli artış, düz çizgi
    MOVJ_XYZ_INC = 8    # kartezyen göreli artış, eklem-uzayında
    JUMP_MOVL_XYZ = 9   # jump + son kısım doğrusal


_XYZ_MODES = {"jump": PTPMode.JUMP_XYZ, "movj": PTPMode.MOVJ_XYZ, "movl": PTPMode.MOVL_XYZ}
_JOINT_MODES = {"jump": PTPMode.JUMP_ANGLE, "movj": PTPMode.MOVJ_ANGLE, "movl": PTPMode.MOVL_ANGLE}

# JOG komut kodları: kartezyen (X+,X-,Y+,Y-,Z+,Z-,R+,R-) / eklem (J1+,J1-,...)
_JOG_AXES = {"x": 1, "y": 3, "z": 5, "r": 7, 1: 1, 2: 3, 3: 5, 4: 7}


class Pose(NamedTuple):
    """get_pose() sonucu: kartezyen (mm) + eklem açıları (derece)."""
    x: float
    y: float
    z: float
    r: float
    j1: float
    j2: float
    j3: float
    j4: float

    @property
    def xyz(self):
        return (self.x, self.y, self.z)

    @property
    def xyzr(self):
        return (self.x, self.y, self.z, self.r)

    @property
    def joints(self):
        return (self.j1, self.j2, self.j3, self.j4)

    def __str__(self):
        return (f"x={self.x:8.2f} y={self.y:8.2f} z={self.z:8.2f} r={self.r:7.2f} | "
                f"j1={self.j1:7.2f} j2={self.j2:7.2f} j3={self.j3:7.2f} j4={self.j4:7.2f}")


class DobotTimeoutError(TimeoutError):
    """Kuyruktaki komut beklenen sürede tamamlanmadı."""


class DobotAlarmError(RuntimeError):
    """Robot alarm durumunda ve kuyruk ilerlemiyor."""


class Dobot:
    def __init__(self, port: Optional[str] = None, *,
                 serial_timeout: float = 5.0,
                 wait_timeout: float = 60.0,
                 speed: float = 50.0,
                 acceleration: float = 50.0,
                 speed_ratio: float = 50.0,
                 workspace=_DEFAULT,
                 clear_alarms: bool = True,
                 interface=None):
        """
        port          : 'COM3', '/dev/ttyUSB0' ... None ise otomatik bulunur.
        serial_timeout: tek bir seri yanıt için zaman aşımı (sn).
        wait_timeout  : bir hareketin bitmesi için azami bekleme (sn).
        speed/acceleration/speed_ratio : başlangıç hız ayarları (bkz. set_speed).
        workspace     : safety.Workspace; None verilirse yazılımsal sınır kontrolü KAPALI.
        interface     : (test/dry-run) hazır Interface benzeri nesne, ör. MockInterface().
        """
        if interface is None:
            if port is None:
                from .ports import find_dobot_port
                port = find_dobot_port()
            interface = Interface(port, timeout=serial_timeout)
        self.interface = interface
        self.port = port
        self.wait_timeout = wait_timeout
        self.workspace = Workspace() if workspace is _DEFAULT else workspace

        self.flush_input()
        # Önceki oturumdan kalan komutları temizle ve kuyruğu çalıştır
        self.interface.stop_queue(force=True)
        self.interface.clear_queue()
        self.interface.start_queue()
        if clear_alarms:
            self.clear_alarms()

        self.set_speed(speed, acceleration, speed_ratio)
        self.set_jump_params(height=20.0, z_limit=100.0)

    # ================================================================== bağlantı
    def close(self):
        """Seri bağlantıyı kapatır (robotu durdurmaz)."""
        try:
            self.interface.serial.close()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
        return False

    def connected(self) -> bool:
        return self.interface.connected()

    def flush_input(self):
        """Seri giriş tamponundaki eski/okunmamış baytları atar."""
        try:
            self.interface.serial.reset_input_buffer()
        except Exception:
            pass

    def info(self) -> dict:
        """Cihaz adı, firmware sürümü, port ve anlık poz."""
        def _safe(fn):
            try:
                return fn()
            except Exception as e:      # bazı firmware'ler bazı alanları desteklemez
                return f"<okunamadı: {e}>"
        return {
            "port": self.port,
            "name": _safe(self.interface.get_device_name),
            "serial_number": _safe(self.interface.get_device_serial_number),
            "version": _safe(self.interface.get_device_version),
            "pose": _safe(self.get_pose),
        }

    # ================================================================== durum
    def get_pose(self) -> Pose:
        """Anlık poz. NOT: Magician'da bu, kontrolcünün HESAPLADIĞI pozdur (adım
        motoru komut sayımı); harici ölçüm değildir. Bkz. docs/VERI_TOPLAMA.md"""
        return Pose(*[float(v) for v in self.interface.get_pose()])

    def get_alarms(self):
        """Aktif alarm kodlarının listesi (boş liste = alarm yok)."""
        return decode_alarms(self.interface.get_alarms_state())

    def clear_alarms(self):
        """Alarmları temizler; temizlendikten sonra hâlâ aktif olan kodları döndürür."""
        self.interface.clear_alarms_state()
        return self.get_alarms()

    def describe_alarms(self) -> str:
        return format_alarms(self.get_alarms())

    def current_queue_index(self) -> int:
        return int(self.interface.get_current_queue_index())

    # ================================================================== parametreler
    def set_speed(self, velocity: float = 50.0, acceleration: Optional[float] = None,
                  ratio: float = 50.0):
        """
        PTP ve CP hareketleri için hız/ivme ayarı.
          velocity     : eklem (°/s) ve kartezyen (mm/s) hız parametresi
          acceleration : ivme parametresi (None -> velocity ile aynı)
          ratio        : 0-100 arası genel çarpan (%); gerçek hız ~ velocity*ratio/100
        Komutlar kuyruğa girer, yani bir sonraki hareketten itibaren geçerlidir.
        """
        a = velocity if acceleration is None else acceleration
        self.interface.set_point_to_point_joint_params([velocity] * 4, [a] * 4)
        self.interface.set_point_to_point_coordinate_params(velocity, velocity, a, a)
        self.interface.set_point_to_point_common_params(ratio, ratio)
        self.interface.set_continous_trajectory_params(a, velocity, a)

    def set_jump_params(self, height: float = 20.0, z_limit: float = 100.0):
        """JUMP modlarında kalkma yüksekliği ve izin verilen maksimum z."""
        self.interface.set_point_to_point_jump_params(height, z_limit)

    # ================================================================== bekleme
    def wait(self, timeout: Optional[float] = None, poll: float = 0.05):
        """
        Kuyruktaki TÜM komutlar fiziksel olarak bitene kadar bekler.

        Yöntem: kuyruğa 0 ms'lik bir WAIT eklenir; firmware'in "mevcut indeks"
        sayacı bu WAIT'in indeksine ulaştığında önceki tüm hareketler bitmiştir.
        Süre aşılırsa DobotTimeoutError; alarm varken kuyruk ilerlemiyorsa
        DobotAlarmError fırlatılır (sonsuz bekleme yok).
        """
        target = int(self.interface.wait(0))
        self.wait_for_index(target, timeout=timeout, poll=poll)

    def wait_for_index(self, target: int, timeout: Optional[float] = None, poll: float = 0.05):
        timeout = self.wait_timeout if timeout is None else timeout
        start = time.monotonic()
        last_alarm_check = start
        last_idx = None
        while True:
            idx = self.current_queue_index()
            if idx >= target:
                return
            now = time.monotonic()
            if now - last_alarm_check > 1.0:
                active = [c for c in self.get_alarms() if c != 0x00]
                if active and idx == last_idx:
                    raise DobotAlarmError(
                        f"Robot alarmda, kuyruk ilerlemiyor (indeks {idx}/{target}). "
                        f"{format_alarms(active)}. bot.clear_alarms() ile temizleyin.")
                last_alarm_check, last_idx = now, idx
            if now - start > timeout:
                raise DobotTimeoutError(
                    f"Komut {timeout:.0f} sn içinde bitmedi (indeks {idx}/{target}). "
                    f"Alarmlar: {self.describe_alarms()}")
            time.sleep(poll)

    def wait_ms(self, milliseconds: int):
        """Kuyruğa gecikme ekler (robot tarafında bekler). Kuyruk indeksini döndürür."""
        return self.interface.wait(int(milliseconds))

    # ================================================================== hareket
    def home(self, wait: bool = True, timeout: float = 120.0):
        """Home (referans) hareketi. Her oturum başında önerilir (~20-40 sn sürer)."""
        idx = self.interface.set_homing_command(0)
        if wait:
            self.wait(timeout=max(timeout, self.wait_timeout))
        return idx

    def _ptp(self, mode: PTPMode, a, b, c, d, wait: bool):
        idx = self.interface.set_point_to_point_command(int(mode), float(a), float(b), float(c), float(d))
        if wait:
            self.wait()
        return idx

    def _check_xyz(self, x, y, z):
        if self.workspace is None:
            return
        cur = None
        if self.workspace.max_step is not None:
            p = self.get_pose()
            cur = (p.x, p.y)
        self.workspace.check(x, y, z, current_xy=cur)

    def move_xyz(self, x: float, y: float, z: float, r: Optional[float] = None,
                 mode: str = "movj", wait: bool = True):
        """
        Mutlak kartezyen hedefe git (mm).
          mode: 'movj' (eklem-uzayı, varsayılan), 'movl' (düz çizgi), 'jump' (kalk-git-in)
          r   : uç efektör açısı (derece); None -> mevcut r korunur.
        """
        if mode not in _XYZ_MODES:
            raise ValueError(f"mode {list(_XYZ_MODES)} olmalı")
        if r is None:
            r = self.get_pose().r
        self._check_xyz(x, y, z)
        return self._ptp(_XYZ_MODES[mode], x, y, z, r, wait)

    def move_linear(self, x, y, z, r=None, wait=True):
        """Düz çizgi (MOVL) ile mutlak kartezyen hedefe git."""
        return self.move_xyz(x, y, z, r, mode="movl", wait=wait)

    def jump_to(self, x, y, z, r=None, wait=True):
        """JUMP: önce yukarı kalk, hedefin üstüne git, in (bkz. set_jump_params)."""
        return self.move_xyz(x, y, z, r, mode="jump", wait=wait)

    def move_relative(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0, dr: float = 0.0,
                      wait: bool = True):
        """Mevcut konuma göre kartezyen artış (MOVL_INC, düz çizgi)."""
        if self.workspace is not None:
            p = self.get_pose()
            self._check_xyz(p.x + dx, p.y + dy, p.z + dz)
        return self._ptp(PTPMode.MOVL_INC, dx, dy, dz, dr, wait)

    def move_joints(self, j1: float, j2: float, j3: float, j4: Optional[float] = None,
                    mode: str = "movj", wait: bool = True):
        """Mutlak eklem açılarına git (derece). mode: 'movj' | 'movl' | 'jump'."""
        if mode not in _JOINT_MODES:
            raise ValueError(f"mode {list(_JOINT_MODES)} olmalı")
        if j4 is None:
            j4 = self.get_pose().j4
        if self.workspace is not None:
            self.workspace.check_joints(j1, j2, j3, j4)
        return self._ptp(_JOINT_MODES[mode], j1, j2, j3, j4, wait)

    def move_joints_relative(self, dj1=0.0, dj2=0.0, dj3=0.0, dj4=0.0, wait=True):
        """Eklem açılarına göreli artış (MOVJ_INC)."""
        if self.workspace is not None:
            p = self.get_pose()
            self.workspace.check_joints(p.j1 + dj1, p.j2 + dj2, p.j3 + dj3, p.j4 + dj4)
        return self._ptp(PTPMode.MOVJ_INC, dj1, dj2, dj3, dj4, wait)

    def follow_path(self, points: Iterable, speed: float = 50.0, relative: bool = False,
                    wait: bool = True, lookahead: int = 16):
        """
        Nokta listesini SÜREKLİ yörünge (CP) modunda izler — noktalar arasında
        durmadan akıcı hareket (çizim, kaynak, boyama vb. için).

          points    : [(x, y, z), ...] (mm). relative=True ise her biri artış.
          speed     : CP hızı (mm/s)
          lookahead : kuyrukta aynı anda bekleyen azami nokta sayısı. Robotun
                      kuyruğu sınırlı olduğundan noktalar akıtılarak gönderilir.
        """
        pts = [tuple(float(v) for v in p[:3]) for p in points]
        if not pts:
            return None
        if self.workspace is not None and not relative:
            for p in pts:
                self.workspace.check(*p)
        mode = 0 if relative else 1
        last_idx = None
        for p in pts:
            if last_idx is not None:
                self._throttle(last_idx, lookahead)
            last_idx = self.interface.set_continous_trajectory_command(mode, p[0], p[1], p[2], speed)
        if wait:
            self.wait()
        return last_idx

    def _throttle(self, last_idx: int, lookahead: int, poll: float = 0.01):
        start = time.monotonic()
        while last_idx - self.current_queue_index() >= lookahead:
            if time.monotonic() - start > self.wait_timeout:
                raise DobotTimeoutError(f"Kuyruk ilerlemiyor. Alarmlar: {self.describe_alarms()}")
            time.sleep(poll)

    def arc(self, via_point, end_point, wait=True):
        """Üç noktadan yay: mevcut konum -> via_point -> end_point. Noktalar (x,y,z,r)."""
        idx = self.interface.set_arc_command(list(via_point), list(end_point))
        if wait:
            self.wait()
        return idx

    def jog(self, axis, positive: bool = True, duration: float = 0.3, joint: bool = False):
        """
        Bir ekseni `duration` saniye boyunca sürekli hareket ettirir, sonra durdurur.
          axis : 'x','y','z','r' (kartezyen) ya da 1..4 (joint=True ile eklem)
        Not: JOG hız parametreleri firmware'e göre farklı davranabilir; düşük
        sürelerle deneyin.
        """
        key = axis.lower() if isinstance(axis, str) else int(axis)
        cmd = _JOG_AXES[key] + (0 if positive else 1)
        jog_type = 1 if joint else 0
        try:
            self.interface.set_jog_command(jog_type, cmd, queue=False)
            time.sleep(duration)
        finally:
            self.interface.set_jog_command(jog_type, 0, queue=False)

    def stop(self):
        """
        YAZILIMSAL acil durdurma: kuyruğu zorla durdurur, temizler ve yeniden
        başlatır. Gerçek acil durumda robotun güç düğmesini kullanın.
        """
        self.interface.stop_queue(force=True)
        self.interface.clear_queue()
        self.interface.start_queue()

    # ================================================================== uç efektörler
    def suction(self, on: bool = True, wait: bool = True):
        """Vakum (suction cup) aç/kapat."""
        idx = self.interface.set_end_effector_suction_cup(1, int(bool(on)))
        if wait:
            self.wait()
        return idx

    def gripper(self, state: str = "close", wait: bool = True):
        """Pnömatik tutucu: 'close' (kapat), 'open' (aç), 'off' (pompayı kapat)."""
        mapping = {"close": (1, 1), "open": (1, 0), "off": (0, 0)}
        if state not in mapping:
            raise ValueError("state 'close', 'open' veya 'off' olmalı")
        idx = self.interface.set_end_effector_gripper(*mapping[state])
        if wait:
            self.wait()
        return idx

    def laser(self, on: bool = True, wait: bool = True):
        """Lazer modülünü aç/kapat. Lazer ile çalışırken koruyucu gözlük kullanın."""
        idx = self.interface.set_end_effector_laser(1, int(bool(on)))
        if wait:
            self.wait()
        return idx

    def end_effector_off(self):
        """Vakum ve tutucu pompasını tamamen kapatır."""
        self.interface.set_end_effector_suction_cup(0, 0)
        self.interface.set_end_effector_gripper(0, 0)
        self.wait()

    # ================================================================== I/O
    def set_digital_output(self, address: int, level: bool):
        """EIO çıkışı (adres 1-20). Önce interface.set_io_multiplexing ile çıkış moduna alın."""
        return self.interface.set_io_do(address, int(bool(level)))

    # ================================================================== eski API
    # Orijinal lib/dobot.py ile geriye dönük uyumluluk. DİKKAT: orijinal adlar
    # yanıltıcıdır — move_to/slide_to KARTEZYEN değil EKLEM AÇISI alır.
    def slide_to(self, j1, j2, j3, j4, wait=True):
        """[ESKİ] MOVJ_ANGLE: eklem açılarına (derece) git. Yeni kod: move_joints()."""
        return self.move_joints(j1, j2, j3, j4, mode="movj", wait=wait)

    def move_to(self, j1, j2, j3, j4, wait=True):
        """[ESKİ] JUMP_ANGLE: eklem açılarına jump ile git. Yeni kod: move_joints(mode='jump')."""
        return self.move_joints(j1, j2, j3, j4, mode="jump", wait=wait)

    def move_to_relative(self, dx, dy, dz, dr, wait=True):
        """[ESKİ] MOVL_INC: kartezyen artış. Yeni kod: move_relative()."""
        return self.move_relative(dx, dy, dz, dr, wait=wait)

    def slide_to_relative(self, dj1, dj2, dj3, dj4, wait=True):
        """[ESKİ] MOVJ_INC: eklem artışı. Yeni kod: move_joints_relative()."""
        return self.move_joints_relative(dj1, dj2, dj3, dj4, wait=wait)
