from time import sleep, monotonic

from lib.interface import Interface


class Dobot:
    def __init__(self, port, serial_timeout=5.0, wait_timeout=40.0):
        # serial_timeout : Interface okuma zaman asimi (sn) — bkz. interface.py
        # wait_timeout   : wait() icin kuyruk yoklama ust siniri (sn); asilirsa
        #                  TimeoutError. Sonsuz "while True" beklemesini onler.
        self._wait_timeout = wait_timeout
        self.interface = Interface(port, timeout=serial_timeout)

        self.interface.stop_queue(True)
        self.interface.clear_queue()
        self.interface.start_queue()

        self.interface.set_point_to_point_jump_params(10, 10)
        self.interface.set_point_to_point_joint_params([50, 50, 50, 50], [50, 50, 50, 50])
        self.interface.set_point_to_point_coordinate_params(50, 50, 50, 50)
        self.interface.set_point_to_point_common_params(50, 50)
        self.interface.set_point_to_point_jump2_params(5, 5, 5)

        self.interface.set_jog_joint_params([50, 50, 50, 50], [50, 50, 50, 50])
        self.interface.set_jog_coordinate_params([50, 50, 50, 50], [50, 50, 50, 50])
        self.interface.set_jog_common_params(50, 50)

        self.interface.set_continous_trajectory_params(50, 50, 50)

    def connected(self):
        return self.interface.connected()

    def get_pose(self):
        return self.interface.get_pose()

    def home(self, wait=True):
        self.interface.set_homing_command(0)
        if wait:
            # Home hareketi digerlerinden uzun surer; daha genis zaman asimi ver.
            self.wait(timeout=max(self._wait_timeout, 90.0))

    # Move to the absolute coordinate, one axis at a time
    def move_to(self, x, y, z, r, wait=True):
        self.interface.set_point_to_point_command(3, x, y, z, r)
        if wait:
            self.wait()

    # Slide to the absolute coordinate, shortest possible path
    def slide_to(self, x, y, z, r, wait=True):
        self.interface.set_point_to_point_command(4, x, y, z, r)
        if wait:
            self.wait()

    # Move to the absolute coordinate, one axis at a time
    def move_to_relative(self, x, y, z, r, wait=True):
        self.interface.set_point_to_point_command(7, x, y, z, r)
        if wait:
            self.wait()

    # Slide to the relative coordinate, one axis at a time
    def slide_to_relative(self, x, y, z, r, wait=True):
        self.interface.set_point_to_point_command(6, x, y, z, r)
        if wait:
            self.wait()

    # Wait until the instruction finishes
    def wait(self, queue_index=None, timeout=None, poll=0.2):
        """
        Kuyruktaki komutlarin fiziksel olarak bitmesini bekler.

        ESKI DAVRANIS (hata): once no-op WAIT eklenir, ardindan currentIndex
        OKUNUP 'kesinlikle buyuk olana kadar' (>) donulurdu. Robot guc dongusu
        yapilmadan yeniden baglandiginda firmware kuyruk sayaci beklenen indekse
        hic ulasmayabildiginden bu 'while True' dongusu SONSUZA dek asili
        kalabiliyordu — 2. calistirmada takilma bunun tipik belirtisiydi.

        YENI DAVRANIS:
          - Hedef, kuyruga eklenen no-op WAIT komutunun DONEN indeksidir. Boylece
            currentIndex bu indekse ULASINCA (>=) tum onceki komutlar (hareket
            dahil) bitmis olur — dogru senkron deseni budur.
          - timeout suresi asilirsa TimeoutError firlatilir; sonsuz bekleme yok.
            (collect_real_gt / run_real bunu yakalayip loglari kaydeder.)

        queue_index verilirse (follow_path) o indeks hedef alinir; no-op WAIT
        eklenmez.
        """
        if timeout is None:
            timeout = getattr(self, "_wait_timeout", 40.0)

        # If there are no more instructions in the queue, get_current_queue_index
        # sticks at the last instruction even after it finished. A zero-length
        # WAIT is enqueued as a no-op; its returned index is our completion target.
        if queue_index is None:
            target = self.interface.wait(0)
        else:
            target = queue_index

        start = monotonic()
        while self.interface.get_current_queue_index() < target:
            if timeout is not None and (monotonic() - start) > timeout:
                raise TimeoutError(
                    "Dobot kuyruk komutu {:.0f} sn icinde tamamlanmadi (hedef "
                    "indeks={}). Kuyruk ilerlemiyor olabilir (alarm ya da guc "
                    "dongusu gerektiren durum).".format(timeout, target)
                )
            sleep(poll)

    # Move according to the given path
    def follow_path(self, path, wait=True):
        self.interface.stop_queue()
        queue_index = None
        for point in path:
            queue_index = self.interface.set_continous_trajectory_command(1, point[0], point[1], point[2], 50)
        self.interface.start_queue()
        if wait:
            self.wait(queue_index)

    # Move according to the given path
    def follow_path_relative(self, path, wait=True):
        self.interface.stop_queue()
        queue_index = None
        for point in path:
            queue_index = self.interface.set_continous_trajectory_command(0,  point[0], point[1], point[2], 50)
        self.interface.start_queue()
        if wait:
            self.wait(queue_index)
