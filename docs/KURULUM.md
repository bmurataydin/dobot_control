# Kurulum

## 1. Donanım bağlantısı

1. Robotu güç adaptörüne bağlayın, USB kablosunu bilgisayara takın.
2. Robotun arkasındaki güç düğmesine basın. Açılışta bir bip sesi ve yeşil LED görülür.
   (Kırmızı LED = alarm; bkz. [SORUN_GIDERME.md](SORUN_GIDERME.md))
3. **DobotStudio / DobotLab açıksa kapatın** — aynı seri portu yalnızca bir program kullanabilir.

## 2. USB-seri sürücüsü

Dobot Magician, kart sürümüne göre şu yongalardan birini kullanır:

| Yonga | USB VID | Windows'ta görünen ad |
|---|---|---|
| CH340 / CH341 | 1A86 | `USB-SERIAL CH340 (COMx)` |
| CH9102 | 1A86 | `USB-Enhanced-SERIAL CH9102 (COMx)` |
| CP210x | 10C4 | `Silicon Labs CP210x USB to UART Bridge (COMx)` |

- **Windows:** Aygıt Yöneticisi → "Bağlantı noktaları (COM ve LPT)". Görünmüyorsa WCH (CH340/CH9102)
  veya Silicon Labs (CP210x) sürücüsünü üreticinin sitesinden kurun. DobotStudio kurulumu da sürücüyü getirir.
- **Linux:** Sürücü çekirdekte hazırdır; port `/dev/ttyUSB0` veya `/dev/ttyACM0` olur. Kullanıcıyı
  `dialout` grubuna ekleyip oturumu kapatıp açın:

  ```bash
  sudo usermod -aG dialout $USER
  ```

  `brltty` paketi CH340 portunu ele geçirebilir; port görünüp kayboluyorsa kaldırın:

  ```bash
  sudo apt remove brltty
  ```

- **macOS:** Port `/dev/cu.wchusbserial*` veya `/dev/cu.SLAB_USBtoUART` olur.

Portu bulmak için:

```bash
python -m dobot_toolkit.ports
```

## 3. Python ortamı

Python 3.8+ gerekir (3.9–3.13 test edildi).

```bash
python -m venv .venv
```

Etkinleştirin (Windows: `.venv\Scripts\activate`, Linux/macOS: `source .venv/bin/activate`) ve kurun:

```bash
pip install -r requirements.txt
```

Paketi kurulu olarak kullanmak isterseniz (başka klasörlerden `import dobot_toolkit` için):

```bash
pip install -e .
```

Conda kullanıyorsanız:

```bash
conda create -n dobot python=3.11 -y
```

```bash
conda activate dobot
```

```bash
pip install -r requirements.txt
```

## 4. İlk test

```bash
python examples/01_connect_read_pose.py --port COM3
```

Beklenen çıktı (home pozunda):

```
-> Bağlandı (COM3). Alarmlar: Alarm yok
-> Poz: x=  259.10 y=    0.00 z=   -8.50 r=   0.00 | j1=   0.00 j2=  45.00 j3=  45.00 j4=   0.00
   version       : [4, 0, 3]
```

Ardından masada boş alan olduğundan emin olup:

```bash
python examples/02_home_and_basic_moves.py --port COM3 --z 30
```

## 5. Masa yüksekliğini (z) belirleme

z değerleri masaya ve uç efektöre göre değişir. Çizim veya al-bırak için:

1. `python examples/01_connect_read_pose.py --port COM3 --watch` çalıştırın.
2. Ön koldaki yuvarlak **kilit açma düğmesine** basılı tutarak ucu elle kağıda/nesneye indirin.
3. Ekranda okunan `z` değerini not edin (ör. -62). Komutlarda bunun 1–2 mm üstüyle başlayın.
4. Gerekirse `Workspace(z_min=...)` ile yazılımsal alt sınırı ayarlayın (varsayılan -70 mm).
