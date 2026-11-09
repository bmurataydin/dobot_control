# dobot_control

Python ile **Dobot Magician** robot kolunu kontrol etme, **veri toplama** ve **test** araçları.

- Saha testinden geçmiş seri protokol katmanı (`lib/`) — DobotStudio veya DLL gerekmez, yalnızca `pyserial`
- Okunabilir üst seviye API (`dobot_toolkit.Dobot`): `move_xyz`, `move_linear`, `move_joints`, `follow_path`, `suction`, `gripper` ...
- Takılmayan bekleme (`wait`): zaman aşımı + alarm tespiti, sonsuz döngü yok
- Veri kaydı: CSV kaydedici, arka planda **30 Hz** poz örnekleyici, kamera ile senkron kare alma
- Yazılımsal güvenlik: çalışma alanı ve eklem limitleri
- **Robot olmadan çalışma** (`--dry-run`): sahte robot ile tüm örnekler ve testler donanımsız çalışır
- 10 hazır örnek script + 16 birim testi + GitHub Actions CI

> ⚠️ **Güvenlik:** Robot hareket ederken çalışma alanından uzak durun. İlk denemeleri yüksek
> z değerleri ve düşük hızla yapın. Gerçek acil durumda robotun **güç düğmesini** kullanın.

---

## İçindekiler

1. [Hızlı başlangıç](#hızlı-başlangıç)
2. [Klasör yapısı](#klasör-yapısı)
3. [Temel kullanım](#temel-kullanım)
4. [Örnek scriptler](#örnek-scriptler)
5. [Veri toplama](#veri-toplama)
6. [Koordinat sistemi ve kinematik](#koordinat-sistemi-ve-kinematik)
7. [Testler](#testler)
8. [Gerçek robot test sonuçları](#gerçek-robot-test-sonuçları)
9. [Dokümantasyon](#dokümantasyon)

---

## Hızlı başlangıç

```bash
git clone https://github.com/bmurataydin/dobot_control.git
cd dobot_control
python -m venv .venv
```

Sanal ortamı etkinleştirin (Windows: `.venv\Scripts\activate`, Linux/macOS: `source .venv/bin/activate`), sonra:

```bash
pip install -r requirements.txt
```

USB sürücüsü gerekiyorsa (CH340 / CH9102 / CP210x) bkz. [docs/KURULUM.md](docs/KURULUM.md).

Bağlantıyı test edin (robot **hareket etmez**):

```bash
python examples/01_connect_read_pose.py --port COM3
```

Robot olmadan denemek için:

```bash
python examples/05_draw_shape.py --dry-run --shape star --plot
```

Portu bilmiyorsanız:

```bash
python -m dobot_toolkit.ports
```

---

## Klasör yapısı

```
dobot_control/
├── lib/                     # Seri protokol katmanı (sahada test edilmiş, DEĞİŞTİRİLMEDEN alındı)
│   ├── dobot.py             #   eski basit Dobot sınıfı (geriye dönük uyumluluk)
│   ├── interface.py         #   her protokol komutu için bir metot
│   ├── message.py           #   paket oluşturma / checksum / okuma
│   └── parsers.py           #   komut ID -> bayt (de)serileştirme tablosu
├── dobot_toolkit/           # Üst seviye araçlar
│   ├── dobot.py             #   Dobot sınıfı: hareket, bekleme, uç efektör, alarm
│   ├── kinematics.py        #   ileri / ters kinematik (robotla birebir uyumlu)
│   ├── safety.py            #   Workspace: çalışma alanı + eklem limitleri
│   ├── logger.py            #   CsvLogger, PoseSampler (arka plan poz kaydı)
│   ├── trajectories.py      #   daire, kare, yıldız, kalp, ızgara ... yörüngeleri
│   ├── camera.py            #   CameraGrabber: robotla senkron kare alma (OpenCV)
│   ├── alarms.py            #   alarm kodlarının çözümü ve açıklamaları
│   ├── ports.py             #   seri port otomatik bulma
│   └── mock.py              #   MockInterface: donanımsız sahte robot
├── examples/                # 01..10 numaralı örnek scriptler
├── tests/                   # birim testleri (donanım gerekmez)
├── docs/                    # ayrıntılı dokümantasyon
├── requirements.txt
└── pyproject.toml           # pip install -e . ile paket olarak kurulum
```

---

## Temel kullanım

```python
from dobot_toolkit import connect

bot = connect("COM3")            # port=None -> otomatik bul; dry_run=True -> sahte robot
bot.home()                       # her oturum başında önerilir (~20 sn)

print(bot.get_pose())            # x, y, z, r (mm/°) + j1..j4 (°)

bot.move_xyz(220, 0, 20)         # mutlak kartezyen hedef (MOVJ: en hızlı eklem yolu)
bot.move_linear(220, 40, 20)     # düz çizgi (MOVL)
bot.jump_to(220, -40, 20)        # kalk -> git -> in (JUMP)
bot.move_relative(dz=15)         # göreli kartezyen artış
bot.move_joints(10, 30, 40)      # eklem açıları (derece)
bot.move_joints_relative(dj1=-10)

# Komutları kuyruğa at, sonra hepsini bekle
for y in (-20, 20, 0):
    bot.move_xyz(220, y, 20, wait=False)
bot.wait()

# Sürekli yörünge (çizim): noktalar arasında durmadan
from dobot_toolkit import trajectories as tr
bot.follow_path(tr.circle(center=(220, 0), radius=20, z=20, n=60), speed=40)

bot.suction(True); bot.suction(False)       # vakum
bot.gripper("close"); bot.gripper("open")   # tutucu
bot.end_effector_off()                      # pompayı kapat

bot.set_speed(velocity=100, ratio=80)       # hız ayarı
print(bot.describe_alarms())                # "Alarm yok" ya da açıklamalı liste
bot.close()
```

`with connect("COM3") as bot:` şeklinde de kullanılabilir; çıkışta port kapanır.

### Hareket modları

| Metot | Protokol modu | Hedef | Yol |
|---|---|---|---|
| `move_xyz(..., mode="movj")` | `MOVJ_XYZ` (1) | kartezyen | eklem uzayında (eğri olabilir) |
| `move_linear(...)` | `MOVL_XYZ` (2) | kartezyen | düz çizgi |
| `jump_to(...)` | `JUMP_XYZ` (0) | kartezyen | kalk-git-in (`set_jump_params`) |
| `move_joints(...)` | `MOVJ_ANGLE` (4) | eklem açıları | eklem uzayında |
| `move_relative(...)` | `MOVL_INC` (7) | kartezyen artış | düz çizgi |
| `move_joints_relative(...)` | `MOVJ_INC` (6) | eklem artışı | eklem uzayında |
| `follow_path(...)` | CP (91) | nokta listesi | sürekli, duraksamasız |

### Eski kodla uyumluluk

Daha önce `from lib.dobot import Dobot` ile yazılmış scriptler **değişmeden** çalışır (`lib/` aynen korunmuştur).
Dikkat: eski adlar yanıltıcıdır — `slide_to(a, b, c, r)` aslında **eklem açısı** (MOVJ_ANGLE),
`move_to` ise **JUMP_ANGLE** gönderir. Yeni `Dobot` sınıfında da aynı anlamla bulunurlar.

---

## Örnek scriptler

Hepsi `--port COM3`, `--dry-run`, `--speed`, `--ratio`, `--no-home` argümanlarını kabul eder.
Hareket öncesi onay sorarlar. Çıktılar `data/<ad>_<tarih>/` klasörüne yazılır.

| # | Script | Ne yapar |
|---|---|---|
| 01 | `01_connect_read_pose.py` | Port listesi, cihaz bilgisi, poz, alarmlar. `--watch` ile canlı poz (hareket yok) |
| 02 | `02_home_and_basic_moves.py` | Home, MOVJ/MOVL kare, göreli hareket, r ekseni, kuyruklu hareket |
| 03 | `03_motion_modes.py` | MOVJ / MOVL / JUMP yollarını kaydedip karşılaştırma, JOG |
| 04 | `04_end_effectors.py` | Vakum / tutucu ile al-bırak, lazer |
| 05 | `05_draw_shape.py` | Kalemle şekil çizme (CP veya PTP), hedef vs. gerçekleşen grafik |
| 06 | `06_record_trajectory.py` | **Veri toplama**: adım adım yörünge + her adımda poz/hata/süre, `meta.json` |
| 07 | `07_teach_and_replay.py` | Kolu elle gezdirerek nokta kaydetme ve tekrar oynatma |
| 08 | `08_camera_sync_capture.py` | **Robot + kamera** senkron kare/poz veri seti |
| 09 | `09_repeatability_test.py` | Izgara üzerinde tekrarlanabilirlik testi, hedef başına istatistik |
| 10 | `10_kinematics_check.py` | Kinematik model kontrolü, L4 / z_offset kalibrasyonu |

Örnek komutlar:

```bash
python examples/05_draw_shape.py --port COM3 --shape star --draw-z -62
```

```bash
python examples/06_record_trajectory.py --port COM3 --shape square --z 0 --repeat 3 --stream
```

```bash
python examples/08_camera_sync_capture.py --port COM3 --camera 0 --shape circle --z 0
```

```bash
python examples/09_repeatability_test.py --port COM3 --cycles 10
```

---

## Veri toplama

```python
from dobot_toolkit import connect, CsvLogger, PoseSampler, trajectories as tr

bot = connect("COM3")
pts = tr.square(center=(220, 0), side=40, z=0, n=40)

# (a) Adım adım: her hedeften sonra bir satır
with CsvLogger("data/steps.csv", extra_fields=["step"]) as log:
    for i, p in enumerate(pts):
        bot.move_linear(*p)
        log.log(bot.get_pose(), step=i)

# (b) Sürekli: hareket boyunca 30 Hz akış
with PoseSampler(bot, hz=30, csv_path="data/stream.csv") as s:
    bot.follow_path(pts, speed=40)
print(s.achieved_hz)
```

CSV sütunları: `t_unix, t_rel, x, y, z, r, j1..j4, vx, vy, vz, vr, vj1..vj4` + sizin ek alanlarınız.
Hızlar bir önceki satıra göre (birim/saniye) hesaplanır.

> **Önemli:** Magician'ın `get_pose()` değeri, kontrolcünün adım motoru sayımından **hesapladığı**
> pozdur; harici bir ölçüm değildir. Gerçek konum hatası / tekrarlanabilirlik ölçmek için kamera
> veya komparatör gibi harici ölçüm kullanın. Ayrıntılar: [docs/VERI_TOPLAMA.md](docs/VERI_TOPLAMA.md).

---

## Koordinat sistemi ve kinematik

- Orijin: taban dönme ekseni, z=0 omuz (arka kol ekseni) yüksekliği. Birimler mm ve derece.
- `j1` taban, `j2` arka kol (dikeyden), `j3` ön kol (yataydan), `j4` uç servo. `r = j1 + j4`.
- Home pozu: `j2 = j3 = 45°` → `x = 259.1, y = 0, z = -8.5`.
- `DobotKinematics()` (L2=135, L3=147, L4=59.7) robotun bildirdiği pozla **0.000 mm** uyumludur.

```python
from dobot_toolkit import DobotKinematics
kin = DobotKinematics()
kin.forward(0, 45, 45)      # (259.1, 0.0, -8.5)
kin.inverse(220, 0, 20)     # (j1, j2, j3) derece
```

---

## Testler

```bash
python -m pytest tests -v
```

pytest kurulu değilse:

```bash
python -m unittest discover tests -v
```

Testler sahte robotla çalışır, donanım gerekmez. Her push'ta GitHub Actions (Windows + Ubuntu) otomatik çalıştırır.

---

## Gerçek robot test sonuçları

Dobot Magician (firmware 4.0.3, CH9102 USB-seri, COM3) üzerinde, z = +20 mm, `speed=50, ratio=50`:

| İşlem | Süre | Sonuç |
|---|---|---|
| `home()` | 19.9 s | x=259.17, z=-8.44 |
| `move_xyz(220, 0, 20)` | 1.9 s | hedef hatası 0.000 mm |
| `move_linear(220, 40, 20)` | 2.7 s | ✓ |
| `jump_to(220, -40, 20)` | 5.5 s | ✓ |
| `move_relative(dz=±15)` | 1.6 s | ✓ |
| `move_joints` / `move_joints_relative` | 1.4 s | ✓ |
| 3 × `wait=False` + `wait()` | 3.3 s | ✓ |
| `follow_path` daire r=20 mm, 60 nokta | 11.9 s | yarıçap 20.00–20.04 mm |
| `PoseSampler(hz=30)` hareket boyunca | — | 358 örnek, **30.0 Hz**, 0 hata |
| `jog("z", duration=0.3)` | 0.3 s | +2.5 mm |
| Kinematik model vs. `get_pose()` | — | 0.000 mm fark |

---

## Dokümantasyon

- [docs/KURULUM.md](docs/KURULUM.md) — sürücü, Python ortamı, Linux port izinleri
- [docs/API.md](docs/API.md) — tüm sınıf ve metotların başvuru kılavuzu
- [docs/PROTOKOL.md](docs/PROTOKOL.md) — seri paket formatı, komut ID'leri, kuyruk mantığı
- [docs/VERI_TOPLAMA.md](docs/VERI_TOPLAMA.md) — veri toplama yöntemleri, senkronizasyon, iyi pratikler
- [docs/SORUN_GIDERME.md](docs/SORUN_GIDERME.md) — takılma, alarm, port meşgul vb.

## Teşekkür

`lib/` protokol katmanı, açık kaynak **dobot-python** (Alex Gustafsson) projesinin uyarlanmış hâlidir
(zaman aşımı ve takılma önleme eklemeleriyle). Dobot iletişim protokolü belgesi: *Dobot Magician
Communication Protocol v1.1.5*.

## Lisans

MIT — bkz. [LICENSE](LICENSE).
