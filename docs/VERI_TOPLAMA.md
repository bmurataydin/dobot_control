# Veri toplama rehberi

## Hangi yöntem ne zaman?

| Yöntem | Araç | Kullanım |
|---|---|---|
| Adım adım (durarak) | `move_*` + `CsvLogger` | Her hedefte durup kaydetmek: davranış klonlama / gözlem-eylem çiftleri, kalibrasyon |
| Sürekli akış | `follow_path` + `PoseSampler` | Akıcı hareket boyunca yüksek frekanslı yörünge (hız profilleri, çizim) |
| Elle demonstrasyon | `07_teach_and_replay.py --continuous` | İnsan gösterimi (imitation learning) |
| Görüntü + poz | `CameraGrabber.grab_fresh` | Görüntü tabanlı veri setleri |

## `get_pose()` neyi ölçer?

Dobot Magician adım motorludur; `get_pose()` kontrolcünün **komut verdiği adımlardan hesapladığı**
pozdur, enkoderle ölçülen gerçek konum değildir. Sonuçları:

- Durgun hâlde `get_pose()` neredeyse her zaman hedefe eşittir (gerçek robotta 0.000 mm ölçüldü).
  Bu, robotun hedefe **gerçekten** gittiğini kanıtlamaz.
- Adım kaçırma, kayış/dişli boşluğu, yük altında esneme `get_pose()`'da **görünmez**.
- Gerçek konum hatası için harici ölçüm kullanın: kamera + işaretçi (ArUco), komparatör, kağıda çizilen izin taranması.
- Hareket sırasında `PoseSampler` ile alınan değerler, kontrolcünün planlanan yörüngesini verir
  (hız/ivme profilini incelemek için uygundur).

## Zaman damgaları ve senkronizasyon

- `CsvLogger` her satıra `t_unix` (duvar saati) ve `t_rel` (ilk satırdan beri) yazar.
- Robot ve kamera verisini aynı saatle (`time.time()`) damgalayın; farklı makineler kullanılıyorsa saatleri senkronlayın.
- Kamera karesi: `grab_fresh(after=t_pose)` pozdan **sonra** çekilmiş ilk kareyi verir. `frames.csv`'deki `dt_ms`
  sütunu poz-kare gecikmesini gösterir (tipik 0–35 ms @30 fps).
- Robot durduktan sonra ~0.1 s bekleyin (`--settle`): mekanik titreşim hareket bulanıklığı yaratır.
- Kamerada otomatik odak/pozlamayı kapatın; sabit sahnede kareler arası parlaklık/keskinlik dalgalanması sahte sinyal üretir.

## Örnekleme frekansı

Her `get_pose()` bir istek/yanıt döngüsüdür (~5–10 ms). Gerçek robotta hareket sırasında
**30 Hz** hatasız doğrulandı. Daha yüksek frekans isterseniz (`hz=50`) `achieved_hz` değerini kontrol edin.
Seri hat kilitli olduğundan örnekleyici, hareket ve bekleme komutlarıyla güvenle paralel çalışır;
ancak çok yüksek frekans, `wait()` döngüsünü bir miktar yavaşlatır.

## Deney kaydı (tekrarlanabilirlik)

`06_record_trajectory.py` her oturum için şu yapıyı üretir:

```
data/record_square_20261109_153000/
├── meta.json      # tüm argümanlar, cihaz bilgisi, başlangıç zamanı
├── target.csv     # hedef yörünge
├── steps.csv      # adım başına poz, hedef, hata, süre
└── stream.csv     # (--stream) sürekli poz akışı
```

İyi pratikler:

- Her oturum başında `home()` (adım kaçırma birikimini sıfırlar).
- Hız ayarlarını (`--speed`, `--ratio`) `meta.json`'da saklayın; hız, yörünge şeklini etkiler.
- Aynı koşulu birkaç kez tekrarlayın (`--repeat`), varyansı raporlayın.
- Alarm oluştuysa o oturumu veri setinden çıkarın (`bot.get_alarms()`'ı kaydedin).
- Büyük veriyi git'e koymayın (`data/` zaten `.gitignore`'da).

## CSV'yi okumak

```python
import pandas as pd
df = pd.read_csv("data/record_square_.../steps.csv")
print(df[["x", "y", "z", "err_mm", "move_time_s"]].describe())
```

```python
import numpy as np
data = np.genfromtxt("data/.../stream.csv", delimiter=",", names=True)
speed = np.hypot(data["vx"], data["vy"])
```
