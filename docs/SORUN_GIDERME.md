# Sorun giderme

## Port açılmıyor: `could not open port 'COM3': PermissionError / Access is denied`

- Portu başka bir program kullanıyor: **DobotStudio / DobotLab**, Arduino IDE seri monitörü, başka
  bir Python oturumu (ör. PyCharm'da hâlâ çalışan / asılı kalan script). Kapatın.
- Linux: kullanıcı `dialout` grubunda değil (bkz. [KURULUM.md](KURULUM.md)).

## Port görünmüyor

- Sürücü eksik (CH340 / CH9102 / CP210x). `python -m dobot_toolkit.ports` ile listeleyin.
- Sadece şarj edebilen (veri hattı olmayan) USB kablosu kullanılıyor olabilir.
- Robot kapalı.

## `IOError: Dobot yanit vermedi` / `DobotTimeoutError`

- Seri tamponda senkron kaybı: script başka bir anda kesildiyse robot yarım yanıt göndermiş olabilir.
  `Dobot(...)` bağlanırken tamponu temizler; tekrar bağlanmayı deneyin.
- Sürmüyorsa robotu **kapatıp açın** (güç döngüsü). Firmware kuyruk sayacı bazen ancak bu şekilde sıfırlanır.
- Baud hızı 115200 olmalı (`lib/interface.py`).

## Kod `wait()`'te takılıyor gibi

Takılmaz — `wait_timeout` (varsayılan 60 sn) sonunda `DobotTimeoutError` fırlatılır. Sebepler:

- Robot alarmda (kırmızı LED). `bot.describe_alarms()` ile bakın, `bot.clear_alarms()` ile temizleyin.
- Hedef erişilemez → planlama alarmı (0x10–0x15); komut yürütülmez.
- Kuyruk durdurulmuş (`stop_queue`) ve yeniden başlatılmamış → `bot.interface.start_queue()`.

## Kırmızı LED / alarm

| Alarm | Tipik sebep | Çözüm |
|---|---|---|
| 0x10–0x12, 0x20–0x22 | Hedef erişim alanı dışında / tekillik / eklem limiti | Hedefi değiştirin; `clear_alarms()` |
| 0x40–0x49 | Eksen limiti | Kolu elle (kilit düğmesi) güvenli bölgeye alın, `clear_alarms()`, `home()` |
| 0x50–0x53 | Adım kaçırma | Hızı/ivmeyi düşürün, yükü azaltın, `home()` |
| 0x00 | Reset alarmı (açılışta) | Zararsız; `clear_alarms()` |

## Robot hedefe gidiyor ama konum kayık

- Uç efektör farklı (kalem, özel tutucu): robotun dahili ofseti (59.7 mm) uca uymuyor. `examples/10_kinematics_check.py`
  ve gerekirse kendi ölçtüğünüz ofsetle düzeltin.
- Adım kaçırma birikti: `home()` yapın.
- Masa yamuk: z hatası konuma göre değişiyorsa masayı veya kağıdı düzeltin, ya da z için düzlem kalibrasyonu yapın.

## `WorkspaceError: Hedef ... reddedildi`

Yazılımsal güvenlik sınırı. Hedef gerçekten güvenliyse sınırı değiştirin:

```python
from dobot_toolkit import Workspace
bot.workspace = Workspace(z_min=-75, r_max=320)
```

Tamamen kapatmak için `bot.workspace = None` (önerilmez).

## Hız ayarı etki etmiyor

Hız parametreleri **kuyruklu** komutlardır; bir sonraki hareketten itibaren geçerli olur.
Gerçek hız ≈ `velocity × ratio / 100`; `ratio` düşükse (`--ratio`) yüksek `velocity` de yavaş kalır.

## Windows'ta Ctrl+C scripti durdurmuyor

Seri okuma sırasında Windows Ctrl+C'yi geciktirebilir; `serial_timeout` (5 sn) içinde döner.
Robotun durması gerekiyorsa: güç düğmesi. Yazılımla: `bot.stop()`.
