# API başvuru kılavuzu

## `connect(port=None, dry_run=False, realtime=True, **kwargs) -> Dobot`

Robota bağlanır. `dry_run=True` ise sahte robot (`MockInterface`) kullanılır.
`kwargs` doğrudan `Dobot(...)`'a geçer.

## `class Dobot`

```python
Dobot(port=None, *, serial_timeout=5.0, wait_timeout=60.0, speed=50.0, acceleration=50.0,
      speed_ratio=50.0, workspace=<Workspace()>, clear_alarms=True, interface=None)
```

| Parametre | Açıklama |
|---|---|
| `port` | `"COM3"`, `"/dev/ttyUSB0"`; `None` → otomatik bul |
| `serial_timeout` | Tek bir yanıt için okuma zaman aşımı (sn). Robot yanıt vermezse kod asılı kalmaz |
| `wait_timeout` | Bir hareketin bitmesi için azami bekleme (sn) |
| `speed`, `acceleration`, `speed_ratio` | Başlangıç hız ayarları (bkz. `set_speed`) |
| `workspace` | `safety.Workspace`; `None` → yazılımsal sınır kontrolü kapalı |
| `clear_alarms` | Bağlanırken alarmları temizle |
| `interface` | Test için hazır arayüz (ör. `MockInterface()`) |

Bağlanırken: giriş tamponu temizlenir, kuyruk zorla durdurulup temizlenir ve yeniden başlatılır,
alarmlar temizlenir, hız ve jump parametreleri ayarlanır.

### Durum

| Metot | Dönüş | Açıklama |
|---|---|---|
| `get_pose()` | `Pose` | `x, y, z, r, j1, j2, j3, j4`; `.xyz`, `.xyzr`, `.joints` özellikleri |
| `info()` | `dict` | port, cihaz adı, seri no, firmware sürümü, poz |
| `get_alarms()` | `list[int]` | aktif alarm kodları |
| `clear_alarms()` | `list[int]` | temizler, kalan alarmları döndürür |
| `describe_alarms()` | `str` | okunabilir açıklama |
| `current_queue_index()` | `int` | firmware kuyruk sayacı |
| `connected()` | `bool` | seri port açık mı |
| `close()` | — | seri portu kapatır (robotu durdurmaz) |

### Parametreler

| Metot | Açıklama |
|---|---|
| `set_speed(velocity=50, acceleration=None, ratio=50)` | PTP eklem + kartezyen + CP hız/ivme. Gerçek hız ≈ `velocity × ratio / 100` |
| `set_jump_params(height=20, z_limit=100)` | JUMP modunda kalkma yüksekliği ve azami z |

### Hareket

Tüm hareketler varsayılan olarak **fiziksel olarak bitene kadar bekler** (`wait=True`) ve
robotun kuyruk indeksini döndürür. `wait=False` ile komut kuyruğa atılır; sonra `wait()` çağrılır.

| Metot | Açıklama |
|---|---|
| `home(wait=True, timeout=120)` | Referans hareketi (~20 sn) |
| `move_xyz(x, y, z, r=None, mode="movj", wait=True)` | Mutlak kartezyen. `mode`: `"movj"`, `"movl"`, `"jump"`. `r=None` → mevcut r |
| `move_linear(x, y, z, r=None)` | `move_xyz(mode="movl")` |
| `jump_to(x, y, z, r=None)` | `move_xyz(mode="jump")` |
| `move_relative(dx=0, dy=0, dz=0, dr=0)` | Kartezyen artış, düz çizgi (MOVL_INC) |
| `move_joints(j1, j2, j3, j4=None, mode="movj")` | Mutlak eklem açıları (derece) |
| `move_joints_relative(dj1=0, dj2=0, dj3=0, dj4=0)` | Eklem artışı (MOVJ_INC) |
| `follow_path(points, speed=50, relative=False, wait=True, lookahead=16)` | Sürekli yörünge (CP). Noktalar kuyruğa akıtılarak gönderilir; kuyruk taşmaz |
| `arc(via_point, end_point)` | Mevcut konum → `via` → `end` yayı; noktalar `(x, y, z, r)` |
| `jog(axis, positive=True, duration=0.3, joint=False)` | Ekseni belirli süre sürekli sür. `axis`: `'x','y','z','r'` veya 1..4 |
| `wait(timeout=None)` | Kuyruktaki tüm komutların bitmesini bekle |
| `wait_ms(ms)` | Robot tarafında kuyruklu gecikme |
| `stop()` | Yazılımsal acil durdurma (kuyruğu zorla durdur + temizle + yeniden başlat) |

`wait()` hataları:
- `DobotTimeoutError` — süre aşıldı (mesajda aktif alarmlar yazılır)
- `DobotAlarmError` — alarm var ve kuyruk ilerlemiyor (ör. limit alarmı)

### Uç efektörler

| Metot | Açıklama |
|---|---|
| `suction(on=True)` | Vakum aç / bırak |
| `gripper("close" \| "open" \| "off")` | Pnömatik tutucu |
| `laser(on=True)` | Lazer modülü |
| `end_effector_off()` | Vakum ve tutucu pompasını kapat |
| `set_digital_output(address, level)` | EIO dijital çıkış |

### Eski adlar (geriye dönük uyumluluk)

| Eski | Gerçek anlamı | Yeni karşılığı |
|---|---|---|
| `slide_to(j1, j2, j3, j4)` | MOVJ_ANGLE (eklem açısı!) | `move_joints(...)` |
| `move_to(j1, j2, j3, j4)` | JUMP_ANGLE (eklem açısı!) | `move_joints(..., mode="jump")` |
| `move_to_relative(dx, dy, dz, dr)` | MOVL_INC | `move_relative(...)` |
| `slide_to_relative(dj1, ...)` | MOVJ_INC | `move_joints_relative(...)` |

### Düşük seviye erişim

`bot.interface` orijinal `lib.interface.Interface` nesnesidir; protokoldeki tüm komutlara erişim sağlar:

```python
bot.interface.get_device_version()
bot.interface.set_point_to_point_command(mode, x, y, z, r)
bot.interface.set_handheld_teaching_state(True)
bot.interface.get_wifi_status()
```

## `PTPMode`

`JUMP_XYZ=0, MOVJ_XYZ=1, MOVL_XYZ=2, JUMP_ANGLE=3, MOVJ_ANGLE=4, MOVL_ANGLE=5, MOVJ_INC=6, MOVL_INC=7, MOVJ_XYZ_INC=8, JUMP_MOVL_XYZ=9`

## `DobotKinematics(L2=135, L3=147, L4=59.7, z_offset=0)`

| Metot | Açıklama |
|---|---|
| `forward(j1, j2, j3, degrees=True)` | → `(x, y, z)` mm |
| `inverse(x, y, z, degrees=True)` | → `(j1, j2, j3)`; erişilemezse `UnreachableError` |

## `Workspace` (safety)

```python
Workspace(r_min=120, r_max=315, z_min=-70, z_max=160, max_step=None, joint_limits=JOINT_LIMITS)
```

- `contains(x, y, z)`, `check(x, y, z)` (→ `WorkspaceError`), `check_joints(j1, j2, j3, j4)`, `clamp(x, y, z)`
- `max_step`: tek hamlede izin verilen azami XY mesafesi (mm)
- `bot.workspace = Workspace(z_min=-63)` ile değiştirin, `bot.workspace = None` ile kapatın

## `CsvLogger(path, extra_fields=(), compute_velocity=True)`

- `log(pose, t=None, **extra)` — her satır anında diske yazılır
- Context manager (`with ... as log:`)

## `PoseSampler(bot, hz=20, csv_path=None, extra=None)`

- Ayrı thread'de sabit frekansta `get_pose()` okur. `start()/stop()` veya `with`.
- `samples`: `[(t_unix, Pose), ...]`, `achieved_hz`, `errors`
- Gerçek robotta 30 Hz doğrulandı. Seri erişim kilitli olduğu için hareket komutlarıyla güvenle birlikte çalışır.

## `trajectories`

`line, circle, ellipse, polygon, rectangle, square, star, heart, figure_eight, grid, resample, make_shape(name, center, size, z, n)`.
Hepsi `(N, 3)` numpy dizisi döndürür. `SHAPES` sözlüğü `make_shape` için isimleri içerir.

## `camera.CameraGrabber(index=0, width=640, height=480, fps=30, autofocus=False, warmup=1.0)`

- `grab_fresh(after=None, timeout=1.0)` → `(frame, t)`: `after` anından **sonra** yakalanmış ilk kare
- `latest()`, `actual_size`, `frames_read`, `close()`; `list_cameras()`

## `mock.MockInterface(realtime=False, kinematics=None, time_scale=1.0)`

Gerçek arayüzle aynı metotlar; kuyruk/indeks semantiği firmware ile aynıdır.
`raise_alarm(code)` ile alarm senaryosu test edilebilir.

## `ports`

`list_serial_ports()`, `find_dobot_ports()`, `find_dobot_port()`; `python -m dobot_toolkit.ports`.

## `alarms`

`decode_alarms(bytes) -> list[int]`, `describe_alarm(code)`, `format_alarms(codes)`, `ALARM_CODES`.
