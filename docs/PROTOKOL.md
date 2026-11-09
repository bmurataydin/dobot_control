# Dobot Magician seri protokolü

Bağlantı: **115200 baud, 8N1**, akış kontrolü yok.

## Paket formatı

```
| 0xAA | 0xAA | Len | ID | Ctrl | Params ... | Checksum |
   başlık       1B    1B   1B     Len-2 bayt     1B
```

- `Len` = 2 + params uzunluğu (ID + Ctrl + Params)
- `Ctrl` bit0 = `rw` (1: yaz/set, 0: oku/get), bit1 = `isQueued` (1: komut kuyruğuna ekle)
- `Checksum` = `(256 - (ID + Ctrl + ΣParams) mod 256) mod 256`
- Sayılar little-endian; `float` = 4 bayt IEEE-754

Robot her isteğe aynı ID ile yanıt verir. Kuyruklu komutların yanıtı, komutun kuyruktaki
**indeksini** (uint64) içerir.

Örnek — `SetPTPCmd(MOVJ_XYZ, 200, 0, 10, 0)`, kuyruklu:

```
AA AA 13 54 03 01 00 00 48 43 00 00 00 00 00 00 20 41 00 00 00 00 CS
            ^ID=84 ^ctrl=rw|queued ^mode ^x=200.0f     ^y=0     ^z=10.0f     ^r=0
```

## Sık kullanılan komutlar

| ID | Komut | Params | Kuyruk |
|---|---|---|---|
| 0 / 1 / 2 | Seri no / cihaz adı / firmware sürümü | — | hayır |
| 10 | GetPose | → 8 float: x, y, z, r, j1, j2, j3, j4 | hayır |
| 20 / 21 | Alarm durumu oku (16 bayt) / temizle | — | hayır |
| 31 | Home komutu | uint32 (0) | evet |
| 40–42 | Elle öğretme modu/durumu/tetik | — | hayır |
| 61 / 62 / 63 | Lazer / vakum / tutucu | ctrlEnable, on | evet |
| 70–73 | JOG parametreleri / komutu | | |
| 80 | PTP eklem hız+ivme | 8 float | evet |
| 81 | PTP kartezyen hız+ivme | 4 float | evet |
| 82 | PTP jump yüksekliği, zLimit | 2 float | evet |
| 83 | PTP ortak hız/ivme oranı | 2 float | evet |
| 84 | **PTP komutu** | mode(uint8) + 4 float | evet |
| 90 / 91 | CP parametreleri / **CP komutu** | | evet |
| 100 / 101 | ARC parametreleri / komutu | | evet |
| 110 | WAIT (ms) | uint32 | evet |
| 131 | Dijital çıkış | addr, level | evet |
| 240 / 241 / 242 | Kuyruğu başlat / durdur / zorla durdur | — | hayır |
| 245 | Kuyruğu temizle | — | hayır |
| 246 | Mevcut kuyruk indeksi | → uint64 | hayır |

## PTP modları (ID 84)

| Mod | Ad | Params anlamı |
|---|---|---|
| 0 | JUMP_XYZ | x, y, z, r (mutlak) |
| 1 | MOVJ_XYZ | x, y, z, r (mutlak) |
| 2 | MOVL_XYZ | x, y, z, r (mutlak) |
| 3 | JUMP_ANGLE | j1, j2, j3, j4 (mutlak) |
| 4 | MOVJ_ANGLE | j1, j2, j3, j4 (mutlak) |
| 5 | MOVL_ANGLE | j1, j2, j3, j4 (mutlak) |
| 6 | MOVJ_INC | Δj1, Δj2, Δj3, Δj4 |
| 7 | MOVL_INC | Δx, Δy, Δz, Δr |
| 8 | MOVJ_XYZ_INC | Δx, Δy, Δz, Δr |
| 9 | JUMP_MOVL_XYZ | x, y, z, r |

## Kuyruk ve "hareket bitti mi?" mantığı

Hareket komutları robotun komut kuyruğuna girer ve sırayla yürütülür. Komut gönderildiğinde
yanıt hemen gelir; hareketin **bittiğini** anlamak için:

1. Kuyruğa 0 ms'lik bir `WAIT` eklenir → yanıtında indeks `N` gelir.
2. `GetQueuedCmdCurrentIndex` (246) okunur. Bu sayaç, yürütülmekte olan / son başlayan
   komutun indeksidir.
3. Sayaç `N`'e ulaştığında WAIT başlamış, yani **önceki tüm hareketler bitmiştir**.

Doğrudan hareket komutunun indeksini beklemek yanlıştır: sayaç o indekse hareket
**başladığında** ulaşır. `Dobot.wait()` bu yüzden WAIT yöntemini kullanır, ayrıca zaman aşımı
ve alarm kontrolü yapar.

Kuyruk kapasitesi sınırlıdır. `follow_path`, noktaları `lookahead` (varsayılan 16) adet önde
olacak şekilde akıtarak gönderir.

## Alarm bitleri

Alarm durumu 16 bayttır. Bayt `i`'nin bit `j`'si → alarm kodu `i*8 + j`. Kodlar ve açıklamalar
için bkz. `dobot_toolkit/alarms.py`.
