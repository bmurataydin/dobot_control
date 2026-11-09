"""
Dobot Magician alarm kodları.

`GetAlarmsState` 16 bayt (128 bit) döndürür. Bayt i'nin bit j'si (LSB önce)
kod = i*8 + j alarmının aktif olduğunu gösterir. Örn. bayt 8, bit 0 -> 0x40.

Tablo Dobot Magician alarm dokümanındaki en sık görülen kodları içerir;
listede olmayan bir kod görürseniz Dobot "Alarm Description" belgesine bakın.
"""

ALARM_CODES = {
    0x00: "Sıfırlama (reset) alarmı — robot yeniden başlatıldı",
    0x01: "Tanımsız komut",
    0x02: "Dosya sistemi hatası",
    0x03: "MCU-FPGA iletişim hatası",
    0x04: "Açı sensörü okuma hatası",
    0x10: "Planlama: ters kinematik tekillik (singularity)",
    0x11: "Planlama: ters kinematik hesaplanamadı (hedef erişilemez)",
    0x12: "Planlama: ters kinematik eklem limiti aşıldı",
    0x13: "Planlama: tekrarlanan nokta verisi",
    0x14: "Planlama: ARC giriş parametresi hatası",
    0x15: "Planlama: JUMP parametresi hatası",
    0x20: "Hareket: ters kinematik tekillik",
    0x21: "Hareket: ters kinematik hesaplanamadı",
    0x22: "Hareket: eklem limiti aşıldı",
    0x30: "Eksen 1 aşırı hız",
    0x31: "Eksen 2 aşırı hız",
    0x32: "Eksen 3 aşırı hız",
    0x33: "Eksen 4 aşırı hız",
    0x40: "Limit: eksen 1 pozitif",
    0x41: "Limit: eksen 1 negatif",
    0x42: "Limit: eksen 2 pozitif",
    0x43: "Limit: eksen 2 negatif",
    0x44: "Limit: eksen 3 pozitif",
    0x45: "Limit: eksen 3 negatif",
    0x46: "Limit: eksen 4 pozitif",
    0x47: "Limit: eksen 4 negatif",
    0x48: "Limit: eksen 2-3 birleşik pozitif",
    0x49: "Limit: eksen 2-3 birleşik negatif",
    0x50: "Adım kaçırma: eksen 1",
    0x51: "Adım kaçırma: eksen 2",
    0x52: "Adım kaçırma: eksen 3",
    0x53: "Adım kaçırma: eksen 4",
}


def decode_alarms(alarm_bytes):
    """Ham alarm baytlarını aktif alarm kodları listesine çevirir."""
    codes = []
    for i, byte in enumerate(alarm_bytes or []):
        byte = int(byte)
        for bit in range(8):
            if byte & (1 << bit):
                codes.append(i * 8 + bit)
    return codes


def describe_alarm(code):
    return ALARM_CODES.get(code, "Bilinmeyen alarm (Dobot alarm belgesine bakın)")


def format_alarms(codes):
    if not codes:
        return "Alarm yok"
    return "; ".join(f"0x{c:02X}: {describe_alarm(c)}" for c in codes)
