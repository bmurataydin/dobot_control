"""
Dobot Magician'ın bağlı olduğu seri portu bulma yardımcıları.

Dobot Magician USB-seri dönüştürücü olarak genellikle CH340/CH9102 (VID 0x1A86) veya
Silicon Labs CP210x (VID 0x10C4) yongası kullanır.
"""
from serial.tools import list_ports

KNOWN_VIDS = {0x1A86, 0x10C4}
KNOWN_KEYWORDS = ("CH340", "CH341", "CH910", "CP210", "SILICON LABS", "USB-SERIAL", "USB SERIAL")


def list_serial_ports():
    """[(device, description, hwid), ...]"""
    return [(p.device, p.description, p.hwid) for p in list_ports.comports()]


def find_dobot_ports():
    """Dobot olma ihtimali yüksek portların listesi."""
    found = []
    for p in list_ports.comports():
        text = f"{p.description} {p.manufacturer or ''} {p.hwid}".upper()
        if (p.vid in KNOWN_VIDS) or any(k in text for k in KNOWN_KEYWORDS):
            found.append(p.device)
    return found


def find_dobot_port():
    """
    Tek bir aday port varsa onu döndürür; yoksa veya birden fazlaysa
    anlaşılır bir hata ile hangi portların görüldüğünü listeler.
    """
    candidates = find_dobot_ports()
    if len(candidates) == 1:
        return candidates[0]
    all_ports = "\n".join(f"  {d}  —  {desc}" for d, desc, _ in list_serial_ports()) or "  (hiç port yok)"
    if not candidates:
        raise RuntimeError(
            "Dobot portu otomatik bulunamadı. Sürücüyü (CH340/CP210x) kurup USB kablosunu "
            "kontrol edin veya portu elle verin (--port COM3).\nGörülen portlar:\n" + all_ports)
    raise RuntimeError(
        f"Birden fazla aday port bulundu: {candidates}. Portu elle verin (--port ...).\n"
        "Görülen portlar:\n" + all_ports)


if __name__ == "__main__":
    for dev, desc, hwid in list_serial_ports():
        mark = "  <-- Dobot adayı" if dev in find_dobot_ports() else ""
        print(f"{dev:12s} {desc}  [{hwid}]{mark}")
