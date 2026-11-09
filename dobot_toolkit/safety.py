"""
Yazılımsal güvenlik sınırları: çalışma alanı ve eklem limitleri.

Bunlar ROBOTUN donanım limitlerinin YERİNE GEÇMEZ; masa, fikstür, kamera
gibi engellere çarpmayı önlemek için ek bir katmandır. Varsayılan değerler
genel bir başlangıç noktasıdır — kendi düzeneğinize göre ölçüp ayarlayın.
"""
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np


class WorkspaceError(ValueError):
    """Hedef, yazılımsal çalışma alanı sınırlarının dışında."""


# Dobot Magician datasheet eklem aralıkları (derece) + ~5° tolerans.
# Datasheet: J1 -90..+90, J2 0..85, J3 -10..+95, J4 (servo) -90..+90.
# Kendi robotunuzda farklıysa Workspace(joint_limits=...) ile değiştirin.
JOINT_LIMITS = {
    "j1": (-95.0, 95.0),
    "j2": (-5.0, 90.0),
    "j3": (-15.0, 100.0),
    "j4": (-150.0, 150.0),
}


@dataclass
class Workspace:
    """
    Silindirik çalışma alanı:
        r_min <= hypot(x, y) <= r_max,  z_min <= z <= z_max
    """
    r_min: float = 120.0
    r_max: float = 315.0
    z_min: float = -70.0      # masa yüksekliğine göre AYARLAYIN
    z_max: float = 160.0
    max_step: Optional[float] = None   # tek hamlede izin verilen maks. XY mesafe (mm)
    joint_limits: dict = field(default_factory=lambda: dict(JOINT_LIMITS))

    def violations(self, x, y, z):
        r = float(np.hypot(x, y))
        out = []
        if r < self.r_min:
            out.append(f"r={r:.1f} < r_min={self.r_min}")
        if r > self.r_max:
            out.append(f"r={r:.1f} > r_max={self.r_max}")
        if z < self.z_min:
            out.append(f"z={z:.1f} < z_min={self.z_min}")
        if z > self.z_max:
            out.append(f"z={z:.1f} > z_max={self.z_max}")
        return out

    def contains(self, x, y, z) -> bool:
        return not self.violations(x, y, z)

    def check(self, x, y, z, current_xy: Optional[Tuple[float, float]] = None):
        """Sınır dışıysa WorkspaceError fırlatır."""
        v = self.violations(x, y, z)
        if self.max_step is not None and current_xy is not None:
            step = float(np.hypot(x - current_xy[0], y - current_xy[1]))
            if step > self.max_step:
                v.append(f"adım={step:.1f} mm > max_step={self.max_step}")
        if v:
            raise WorkspaceError(f"Hedef ({x:.1f}, {y:.1f}, {z:.1f}) reddedildi: " + ", ".join(v))

    def check_joints(self, j1, j2, j3, j4=0.0):
        v = []
        for name, val in zip(("j1", "j2", "j3", "j4"), (j1, j2, j3, j4)):
            lo, hi = self.joint_limits[name]
            if not lo <= val <= hi:
                v.append(f"{name}={val:.1f} ∉ [{lo}, {hi}]")
        if v:
            raise WorkspaceError("Eklem hedefi reddedildi: " + ", ".join(v))

    def clamp(self, x, y, z):
        """Hedefi sınırlara kırpar (aynı doğrultuda). (x, y, z, kırpıldı_mı) döner."""
        r = float(np.hypot(x, y))
        clipped = False
        if r < 1e-9:
            x, y, r, clipped = self.r_min, 0.0, self.r_min, True
        r_c = min(max(r, self.r_min), self.r_max)
        if abs(r_c - r) > 1e-9:
            s = r_c / r
            x, y, clipped = x * s, y * s, True
        z_c = min(max(z, self.z_min), self.z_max)
        if abs(z_c - z) > 1e-9:
            z, clipped = z_c, True
        return float(x), float(y), float(z), clipped
