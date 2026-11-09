"""
Dobot Magician için basitleştirilmiş ileri / ters kinematik.

Açı tanımları (Dobot'un get_pose ile döndürdüğü j1..j3 ile aynı):
    j1 : taban dönüşü (yatay düzlemde, +x ekseninden)
    j2 : arka kol (rear arm) açısı — DİKEYDEN ölçülür
    j3 : ön kol (forearm) açısı   — YATAYDAN ölçülür

    yatay uzanım  Lx = L2*sin(j2) + L3*cos(j3) + L4
    x = Lx*cos(j1),  y = Lx*sin(j1),  z = L2*cos(j2) - L3*sin(j3) + z_offset

L2 = arka kol (135 mm), L3 = ön kol (147 mm), L4 = bilekten uç efektör
ucuna yatay ofset. L4=59.7 ile model, robotun get_pose() ile bildirdiği x/y/z'yi
birebir verir (home: j2=j3=45° -> x=259.1, z=-8.5). Kalem ucu gibi fiziksel
ofseti farklı bir uç kullanıyorsanız L4'ü ona göre değiştirin.

ÖNEMLİ: Robot XYZ komutlarında IK'yı kendi içinde çözer; bu modül analiz,
sahte robot (mock) ve eklem-uzayı komutları içindir. Kendi düzeneğiniz için
`examples/10_kinematics_check.py` ile L4/z_offset değerlerini doğrulayın.
"""
from dataclasses import dataclass
from typing import Tuple

import numpy as np


class UnreachableError(ValueError):
    """Hedef nokta kinematik olarak erişilemez."""


@dataclass
class DobotKinematics:
    L2: float = 135.0
    L3: float = 147.0
    L4: float = 59.7      # Dobot'un varsayılan uç ofseti (gerçek robotun get_pose'u ile doğrulandı)
    z_offset: float = 0.0

    def forward(self, j1, j2, j3, degrees=True) -> Tuple[float, float, float]:
        """Eklem açılarından (j1, j2, j3) uç nokta (x, y, z) [mm]."""
        if degrees:
            j1, j2, j3 = np.deg2rad([j1, j2, j3])
        lx = self.L2 * np.sin(j2) + self.L3 * np.cos(j3) + self.L4
        x = lx * np.cos(j1)
        y = lx * np.sin(j1)
        z = self.L2 * np.cos(j2) - self.L3 * np.sin(j3) + self.z_offset
        return float(x), float(y), float(z)

    def inverse(self, x, y, z, degrees=True) -> Tuple[float, float, float]:
        """(x, y, z) [mm] -> (j1, j2, j3). Erişilemezse UnreachableError."""
        z = z - self.z_offset
        j1 = np.arctan2(y, x)
        lx = np.hypot(x, y) - self.L4
        r = np.hypot(lx, z)
        if r < 1e-9:
            raise UnreachableError(f"Hedef ({x:.1f}, {y:.1f}, {z:.1f}) omuz eksenine çakışık.")
        alpha = np.arctan2(lx, z)
        cos_beta = (r ** 2 + self.L2 ** 2 - self.L3 ** 2) / (2 * r * self.L2)
        if not -1.0 <= cos_beta <= 1.0:
            raise UnreachableError(f"Hedef ({x:.1f}, {y:.1f}, {z:.1f}) erişim alanı dışında.")
        j2 = alpha - np.arccos(cos_beta)
        j3 = np.arctan2(self.L2 * np.cos(j2) - z, lx - self.L2 * np.sin(j2))
        if degrees:
            return tuple(float(v) for v in np.rad2deg([j1, j2, j3]))
        return float(j1), float(j2), float(j3)
