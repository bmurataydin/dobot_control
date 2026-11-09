"""
Yörünge / şekil üreticileri. Hepsi (N, 3) boyutlu numpy dizisi döndürür:
her satır bir (x, y, z) noktası (mm).

    from dobot_toolkit import trajectories as tr
    pts = tr.circle(center=(220, 0), radius=30, z=-50, n=100)
    bot.follow_path(pts)
"""
import numpy as np


def _with_z(xy, z):
    xy = np.asarray(xy, dtype=float)
    return np.column_stack([xy, np.full(len(xy), float(z))])


def _densify(corners, n):
    """Köşe listesini kenar uzunluklarıyla orantılı n noktaya örnekler (kapalı/açık)."""
    corners = np.asarray(corners, dtype=float)
    seg = np.diff(corners, axis=0)
    lengths = np.hypot(seg[:, 0], seg[:, 1])
    cum = np.concatenate([[0.0], np.cumsum(lengths)])
    s = np.linspace(0.0, cum[-1], n)
    x = np.interp(s, cum, corners[:, 0])
    y = np.interp(s, cum, corners[:, 1])
    return np.column_stack([x, y])


def line(p0, p1, n=50):
    """İki 3B nokta arasında doğru."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    s = np.linspace(0, 1, n)[:, None]
    return p0 + s * (p1 - p0)


def circle(center, radius, z, n=100, start_angle=0.0, clockwise=False):
    a = np.linspace(0, 2 * np.pi, n) * (-1 if clockwise else 1) + start_angle
    return _with_z(np.column_stack([center[0] + radius * np.cos(a),
                                    center[1] + radius * np.sin(a)]), z)


def ellipse(center, rx, ry, z, n=100):
    a = np.linspace(0, 2 * np.pi, n)
    return _with_z(np.column_stack([center[0] + rx * np.cos(a), center[1] + ry * np.sin(a)]), z)


def polygon(center, radius, sides, z, n=100, rotation=0.0):
    """Düzgün çokgen (üçgen=3, beşgen=5, ...)."""
    a = np.linspace(0, 2 * np.pi, sides + 1) + rotation
    corners = np.column_stack([center[0] + radius * np.cos(a), center[1] + radius * np.sin(a)])
    return _with_z(_densify(corners, n), z)


def rectangle(center, width, height, z, n=100):
    cx, cy = center
    w, h = width / 2.0, height / 2.0
    corners = [(cx - w, cy - h), (cx + w, cy - h), (cx + w, cy + h), (cx - w, cy + h), (cx - w, cy - h)]
    return _with_z(_densify(corners, n), z)


def square(center, side, z, n=100):
    return rectangle(center, side, side, z, n)


def star(center, radius, z, n=100, points=5, inner_ratio=0.4):
    k = 2 * points
    a = np.linspace(0, 2 * np.pi, k + 1) + np.pi / 2
    r = np.where(np.arange(k + 1) % 2 == 0, radius, radius * inner_ratio)
    corners = np.column_stack([center[0] + r * np.cos(a), center[1] + r * np.sin(a)])
    return _with_z(_densify(corners, n), z)


def heart(center, size, z, n=100):
    t = np.linspace(0, 2 * np.pi, n)
    x = 16 * np.sin(t) ** 3
    y = 13 * np.cos(t) - 5 * np.cos(2 * t) - 2 * np.cos(3 * t) - np.cos(4 * t)
    return _with_z(np.column_stack([center[0] + x / 16.0 * size, center[1] + y / 16.0 * size]), z)


def figure_eight(center, size, z, n=100):
    t = np.linspace(0, 2 * np.pi, n)
    return _with_z(np.column_stack([center[0] + size * np.sin(t),
                                    center[1] + size * np.sin(t) * np.cos(t)]), z)


def grid(x_range, y_range, z, nx=3, ny=3, serpentine=True):
    """Izgara noktaları (tekrarlanabilirlik / kalibrasyon testleri için)."""
    xs = np.linspace(*x_range, nx)
    ys = np.linspace(*y_range, ny)
    pts = []
    for i, x in enumerate(xs):
        col = ys if (not serpentine or i % 2 == 0) else ys[::-1]
        pts.extend((x, y, z) for y in col)
    return np.array(pts, dtype=float)


def resample(points, spacing_mm):
    """Yörüngeyi yaklaşık eşit aralıklı (spacing_mm) noktalara yeniden örnekler."""
    p = np.asarray(points, dtype=float)
    d = np.linalg.norm(np.diff(p, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(d)])
    n = max(2, int(np.ceil(cum[-1] / spacing_mm)) + 1)
    s = np.linspace(0, cum[-1], n)
    return np.column_stack([np.interp(s, cum, p[:, k]) for k in range(p.shape[1])])


SHAPES = {
    "circle": lambda c, s, z, n: circle(c, s, z, n),
    "square": lambda c, s, z, n: square(c, 2 * s, z, n),
    "triangle": lambda c, s, z, n: polygon(c, s, 3, z, n, rotation=np.pi / 2),
    "pentagon": lambda c, s, z, n: polygon(c, s, 5, z, n, rotation=np.pi / 2),
    "hexagon": lambda c, s, z, n: polygon(c, s, 6, z, n),
    "star": lambda c, s, z, n: star(c, s, z, n),
    "heart": lambda c, s, z, n: heart(c, s, z, n),
    "ellipse": lambda c, s, z, n: ellipse(c, s, s * 0.6, z, n),
    "figure_eight": lambda c, s, z, n: figure_eight(c, s, z, n),
}


def make_shape(name, center, size, z, n=100):
    """İsimle şekil üret: make_shape('star', (220, 0), 30, -50)."""
    if name not in SHAPES:
        raise ValueError(f"Bilinmeyen şekil '{name}'. Seçenekler: {sorted(SHAPES)}")
    return SHAPES[name](center, size, z, n)
