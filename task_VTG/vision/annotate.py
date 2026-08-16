"""
task_VTG/vision/annotate.py
===========================
M5：Demo 画面一 — RGB 叠加检测点与基座系坐标文字。
"""

from __future__ import annotations

import os
from typing import Any

import numpy as np

from vision.camera import save_rgb


def annotate_rgb(
    rgb: Any,
    items: list[dict[str, Any]],
    *,
    title: str | None = None,
) -> np.ndarray:
    """
    在 RGB 上画圆心 + 标签 + base 坐标。

    items 每项建议含: u, v, label, xyz_base (len-3)；可选 color。
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_agg import FigureCanvasAgg

    img = np.asarray(rgb, dtype=np.uint8)
    h, w = img.shape[:2]
    dpi = 100
    fig = plt.figure(figsize=(w / dpi, h / dpi), dpi=dpi)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(img)
    ax.set_axis_off()

    palette = {"red": "red", "yellow": "gold", "hard": "red", "soft": "gold"}
    for it in items:
        u = float(it["u"])
        v = float(it["v"])
        label = str(it.get("label", ""))
        key = str(it.get("color") or it.get("kind") or label).lower()
        c = "white"
        for k, col in palette.items():
            if k in key:
                c = col
                break
        ax.plot(u, v, "o", ms=12, mfc="none", mec=c, mew=2.2)
        ax.plot(u, v, ".", ms=5, color=c)
        # 小框
        s = 18
        ax.plot(
            [u - s, u + s, u + s, u - s, u - s],
            [v - s, v - s, v + s, v + s, v - s],
            color=c,
            lw=1.2,
            alpha=0.85,
        )
        xyz = it.get("xyz_base")
        if xyz is not None:
            p = np.asarray(xyz, dtype=float).reshape(-1)
            text = f"{label} base=({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})"
        else:
            text = label
        ax.text(
            u + 10,
            v - 10,
            text,
            color=c,
            fontsize=9,
            bbox=dict(boxstyle="round,pad=0.25", fc="black", alpha=0.5, ec="none"),
        )

    if title:
        ax.text(
            8,
            18,
            title,
            color="white",
            fontsize=11,
            fontweight="bold",
            bbox=dict(boxstyle="round,pad=0.3", fc="black", alpha=0.55, ec="none"),
        )

    canvas = FigureCanvasAgg(fig)
    canvas.draw()
    buf = np.asarray(canvas.buffer_rgba())[:, :, :3].copy()
    plt.close(fig)
    if buf.shape[0] != h or buf.shape[1] != w:
        ys = (np.linspace(0, buf.shape[0] - 1, h)).astype(int)
        xs = (np.linspace(0, buf.shape[1] - 1, w)).astype(int)
        buf = buf[ys][:, xs]
    return buf


def annotate_from_targets(
    rgb: Any,
    targets: list[Any],
    *,
    title: str | None = None,
) -> np.ndarray:
    """VisionTarget 列表 → 标注图（需含 u,v,xyz_base）。"""
    items = []
    for t in targets:
        items.append(
            {
                "u": getattr(t, "u", 0.0),
                "v": getattr(t, "v", 0.0),
                "label": getattr(t, "label", getattr(t, "color", "")),
                "color": getattr(t, "color", ""),
                "kind": getattr(t, "kind", ""),
                "xyz_base": getattr(t, "xyz_base", None),
            }
        )
    return annotate_rgb(rgb, items, title=title)


def save_annotation(path: str, rgb_annot: Any) -> str:
    """保存标注 PNG，返回绝对路径。"""
    path = os.path.abspath(path)
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    return save_rgb(path, rgb_annot)
