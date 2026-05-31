"""
plot_detection_growth.py — Muestra cómo el tiempo de detección crece como O(log N).

Uso:
    python plot_detection_growth.py
"""

import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

# Config
INPUT_FILE  = Path(__file__).parent / "benchmark_realtime_insertion_output_2000.txt"
OUTPUT_FILE = Path(__file__).parent / "detection_growth.pdf"
BG = "#FAFAFA"

# Leer datos
pattern_uav = re.compile(r"UAV (\d+)")
pattern_detect = re.compile(r"detect=\s*([\d.]+)\s*ms")

indices = []
detects = []

with open(INPUT_FILE, encoding="utf-8", errors="ignore") as f:
    for line in f:
        m_uav = pattern_uav.search(line)
        m_det = pattern_detect.search(line)
        if m_uav and m_det:
            indices.append(int(m_uav.group(1)))
            detects.append(float(m_det.group(1)))

# Suavizar un poco los datos para que se vea la tendencia (media móvil de 20)
window_size = 20
detects_smooth = np.convolve(detects, np.ones(window_size)/window_size, mode='valid')
indices_smooth = indices[window_size-1:]

# Plot
fig, ax = plt.subplots(figsize=(8, 4.5), facecolor=BG)
ax.set_facecolor(BG)

ax.scatter(indices, detects, color="#4C9BE8", s=5, alpha=0.3, edgecolors='none', label="Raw detection time")
ax.plot(indices_smooth, detects_smooth, color="#E63946", linewidth=2, label="Moving average (Trend)")

ax.set_xlabel("Fleet Size $N$ (Number of inserted UAVs)", fontsize=11)
ax.set_ylabel("Detection time (ms)", fontsize=11)
ax.set_title("Conflict Detection Time Growth \u2014 Empirical $O(\\log N)$ Validation", fontsize=12)

ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
ax.legend(fontsize=10, framealpha=0.9)

# Limitar Y para no coger picos de basura extrema si los hay
max_expected = np.percentile(detects, 99) * 1.5
ax.set_ylim(0, max_expected)

plt.tight_layout()
fig.savefig(OUTPUT_FILE, dpi=200, bbox_inches="tight")
print(f"Figura guardada en: {OUTPUT_FILE}")
