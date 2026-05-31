"""
plot_time_breakdown.py — Muestra dónde se va el tiempo (Insert vs Detect vs Resolve).

Uso:
    python plot_time_breakdown.py
"""

import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

# Config
INPUT_FILE  = Path(__file__).parent / "benchmark_realtime_insertion_output_2000.txt"
OUTPUT_FILE = Path(__file__).parent / "time_breakdown.pdf"
BG = "#FAFAFA"

# Leer datos
pattern_uav     = re.compile(r"UAV (\d+)")
pattern_insert  = re.compile(r"insert=\s*([\d.]+)\s*ms")
pattern_detect  = re.compile(r"detect=\s*([\d.]+)\s*ms")
pattern_resolve_loop = re.compile(r"resolve_loop=\s*([\d.]+)\s*ms")

indices = []
inserts = []
detects = []
resolves = []

with open(INPUT_FILE, encoding="utf-8", errors="ignore") as f:
    for line in f:
        if not line.startswith("UAV"):
            continue
            
        m_uav = pattern_uav.search(line)
        m_ins = pattern_insert.search(line)
        m_det = pattern_detect.search(line)
        m_res = pattern_resolve_loop.search(line)
        
        if m_uav and m_ins and m_det:
            indices.append(int(m_uav.group(1)))
            inserts.append(float(m_ins.group(1)))
            detects.append(float(m_det.group(1)))
            if m_res:
                resolves.append(float(m_res.group(1)))
            else:
                resolves.append(0.0)

# Suavizar para que el area plot quede bien
window = 10
inserts_sm = np.convolve(inserts, np.ones(window)/window, mode='valid')
detects_sm = np.convolve(detects, np.ones(window)/window, mode='valid')
resolves_sm = np.convolve(resolves, np.ones(window)/window, mode='valid')
idx_sm = indices[window-1:]

# Plot
fig, ax = plt.subplots(figsize=(10, 5), facecolor=BG)
ax.set_facecolor(BG)

ax.stackplot(idx_sm, inserts_sm, detects_sm, resolves_sm,
             labels=['R-Tree Registration', 'Conflict Detection', 'Resolution Loop'],
             colors=['#A8DADC', '#457B9D', '#E63946'],
             alpha=0.8)

ax.set_xlabel("Fleet Size $N$ (Number of inserted UAVs)", fontsize=11)
ax.set_ylabel("Processing Time (ms) - Smoothed", fontsize=11)
ax.set_title("Time Breakdown: Where is compute spent? (2\u202f000 UAVs Incremental)", fontsize=12)

# Escala log en Y para que se vea el resolve disparado sin ocultar la base
ax.set_yscale('symlog', linthresh=5000)

ax.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
ax.legend(loc='upper left', fontsize=10, framealpha=0.9)

plt.tight_layout()
fig.savefig(OUTPUT_FILE, dpi=200, bbox_inches="tight")
print(f"Figura guardada en: {OUTPUT_FILE}")
