"""
plot_latency_scatter.py — Genera un scatter plot de latencias vs numero de UAV.

Lee los ficheros de benchmark incremental (normal y fast) y muestra cómo evoluciona
el tiempo de procesamiento a medida que se insertan más UAVs.

Uso:
    python plot_latency_scatter.py
"""

import re
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
FILE_NORMAL = Path(__file__).parent / "benchmark_realtime_insertion_output_2000.txt"
FILE_FAST   = Path(__file__).parent / "benchmark_realtime_insertion_output_2000_fast.txt"
OUTPUT_FILE = Path(__file__).parent / "latency_scatter.pdf"

COLOR_NORMAL = "#4C9BE8"   # azul
COLOR_FAST   = "#F4A261"   # naranja
BG           = "#FAFAFA"

# ---------------------------------------------------------------------------
# Leer tiempos
# ---------------------------------------------------------------------------
pattern_uav = re.compile(r"UAV (\d+)")
pattern_total = re.compile(r"total=\s*([\d.]+)\s*ms")

def load_data(filepath):
    indices = []
    latencies = []
    if not filepath.exists():
        print(f"Warning: File not found: {filepath}")
        return indices, latencies
        
    with open(filepath, encoding="utf-8", errors="ignore") as f:
        for line in f:
            if not line.startswith("UAV"):
                continue
            
            m_uav = pattern_uav.search(line)
            m_total = pattern_total.search(line)
            
            if m_uav and m_total:
                indices.append(int(m_uav.group(1)))
                latencies.append(float(m_total.group(1)))
                
    return indices, latencies

idx_normal, lat_normal = load_data(FILE_NORMAL)
idx_fast, lat_fast = load_data(FILE_FAST)

# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(10, 5), facecolor=BG)
ax.set_facecolor(BG)

# Usamos scatter plot con puntos pequeños y semi-transparentes
if idx_normal:
    ax.scatter(idx_normal, lat_normal, color=COLOR_NORMAL, s=15, alpha=0.6, edgecolors='none', label="Normal Run")
if idx_fast:
    ax.scatter(idx_fast, lat_fast, color=COLOR_FAST, s=15, alpha=0.6, edgecolors='none', label="Fast Run")

# --- Formato ejes ---
ax.set_xlabel("UAV Insertion Sequence (Chronological)", fontsize=11)
ax.set_ylabel("Total processing time (ms, log scale)", fontsize=11)
ax.set_title("Processing Time Evolution — Incremental Benchmark (2\u202f000 UAVs)", fontsize=12)

# Eje Y logarítmico porque los tiempos varían muchísimo
ax.set_yscale('log')

ax.grid(True, which="both", linestyle="--", linewidth=0.5, alpha=0.5)
ax.legend(fontsize=10, framealpha=0.9, markerscale=2)

plt.tight_layout()
fig.savefig(OUTPUT_FILE, dpi=200, bbox_inches="tight")
print(f"\nFigura guardada en: {OUTPUT_FILE}")
plt.show()
