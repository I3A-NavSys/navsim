"""
plot_latency_scatter_sequential.py — Genera un scatter plot de latencias vs numero de UAV.

Lee los ficheros de benchmark sequential (1 y 2000) y muestra cómo evoluciona
el tiempo de procesamiento a medida que se insertan más UAVs.

Uso:
    python plot_latency_scatter_sequential.py
"""

import re
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
FILE_SEQ_500  = Path(__file__).parent / "benchmark_realtime_sequential_conflicts_output_1.txt"
FILE_SEQ_2000 = Path(__file__).parent / "benchmark_realtime_sequential_conflicts_output2000.txt"
OUTPUT_FILE   = Path(__file__).parent / "latency_scatter_sequential.pdf"

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

idx_500, lat_500 = load_data(FILE_SEQ_500)
idx_2000, lat_2000 = load_data(FILE_SEQ_2000)

# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(10, 5), facecolor=BG)
ax.set_facecolor(BG)

# Usamos scatter plot con puntos pequeños y semi-transparentes
if idx_500:
    ax.scatter(idx_500, lat_500, color=COLOR_NORMAL, s=15, alpha=0.6, edgecolors='none', label="Sequential (500 UAVs)")
if idx_2000:
    ax.scatter(idx_2000, lat_2000, color=COLOR_FAST, s=15, alpha=0.6, edgecolors='none', label="Sequential (2\u202f000 UAVs)")

# --- Formato ejes ---
ax.set_xlabel("UAV Insertion Sequence (Chronological)", fontsize=11)
ax.set_ylabel("Total processing time per UAV (ms, log scale)", fontsize=11)
ax.set_title("Processing Time Evolution \u2014 Sequential Benchmark", fontsize=12)

# Eje Y logarítmico porque los tiempos varían muchísimo
ax.set_yscale('log')

ax.grid(True, which="both", linestyle="--", linewidth=0.5, alpha=0.5)
ax.legend(fontsize=10, framealpha=0.9, markerscale=2)

plt.tight_layout()
fig.savefig(OUTPUT_FILE, dpi=200, bbox_inches="tight")
print(f"\nFigura guardada en: {OUTPUT_FILE}")
plt.show()
