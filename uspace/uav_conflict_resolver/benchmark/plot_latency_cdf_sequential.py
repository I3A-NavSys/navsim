"""
plot_latency_cdf_sequential.py — Genera la CDF de latencias de los benchmarks sequential.

Lee los ficheros benchmark_realtime_sequential_conflicts_output_1.txt y
benchmark_realtime_sequential_conflicts_output2000.txt.
Genera la CDF para el de 2000 UAVs (el más representativo para la CDF).

Uso:
    python plot_latency_cdf_sequential.py
"""

import re
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
INPUT_FILE  = Path(__file__).parent / "benchmark_realtime_sequential_conflicts_output2000.txt"
OUTPUT_FILE = Path(__file__).parent / "latency_cdf_sequential.pdf"

# Colores del tema
COLOR_MAIN   = "#4C9BE8"   # azul principal
COLOR_MED    = "#F4A261"   # naranja para mediana
COLOR_P95    = "#E63946"   # rojo para P95
BG           = "#FAFAFA"

# ---------------------------------------------------------------------------
# Leer tiempos
# ---------------------------------------------------------------------------
pattern = re.compile(r"total=\s*([\d.]+)\s*ms")
latencies = []

if not INPUT_FILE.exists():
    print(f"Error: No se encuentra el fichero {INPUT_FILE}")
    exit(1)

with open(INPUT_FILE, encoding="utf-8", errors="ignore") as f:
    for line in f:
        # Solo líneas de UAV (empiezan con "UAV XXXX |")
        if not line.startswith("UAV"):
            continue
        m = pattern.search(line)
        if m:
            latencies.append(float(m.group(1)))

if not latencies:
    raise RuntimeError(f"No se encontraron datos en {INPUT_FILE}")

latencies = np.array(sorted(latencies))
n         = len(latencies)
cdf       = np.arange(1, n + 1) / n * 100.0   # en porcentaje

median = float(np.median(latencies))
p95    = float(np.percentile(latencies, 95))
p99    = float(np.percentile(latencies, 99))

print(f"UAVs leídos:    {n}")
print(f"Mediana:        {median:,.1f} ms")
print(f"P95:            {p95:,.1f} ms")
print(f"P99:            {p99:,.1f} ms")
print(f"Peor caso:      {latencies[-1]:,.1f} ms")

# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(8, 4.5), facecolor=BG)
ax.set_facecolor(BG)

# --- CDF principal (escala log en X para ver la cola) ---
ax.semilogx(latencies, cdf, color=COLOR_MAIN, linewidth=2, label="CDF")

# --- Mediana ---
ax.axvline(median, color=COLOR_MED, linestyle="--", linewidth=1.4,
           label=f"Median  {median/1000:.2f} s")
ax.axhline(50, color=COLOR_MED, linestyle=":", linewidth=0.8, alpha=0.6)

# --- P95 ---
ax.axvline(p95, color=COLOR_P95, linestyle="--", linewidth=1.4,
           label=f"P95  {p95/1000:.1f} s")
ax.axhline(95, color=COLOR_P95, linestyle=":", linewidth=0.8, alpha=0.6)

# --- Formato ejes ---
ax.set_xlabel("Total processing time per UAV (ms, log scale)", fontsize=11)
ax.set_ylabel("Cumulative UAVs (%)", fontsize=11)
ax.set_title("Total Processing Time CDF \u2014 Sequential Benchmark (2\u202f000 UAVs)", fontsize=12)

ax.set_xlim(latencies[0] * 0.9, latencies[-1] * 1.1)
ax.set_ylim(0, 101)
ax.yaxis.set_major_formatter(ticker.FormatStrFormatter('%g%%'))
ax.xaxis.set_major_formatter(
    ticker.FuncFormatter(lambda x, _: f"{x:,.0f}")
)

ax.grid(True, which="both", linestyle="--", linewidth=0.5, alpha=0.5)
ax.legend(fontsize=10, framealpha=0.9)

plt.tight_layout()
fig.savefig(OUTPUT_FILE, dpi=200, bbox_inches="tight")
print(f"\nFigura guardada en: {OUTPUT_FILE}")
plt.show()
