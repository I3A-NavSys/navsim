"""
plot_scalability.py — Gráficas de escalabilidad comparando diferentes tamaños de flota.

Datos extraídos manualmente de los ficheros de resultados del benchmark
incremental real-time insertion.

Genera 3 gráficas:
  1. Avg / Median / P95 total processing time per UAV vs fleet size
  2. Avg detection time vs fleet size
  3. Memory usage vs fleet size
"""

from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np

# ---------------------------------------------------------------------------
# DATA — extraído de los ficheros SUMMARY de cada run
# ---------------------------------------------------------------------------
# Columns: fleet_size, avg_total_ms, median_total_ms, p95_total_ms,
#          avg_detect_ms, avg_insert_ms, avg_resolve_ms, memory_mb
data = [
    # n    avg_total  median    p95        avg_det   avg_ins   avg_res    mem_mb
    ( 500,  4348.23,  1660.44, 19042.68,   231.06,  1720.01,  13310.51,  1230.12),
    (1000,  7926.09,  2943.44, 30003.92,   323.34,  1970.48,  17167.04,  2400.21),
    (2000,  11720.77,  3988.42, 36363.87,   273.91,  1362.79,  20125.65,  4211.98),
]

n_uavs    = [d[0] for d in data]
avg_total = [d[1] for d in data]
med_total = [d[2] for d in data]
p95_total = [d[3] for d in data]
avg_det   = [d[4] for d in data]
avg_ins   = [d[5] for d in data]
avg_res   = [d[6] for d in data]
memory    = [d[7] for d in data]

OUT_DIR = Path(__file__).parent

# ---------------------------------------------------------------------------
# STYLE helpers
# ---------------------------------------------------------------------------
FONT_TITLE  = dict(fontsize=13, fontweight='bold')
FONT_AXIS   = dict(fontsize=11)
FONT_LEGEND = dict(fontsize=10)
BG = '#FAFAFA'
BLUE   = '#457B9D'
ORANGE = '#E9C46A'
RED    = '#E63946'
GREEN  = '#2A9D8F'
GREY   = '#6C757D'

def styled_ax(ax):
    ax.set_facecolor(BG)
    ax.grid(True, linestyle='--', linewidth=0.6, alpha=0.5, color='#CCCCCC')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

# ---------------------------------------------------------------------------
# FIGURE 1 — Total processing time per UAV (Avg / Median / P95)
# ---------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(8, 5), facecolor=BG)
styled_ax(ax)

x = np.array(n_uavs)

ax.plot(x, avg_total, 'o-', color=BLUE,   lw=2.2, ms=8, label='Average')
ax.plot(x, med_total, 's--', color=GREEN, lw=2.2, ms=8, label='Median')
ax.plot(x, p95_total, '^:', color=RED,    lw=2.2, ms=8, label='P95')

# Annotate points
for xi, ya, ym, yp in zip(x, avg_total, med_total, p95_total):
    ax.annotate(f'{ya/1000:.1f}s', (xi, ya), textcoords='offset points',
                xytext=(8, 4), fontsize=8, color=BLUE)
    ax.annotate(f'{ym/1000:.1f}s', (xi, ym), textcoords='offset points',
                xytext=(8, -12), fontsize=8, color=GREEN)
    ax.annotate(f'{yp/1000:.1f}s', (xi, yp), textcoords='offset points',
                xytext=(8, 4), fontsize=8, color=RED)

ax.set_xticks(x)
ax.set_xticklabels([f'{n:,}' for n in n_uavs])
ax.set_xlabel('Fleet size (UAVs)', **FONT_AXIS)
ax.set_ylabel('Total processing time per UAV (ms)', **FONT_AXIS)
ax.set_title('End-to-End Latency per UAV — Scalability', **FONT_TITLE)
ax.legend(**FONT_LEGEND)
ax.yaxis.set_major_formatter(ticker.FuncFormatter(lambda v, _: f'{v/1000:.0f}s'))

plt.tight_layout()
out1 = OUT_DIR / 'scalability_latency.pdf'
fig.savefig(out1, dpi=200, bbox_inches='tight')
print(f'Saved: {out1}')

# ---------------------------------------------------------------------------
# FIGURE 2 — Stacked Bar: breakdown per component (insert / detect / resolve)
# ---------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(8, 5), facecolor=BG)
styled_ax(ax)

bar_w = 200
bars_ins = ax.bar(x, avg_ins, bar_w, label='R-Tree Registration', color=BLUE,   alpha=0.9)
bars_det = ax.bar(x, avg_det, bar_w, bottom=avg_ins, label='Conflict Detection', color=ORANGE, alpha=0.9)
bars_res = ax.bar(x, avg_res, bar_w,
                  bottom=[i+d for i,d in zip(avg_ins, avg_det)],
                  label='Conflict Resolution', color=RED, alpha=0.9)

# Annotate total on top
for xi, ya in zip(x, avg_total):
    ax.text(xi, ya + 150, f'{ya/1000:.1f}s', ha='center', va='bottom',
            fontsize=9, fontweight='bold', color='#333333')

ax.set_xticks(x)
ax.set_xticklabels([f'{n:,}' for n in n_uavs])
ax.set_xlabel('Fleet size (UAVs)', **FONT_AXIS)
ax.set_ylabel('Avg. time per UAV (ms)', **FONT_AXIS)
ax.set_title('Processing Time Breakdown per Component — Scalability', **FONT_TITLE)
ax.legend(**FONT_LEGEND)
ax.yaxis.set_major_formatter(ticker.FuncFormatter(lambda v, _: f'{v/1000:.1f}s'))

plt.tight_layout()
out2 = OUT_DIR / 'scalability_breakdown.pdf'
fig.savefig(out2, dpi=200, bbox_inches='tight')
print(f'Saved: {out2}')

# ---------------------------------------------------------------------------
# FIGURE 3 — Memory usage vs fleet size
# ---------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(7, 4.5), facecolor=BG)
styled_ax(ax)

ax.plot(x, memory, 'D-', color=GREEN, lw=2.2, ms=9, label='Total memory delta')

# Memory per UAV secondary line
mem_per_uav = [m/n for m, n in zip(memory, n_uavs)]
ax2 = ax.twinx()
ax2.plot(x, mem_per_uav, 's--', color=ORANGE, lw=1.8, ms=7, label='Memory / UAV')
ax2.set_ylabel('Memory per UAV (MB/UAV)', **FONT_AXIS, color=ORANGE)
ax2.tick_params(axis='y', labelcolor=ORANGE)

for xi, m, mp in zip(x, memory, mem_per_uav):
    ax.annotate(f'{m:.0f} MB', (xi, m), textcoords='offset points',
                xytext=(8, 4), fontsize=8, color=GREEN)
    ax2.annotate(f'{mp:.2f} MB', (xi, mp), textcoords='offset points',
                 xytext=(8, -12), fontsize=8, color=ORANGE)

ax.set_xticks(x)
ax.set_xticklabels([f'{n:,}' for n in n_uavs])
ax.set_xlabel('Fleet size (UAVs)', **FONT_AXIS)
ax.set_ylabel('Total memory usage (MB)', **FONT_AXIS, color=GREEN)
ax.tick_params(axis='y', labelcolor=GREEN)
ax.set_title('Memory Consumption — Scalability', **FONT_TITLE)

lines1, labels1 = ax.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax.legend(lines1 + lines2, labels1 + labels2, **FONT_LEGEND)

plt.tight_layout()
out3 = OUT_DIR / 'scalability_memory.pdf'
fig.savefig(out3, dpi=200, bbox_inches='tight')
print(f'Saved: {out3}')

plt.show()
print('\nDone. All 3 figures saved.')
