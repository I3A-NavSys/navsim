"""
viz_07_detour_academic.py — Visualizador académico: Detour (diseño y arquitectura)
========================================================================

Propósito:
    Proporciona una visualización conceptual y profesional del mecanismo
    de detour (rigid shift / MTV → detour) pensada para incluir en el
    capítulo 4 (diseño y arquitectura) del TFG. Incluye:
      - Vista 3D simplificada de rutas y detour
      - Diagrama de arquitectura (componentes y flujo de datos)
      - Anotaciones académicas y leyenda preparada para exportar a PDF/PNG

Uso:
    Ejecutar como script para generar una imagen de ejemplo:
        python visualizers/viz_07_detour_academic.py

Salida:
    Guarda `detour_academic_view.png` y `detour_academic_view.pdf` en el
    directorio del proyecto.

Nota: Este script está pensado como herramienta ilustrativa. Puede
      integrarse con los objetos reales (`FlightPlan`, `OBB`, etc.) si
      se le pasan datos concretos.
"""

from pathlib import Path
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, ConnectionPatch
from mpl_toolkits.mplot3d import Axes3D

BG = "#ffffff"
ACCENT = "#2c3e50"

# Ensure project root is importable if user integrates with real objects
_ROOT = str(Path(__file__).resolve().parent.parent)
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)


def _make_synthetic_scenario():
    """Crea un par de trayectorias simples que se cruzan y una detour."""
    t = np.linspace(0, 60, 601)

    # Plebeian: line along X from left to right, slight altitude
    x1 = np.linspace(-300, 300, t.size)
    y1 = np.zeros_like(x1) + 0.0
    z1 = np.ones_like(x1) * 100.0

    # VIP: line along Y crossing at origin
    x2 = np.zeros_like(t) + 0.0
    y2 = np.linspace(-300, 300, t.size)
    z2 = np.ones_like(y2) * 110.0

    # Detour: simple 3-point topology (anc -> det -> ret)
    anc = np.array([-60.0, 0.0, 100.0])
    det = np.array([0.0, 80.0, 100.0])
    ret = np.array([60.0, 0.0, 100.0])

    # Build short traces for new path (concatenate linear segments)
    seg1 = np.linspace(anc, det, 80)
    seg2 = np.linspace(det, ret, 80)
    trace_new = np.vstack([seg1, seg2])

    trace_orig = np.vstack([x1, y1, z1]).T
    trace_vip = np.vstack([x2, y2, z2]).T

    return t, trace_orig, trace_vip, trace_new, anc, det, ret


def visualize_detour_academic(trace_orig, trace_vip, trace_new, anc, det, ret, save_path=None, dpi=300):
    """Dibuja la visualización académica con arquitectura y anotaciones.

    Inputs are simple numpy arrays (N,3). If you have `FlightPlan` objects,
    convert them to traces before calling this function.
    """
    fig = plt.figure(figsize=(12, 7), constrained_layout=True)

    # Left: 3D scene
    ax3d = fig.add_subplot(1, 2, 1, projection='3d')
    ax3d.set_facecolor(BG)
    ax3d.plot(trace_orig[:, 0], trace_orig[:, 1], trace_orig[:, 2],
              color="#7f8c8d", linestyle='--', linewidth=2, label='Ruta original (plebeian)')
    ax3d.plot(trace_vip[:, 0], trace_vip[:, 1], trace_vip[:, 2],
              color="#c0392b", linestyle='-', linewidth=2, label='Ruta VIP (obstáculo)')
    ax3d.plot(trace_new[:, 0], trace_new[:, 1], trace_new[:, 2],
              color="#27ae60", linewidth=3, label='Trayectoria evasiva (detour)')

    # Mark the 3 magic waypoints
    ax3d.scatter(*anc, color="#e67e22", s=70, marker='s')
    ax3d.text(anc[0], anc[1], anc[2]+8, 'anc', color="#e67e22", fontsize=9, fontweight='bold')
    ax3d.scatter(*det, color="#2980b9", s=70, marker='^')
    ax3d.text(det[0], det[1], det[2]+8, 'det', color="#2980b9", fontsize=9, fontweight='bold')
    ax3d.scatter(*ret, color="#1abc9c", s=70, marker='D')
    ax3d.text(ret[0], ret[1], ret[2]+8, 'ret', color="#1abc9c", fontsize=9, fontweight='bold')

    ax3d.set_xlabel('X (m)', fontsize=9)
    ax3d.set_ylabel('Y (m)', fontsize=9)
    ax3d.set_zlabel('Z (m)', fontsize=9)
    ax3d.view_init(elev=25, azim=30)
    ax3d.set_box_aspect([1, 1, 0.4])
    ax3d.legend()

    # Right: Architecture diagram + explanations
    ax_arch = fig.add_subplot(1, 2, 2)
    ax_arch.axis('off')

    # Define component boxes (x, y, width, height) in axes coords
    boxes = [
        (0.12, 0.72, 0.32, 0.16, 'Sensing / OBB Generation'),
        (0.12, 0.48, 0.32, 0.16, 'Collision Detection (R-Tree + SAT)'),
        (0.12, 0.24, 0.32, 0.16, 'MTV Selector & Filter'),
        (0.12, 0.00, 0.32, 0.16, 'Kinematic Feasibility / Planner'),
    ]

    # Draw boxes with connecting arrows
    for (x, y, w, h, label) in boxes:
        rect = FancyBboxPatch((x, y), w, h, boxstyle='round,pad=0.02',
                              ec=ACCENT, fc='#f7f9fb', linewidth=1.3)
        ax_arch.add_patch(rect)
        ax_arch.text(x + 0.02, y + h/2, label, fontsize=9, color=ACCENT, va='center')

    # Arrows between boxes
    arrow_props = dict(arrowstyle='-|>', linewidth=1.2, color=ACCENT)
    ax_arch.annotate('', xy=(0.28, 0.72), xytext=(0.28, 0.64), arrowprops=arrow_props)
    ax_arch.annotate('', xy=(0.28, 0.48), xytext=(0.28, 0.36), arrowprops=arrow_props)
    ax_arch.annotate('', xy=(0.28, 0.24), xytext=(0.28, 0.12), arrowprops=arrow_props)

    # Add a small flow legend
    ax_arch.text(0.5, 0.82, 'Flujo de Datos y Decisión', fontsize=11, weight='bold', color=ACCENT)
    ax_arch.text(0.5, 0.75, '1) Sensado → 2) Detección → 3) MTV → 4) Planificación', fontsize=9)

    # Add compact explanation boxes (academic tone)
    expl = (
        'Conceptos clave:\n'
        '• MTV (Minimum Translation Vector): vector mínimo para separar OBBs.\n'
        '• Filtrado cinemático: descartar soluciones no factibles.\n'
        '• 3-point topology: anc → det → ret, mínima desviación temporal.'
    )
    ax_arch.text(0.5, 0.45, expl, fontsize=9, va='center')

    # Draw a small schematic showing where the detour occurs relative to conflict
    ax_arch.text(0.5, 0.22, 'Interpretación académica:', fontsize=10, weight='bold', color=ACCENT)
    ax_arch.text(
        0.5,
        0.16,
        'El sistema propone un desplazamiento lateral (MTV) y verifica\n'
        'la factibilidad cinemática antes de emitir la nueva ruta.',
        fontsize=9,
    )

    # Footer: caption and authorship for academic figure
    caption = 'Figura: Esquema conceptual del detour y diagrama de arquitectura del módulo de resolución.'
    fig.text(0.12, 0.02, caption, fontsize=8, color='#4b4b4b')

    if save_path:
        png = Path(save_path).with_suffix('.png')
        pdf = Path(save_path).with_suffix('.pdf')
    else:
        png = Path('detour_academic_view.png')
        pdf = Path('detour_academic_view.pdf')

    plt.savefig(png, dpi=dpi, bbox_inches='tight')
    plt.savefig(pdf, dpi=dpi, bbox_inches='tight')
    plt.show()

    return str(png), str(pdf)


def main():
    t, trace_orig, trace_vip, trace_new, anc, det, ret = _make_synthetic_scenario()
    out_png, out_pdf = visualize_detour_academic(trace_orig, trace_vip, trace_new, anc, det, ret)
    print(f"Saved: {out_png}")
    print(f"Saved: {out_pdf}")


if __name__ == '__main__':
    main()
