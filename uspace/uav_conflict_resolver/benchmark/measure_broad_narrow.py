"""
measure_broad_narrow.py — Perfila el Broad-Phase vs Narrow-Phase.

Inyecta contadores en el RTreeDetector para medir:
- Tasa de filtrado (Broad hits vs Narrow hits)
- Distribución de tiempo (R-Tree vs SAT)

Uso:
    python measure_broad_narrow.py
"""

import time
import matplotlib.pyplot as plt
from pathlib import Path
import sys

sys.path.append(str(Path(__file__).resolve().parent.parent))

from detection.rtree_detector import RTreeDetector
from central_manager import CentralManager
from benchmark.benchmark_large_scale import generate_large_fleet

# --- Monkey-Patching the Detector ---
_original_detect = RTreeDetector.detect_all_conflicts_system_wide

def profiled_detect_system_wide(self):
    conflicts = []
    uav_ids = list(self.uavs.keys())
    uav_index_map = {uav_id: idx for idx, uav_id in enumerate(uav_ids)}
    
    self.prof_broad_hits = 0
    self.prof_narrow_hits = 0
    self.prof_broad_time_ms = 0.0
    self.prof_narrow_time_ms = 0.0
    
    for current_uav_id, current_data in self.uavs.items():
        current_uav_idx = uav_index_map[current_uav_id]
        current_boxes = current_data["boxes"]
        
        for box_idx, my_box in enumerate(current_boxes):
            
            # --- BROAD PHASE ---
            t0 = time.perf_counter()
            potential_hits = self.tree_index.intersection(my_box.get_4d_bounds())
            self.prof_broad_time_ms += (time.perf_counter() - t0) * 1000.0
            
            potential_hits_list = list(potential_hits)
            
            for entry_id in potential_hits_list:
                resolved = self._resolve_entry(entry_id)
                if resolved is None: continue
                other_uav_id, other_box_idx = resolved
                
                if other_uav_id == current_uav_id: continue
                
                other_uav_idx = uav_index_map[other_uav_id]
                if other_uav_idx <= current_uav_idx: continue
                
                # It passed the broad phase filter
                self.prof_broad_hits += 1
                
                # --- NARROW PHASE ---
                other_box = self.uavs[other_uav_id]["boxes"][other_box_idx]
                
                t1 = time.perf_counter()
                is_collision, _ = my_box.collides_with(other_box)
                self.prof_narrow_time_ms += (time.perf_counter() - t1) * 1000.0
                
                if is_collision:
                    self.prof_narrow_hits += 1
                    conflicts.append("dummy")
                    
    return conflicts

# Apply patch
RTreeDetector.detect_all_conflicts_system_wide = profiled_detect_system_wide

# --- Run Benchmark ---
print("Generando flota de 300 UAVs para el profiling (modo denso)...")
fleet = generate_large_fleet(n_uavs=300, seed=42, verbose=False)

print("Inicializando R-Tree (Bulk Load)...")
manager = CentralManager()
manager.bulk_register_uavs([(fp.id, fp) for fp in fleet])
detector = manager._rtree_detector

print("Ejecutando detect_all_conflicts_system_wide()...")
t_start = time.perf_counter()
detector.detect_all_conflicts_system_wide()
total_ms = (time.perf_counter() - t_start) * 1000.0

total_indexed = sum(len(d["boxes"]) for d in detector.uavs.values())
total_possible_pairs = (total_indexed * (total_indexed - 1)) // 2

print("\n" + "="*60)
print("RESULTADOS DEL PROFILING (Broad vs Narrow)")
print("="*60)
print(f"Flota                       : {len(fleet)} UAVs")
print(f"Cajas (OBBs) indexadas      : {total_indexed:,}")
print(f"Posibles pares OBBxOBB      : {total_possible_pairs:,}")
print("-" * 60)
print(f"Broad Phase (R-Tree) extrajo: {detector.prof_broad_hits:,} pares sospechosos")
print(f"Narrow Phase (SAT) confirmo : {detector.prof_narrow_hits:,} colisiones reales")
print(f"Tasa de descarte R-Tree     : {100.0 - (detector.prof_broad_hits / total_possible_pairs * 100):.6f}%")
print(f"Falsos positivos (Broad)    : {100.0 - (detector.prof_narrow_hits / max(1, detector.prof_broad_hits) * 100):.2f}%")
print("-" * 60)
print(f"Tiempo en Broad Phase       : {detector.prof_broad_time_ms:.2f} ms")
print(f"Tiempo en Narrow Phase (SAT): {detector.prof_narrow_time_ms:.2f} ms")
print(f"Tiempo Overhead (Bucle PY)  : {total_ms - detector.prof_broad_time_ms - detector.prof_narrow_time_ms:.2f} ms")
print("="*60)

# --- Plot Pie Chart ---
OUTPUT_FILE = Path(__file__).parent / "broad_vs_narrow_pie.pdf"
BG = "#FAFAFA"
fig, ax = plt.subplots(figsize=(7, 5), facecolor=BG)

def format_large_number(n):
    if n >= 1e9: return f"{n/1e9:.1f}B"
    if n >= 1e6: return f"{n/1e6:.1f}M"
    return f"{n:,}"

lbl_broad = f"Broad Phase (R-Tree)\nTime: {detector.prof_broad_time_ms/1000:.1f} s\nDiscarded: {format_large_number(total_possible_pairs - detector.prof_broad_hits)} pairs"
lbl_narrow = f"Narrow Phase (SAT)\nTime: {detector.prof_narrow_time_ms:.0f} ms\nEvaluated: {format_large_number(detector.prof_broad_hits)} pairs\nCollisions: {detector.prof_narrow_hits}"

labels = [lbl_broad, lbl_narrow]
sizes = [detector.prof_broad_time_ms, detector.prof_narrow_time_ms]
colors = ['#457B9D', '#E63946']
explode = (0, 0.1)

ax.pie(sizes, explode=explode, labels=labels, colors=colors, autopct='%1.1f%%',
       live=False, startangle=90, textprops={'fontsize': 12, 'weight': 'bold'})
ax.axis('equal')  

plt.title("Time Distribution: Broad vs Narrow Phase (System-Wide Detection)", fontsize=14, y=1.05)
plt.tight_layout()
fig.savefig(OUTPUT_FILE, dpi=200, bbox_inches="tight")
print(f"\nGrafico guardado en: {OUTPUT_FILE}")
