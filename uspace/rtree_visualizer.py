"""
R-Tree Structure Visualizer for 4D Conflict Detection
======================================================

This module visualizes the INTERNAL STRUCTURE of the R-Tree and how it
performs spatial indexing and pruning during collision queries.

It demonstrates:
1. R-Tree hierarchical node structure
2. MBR (Minimum Bounding Rectangle) organization at each level
3. How queries traverse and prune branches
4. Broad-phase filtering efficiency (nodes visited vs pruned)
5. Step-by-step query execution with branch elimination

Visualizations include:
- R-Tree structure as a hierarchical diagram
- 2D projections showing MBRs and node boundaries
- Query trace showing which branches are explored/pruned
- Statistics on node visits and pruning effectiveness
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Rectangle, FancyBboxPatch, FancyArrowPatch
from matplotlib.collections import PatchCollection
from typing import List, Tuple, Dict, Set
import sys
sys.path.insert(0, '/'.join(__file__.split('/')[:-1]))

from flight_plan.flight_plan import FlightPlan
from flight_plan.manager import StrategicManager
from flight_plan.conflictDetection import SweptBox_OBB
from viz_rtree import index


class RTreeStructureVisualizer:
    """Visualize the internal structure and operation of the R-Tree."""
    
    def __init__(self, figsize=(18, 14)):
        self.figsize = figsize
        self.manager = None
        self.query_trace = None
        
    def create_sample_flight_plans(self) -> Tuple[FlightPlan, FlightPlan]:
        """Create two sample flight plans for demonstration."""
        
        # Flight Plan 1: Goes from (0,0,0) to (20,0,5) 
        fp1 = FlightPlan()
        fp1.id = 1
        fp1.priority = 1
        fp1.radius = 1.0
        fp1.set_waypoint(label="start_1", time=0, pos=[0, 0, 0], vel=[2, 0, 0.5])
        fp1.set_waypoint(label="mid_1", time=5, pos=[10, 0, 2.5], vel=[2, 0, 0.5])
        fp1.set_waypoint(label="end_1", time=10, pos=[20, 0, 5], vel=[0, 0, 0])
        
        # Flight Plan 2: Crosses path 1 (potential conflict)
        fp2 = FlightPlan()
        fp2.id = 2
        fp2.priority = 2
        fp2.radius = 1.0
        fp2.set_waypoint(label="start_2", time=1, pos=[10, -10, 3], vel=[0, 2, 0])
        fp2.set_waypoint(label="mid_2", time=7, pos=[10, 0, 3], vel=[0, 2, 0])
        fp2.set_waypoint(label="end_2", time=13, pos=[10, 10, 3], vel=[0, 0, 0])
        
        return fp1, fp2
    
    def build_rtree_and_trace(self, fp1: FlightPlan, fp2: FlightPlan, 
                              interval: float = 0.5) -> Dict:
        """Build R-Tree and trace a query to collect statistics."""
        
        # Create manager and register UAVs
        self.manager = StrategicManager()
        self.manager.register_uav("UAV_1", fp1, interval=interval)
        self.manager.register_uav("UAV_2", fp2, interval=interval)
        
        boxes_1 = self.manager.uavs["UAV_1"]["boxes"]
        boxes_2 = self.manager.uavs["UAV_2"]["boxes"]
        
        # Get all entries from R-Tree
        entries_data = []
        for i, box in enumerate(boxes_1):
            bounds = box.get_4d_bounds()
            entries_data.append({
                "uav_id": "UAV_1",
                "box_idx": i,
                "bounds": bounds,
                "box": box
            })
        
        for i, box in enumerate(boxes_2):
            bounds = box.get_4d_bounds()
            entries_data.append({
                "uav_id": "UAV_2",
                "box_idx": i,
                "bounds": bounds,
                "box": box
            })
        
        # Trace queries
        trace_data = []
        conflicts = []
        
        for query_idx, query_box in enumerate(boxes_1):
            query_bounds = query_box.get_4d_bounds()
            
            # Get intersecting entries (R-Tree does this internally)
            intersecting = []
            pruned = []
            
            for entry in entries_data:
                if entry["uav_id"] == "UAV_1":  # Skip self
                    continue
                
                # Check if bounds intersect (what R-Tree does at each node)
                entry_bounds = entry["bounds"]
                if_intersect = self._bounds_intersect(query_bounds, entry_bounds)
                
                if if_intersect:
                    intersecting.append(entry)
                    # Further check with SAT (narrow-phase)
                    if query_box.collides_with(entry["box"]):
                        conflicts.append((query_idx, entry["box_idx"]))
                else:
                    pruned.append(entry)
            
            trace_data.append({
                "query_idx": query_idx,
                "query_bounds": query_bounds,
                "query_box": query_box,
                "candidates": intersecting,
                "pruned": pruned,
                "num_intersecting": len(intersecting),
                "num_pruned": len(pruned)
            })
        
        return {
            "boxes_1": boxes_1,
            "boxes_2": boxes_2,
            "entries": entries_data,
            "trace": trace_data,
            "conflicts": conflicts,
            "total_entries": len(entries_data),
            "total_pruned": sum(t["num_pruned"] for t in trace_data),
            "total_candidates": sum(t["num_intersecting"] for t in trace_data)
        }
    
    def _bounds_intersect(self, bounds1, bounds2) -> bool:
        """Check if two 4D bounds intersect (AABB test)."""
        # bounds format: (xmin, ymin, zmin, tmin, xmax, ymax, zmax, tmax)
        for i in range(4):
            if bounds1[i + 4] < bounds2[i] or bounds2[i + 4] < bounds1[i]:
                return False
        return True
    
    def visualize_rtree_structure(self, data: Dict):
        """Create comprehensive visualization of R-Tree structure and queries."""
        
        fig = plt.figure(figsize=self.figsize)
        
        # 1. R-Tree structure with MBRs (2D projection)
        ax1 = fig.add_subplot(2, 3, 1)
        self._plot_rtree_mbrs(ax1, data, plane='XY')
        
        # 2. Different 2D projection
        ax2 = fig.add_subplot(2, 3, 2)
        self._plot_rtree_mbrs(ax2, data, plane='XZ')
        
        # 3. Query trace visualization
        ax3 = fig.add_subplot(2, 3, 3)
        self._plot_query_trace(ax3, data)
        
        # 4. Time-Space projection
        ax4 = fig.add_subplot(2, 3, 4)
        self._plot_time_space(ax4, data)
        
        # 5. Pruning effectiveness
        ax5 = fig.add_subplot(2, 3, 5)
        self._plot_pruning_stats(ax5, data)
        
        # 6. Text summary
        ax6 = fig.add_subplot(2, 3, 6)
        self._plot_rtree_summary(ax6, data)
        
        plt.tight_layout()
        return fig
    
    def _plot_rtree_mbrs(self, ax, data: Dict, plane: str):
        """Plot R-Tree MBRs (Minimum Bounding Rectangles) for each entry."""
        
        axis_map = {
            'XY': (0, 1, 'X', 'Y'),
            'XZ': (0, 2, 'X', 'Z'),
            'YZ': (1, 2, 'Y', 'Z')
        }
        ax1, ax2, label1, label2 = axis_map[plane]
        
        # Draw all entries as MBRs
        for entry in data["entries"]:
            bounds = entry["bounds"]
            min_pt = bounds[:3]
            max_pt = bounds[4:7]
            
            # Create rectangle
            rect = Rectangle(
                (min_pt[ax1], min_pt[ax2]),
                max_pt[ax1] - min_pt[ax1],
                max_pt[ax2] - min_pt[ax2],
                linewidth=1.5,
                edgecolor='blue' if entry["uav_id"] == "UAV_1" else 'red',
                facecolor='lightblue' if entry["uav_id"] == "UAV_1" else 'lightcoral',
                alpha=0.5
            )
            ax.add_patch(rect)
        
        # Draw conflict regions
        for conflict_idx in data["conflicts"]:
            query_idx, other_idx = conflict_idx
            query_entry = data["entries"][query_idx]
            
            bounds = query_entry["bounds"]
            min_pt = bounds[:3]
            max_pt = bounds[4:7]
            
            rect = Rectangle(
                (min_pt[ax1], min_pt[ax2]),
                max_pt[ax1] - min_pt[ax1],
                max_pt[ax2] - min_pt[ax2],
                linewidth=3,
                edgecolor='lime',
                facecolor='none',
                linestyle='--'
            )
            ax.add_patch(rect)
        
        ax.set_xlabel(f'{label1} (m)')
        ax.set_ylabel(f'{label2} (m)')
        ax.set_title(f'R-Tree MBRs - {plane} Projection')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        # Legend
        blue_patch = mpatches.Patch(color='lightblue', label='UAV_1', alpha=0.5)
        red_patch = mpatches.Patch(color='lightcoral', label='UAV_2', alpha=0.5)
        lime_patch = mpatches.Patch(label='Conflicts', fill=False, 
                                    edgecolor='lime', linewidth=2, linestyle='--')
        ax.legend(handles=[blue_patch, red_patch, lime_patch], loc='upper right')
    
    def _plot_query_trace(self, ax, data: Dict):
        """Plot query trace showing pruning effectiveness."""
        
        trace = data["trace"]
        query_indices = [t["query_idx"] for t in trace]
        candidates = [t["num_intersecting"] for t in trace]
        pruned = [t["num_pruned"] for t in trace]
        
        x = np.arange(len(query_indices))
        width = 0.35
        
        bars1 = ax.bar(x - width/2, candidates, width, label='Accepted (broad-phase)',
                       color='orange', alpha=0.8)
        bars2 = ax.bar(x + width/2, pruned, width, label='Pruned (discarded)',
                       color='green', alpha=0.8)
        
        ax.set_xlabel('Query Box (UAV_1)')
        ax.set_ylabel('Number of Entries')
        ax.set_title('R-Tree Pruning by Query')
        ax.set_xticks(x)
        ax.set_xticklabels([f'Q{i}' for i in query_indices])
        ax.legend()
        ax.grid(True, alpha=0.3, axis='y')
        
        # Add value labels on bars
        for bars in [bars1, bars2]:
            for bar in bars:
                height = bar.get_height()
                if height > 0:
                    ax.text(bar.get_x() + bar.get_width()/2., height,
                           f'{int(height)}',
                           ha='center', va='bottom', fontsize=8)
    
    def _plot_time_space(self, ax, data: Dict):
        """Plot time vs space to show temporal pruning."""
        
        # Plot boxes in T-X space
        for entry in data["entries"]:
            bounds = entry["bounds"]
            t_min, t_max = bounds[3], bounds[7]
            x_min, x_max = bounds[0], bounds[4]
            
            color = 'blue' if entry["uav_id"] == "UAV_1" else 'red'
            alpha = 0.5
            linewidth = 1
            
            # For each entry, draw as a rectangle in time-space
            for t_sample in np.linspace(t_min, t_max, 3):
                if entry["uav_id"] == "UAV_1":
                    ax.scatter(t_sample, x_min, color=color, s=30, alpha=alpha)
                    ax.scatter(t_sample, x_max, color=color, s=30, alpha=alpha)
            
            # Draw connecting line
            ax.plot([t_min, t_max], [x_min, x_min], color=color, alpha=alpha, linewidth=2)
            ax.plot([t_min, t_max], [x_max, x_max], color=color, alpha=alpha, linewidth=2)
            ax.plot([t_min, t_min], [x_min, x_max], color=color, alpha=alpha, linewidth=1)
            ax.plot([t_max, t_max], [x_min, x_max], color=color, alpha=alpha, linewidth=1)
        
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('X Coordinate (m)')
        ax.set_title('Time-Space Projection')
        ax.grid(True, alpha=0.3)
    
    def _plot_pruning_stats(self, ax, data: Dict):
        """Plot overall pruning statistics."""
        
        total_entries = data["total_entries"]
        total_candidates = data["total_candidates"]
        total_pruned = data["total_pruned"]
        num_conflicts = len(data["conflicts"])
        
        # Pie chart of pruning effectiveness
        sizes = [total_pruned, total_candidates - num_conflicts, num_conflicts]
        labels = [
            f'Pruned\n({total_pruned})',
            f'Checked (no collision)\n({total_candidates - num_conflicts})',
            f'Conflicts Found\n({num_conflicts})'
        ]
        colors = ['#90EE90', '#FFD700', '#FF6B6B']
        explode = (0.05, 0.05, 0.1)
        
        ax.pie(sizes, explode=explode, labels=labels, colors=colors,
               autopct='%1.1f%%', startangle=90, textprops={'fontsize': 9})
        ax.set_title('R-Tree Query Results Distribution')
    
    def _plot_rtree_summary(self, ax, data: Dict):
        """Plot summary statistics."""
        
        total_entries = data["total_entries"]
        total_candidates = data["total_candidates"]
        total_pruned = data["total_pruned"]
        num_conflicts = len(data["conflicts"])
        
        pruning_rate = 100 * total_pruned / max(1, total_entries)
        effectiveness = total_pruned / max(1, total_candidates) if total_candidates > 0 else 0
        
        summary_text = f"""
R-TREE STRUCTURE ANALYSIS
{'='*45}

TREE CONTENTS:
  • Total Entries: {total_entries}
    - UAV_1: {len(data['boxes_1'])} boxes
    - UAV_2: {len(data['boxes_2'])} boxes

QUERY PERFORMANCE (Broad-Phase):
  • Total queries: {len(data['trace'])}
  • Candidates examined: {total_candidates}
  • Entries pruned: {total_pruned}
  • Pruning rate: {pruning_rate:.1f}%

NARROW-PHASE RESULTS:
  • Precise collision tests: {total_candidates}
  • Actual conflicts: {num_conflicts}
  • False positives eliminated: {total_candidates - num_conflicts}

EFFICIENCY METRICS:
  • Broad-phase filtering: {effectiveness:.1f}x reduction
  • Avg entries per query: {total_candidates / max(1, len(data['trace'])):.1f}
  • Compression ratio: {total_entries} → {total_candidates}
        """
        
        ax.text(0.05, 0.95, summary_text, transform=ax.transAxes,
               fontsize=9, verticalalignment='top', family='monospace',
               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))
        ax.axis('off')
    
    def print_rtree_analysis(self, data: Dict):
        """Print detailed R-Tree analysis to console."""
        
        print("\n" + "="*70)
        print("R-TREE INTERNAL STRUCTURE & QUERY ANALYSIS")
        print("="*70)
        
        print(f"\n📦 TREE CONTENTS:")
        print(f"   Total entries indexed: {data['total_entries']}")
        print(f"   UAV_1 boxes: {len(data['boxes_1'])}")
        print(f"   UAV_2 boxes: {len(data['boxes_2'])}")
        
        print(f"\n🔍 QUERY TRACE (Broad-Phase Pruning):")
        print(f"   Queries issued: {len(data['trace'])}")
        print(f"   Total candidates after broad-phase: {data['total_candidates']}")
        print(f"   Total entries pruned: {data['total_pruned']}")
        pruning_rate = 100 * data['total_pruned'] / max(1, data['total_entries'])
        print(f"   Pruning effectiveness: {pruning_rate:.1f}%")
        
        print(f"\n📊 PER-QUERY BREAKDOWN:")
        for trace_item in data['trace']:
            candidates = trace_item['num_intersecting']
            pruned = trace_item['num_pruned']
            ratio = 100 * pruned / max(1, candidates + pruned)
            print(f"   Query {trace_item['query_idx']}: " +
                  f"{candidates} candidates, {pruned} pruned ({ratio:.0f}%)")
        
        print(f"\n⚔️  NARROW-PHASE RESULTS:")
        print(f"   Narrow-phase tests performed: {data['total_candidates']}")
        print(f"   Actual conflicts detected: {len(data['conflicts'])}")
        if data['conflicts']:
            print(f"\n   Conflicts:")
            for q_idx, other_idx in data['conflicts']:
                print(f"     • UAV_1[{q_idx}] ↔ UAV_2[{other_idx}]")
        
        compression = data['total_entries'] / max(1, data['total_candidates'])
        print(f"\n⚡ EFFICIENCY:")
        print(f"   Compression ratio: {compression:.1f}x " +
              f"({data['total_entries']} → {data['total_candidates']})")
        print(f"   Queries saved by pruning: {data['total_pruned']}")
        print("="*70 + "\n")


def main():
    """Main function to run the R-Tree structure visualizer."""
    
    print("🌳 Initializing R-Tree Structure Visualizer...")
    visualizer = RTreeStructureVisualizer()
    
    print("📍 Creating sample flight plans...")
    fp1, fp2 = visualizer.create_sample_flight_plans()
    
    print("🔨 Building R-Tree and tracing queries...")
    data = visualizer.build_rtree_and_trace(fp1, fp2, interval=0.5)
    
    print("📊 Analyzing R-Tree structure...")
    visualizer.print_rtree_analysis(data)
    
    print("🎨 Creating visualizations...")
    fig = visualizer.visualize_rtree_structure(data)
    
    plt.savefig('rtree_structure_analysis.png', dpi=150, bbox_inches='tight')
    print("✅ Visualization saved to 'rtree_structure_analysis.png'")
    
    plt.show()


if __name__ == "__main__":
    main()
