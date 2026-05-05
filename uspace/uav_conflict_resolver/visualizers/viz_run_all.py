"""
viz_run_all.py — Master Visualizer Runner
========================================================================

PURPOSE:
    Provides an interactive terminal menu to run any of the 6 visualizer
    scripts created for the TFG presentation.
"""

import sys
import os
import importlib

def clrscr():
    os.system('cls' if os.name == 'nt' else 'clear')

def main():
    scripts = {
        "1": ("Visualizer 1: OBB Swept Volumes & Trajectory Trace", "viz_01_obb_swept_volumes"),
        "2": ("Visualizer 2: SAT 15-Axis Collision Detection", "viz_02_sat_collision"),
        "3": ("Visualizer 3: R-Tree Broad-Phase 4D MBRs", "viz_03_rtree_broad_phase"),
        "4": ("Visualizer 4: Resolution Cascade Orchestration", "viz_04_resolution_cascade"),
        "5": ("Visualizer 5: Strategy 1 Kinematic Bounding", "viz_06_kinematic_bounding"),
        "6": ("Visualizer 6: Strategy 2 Trapezoid Geometry", "viz_07_trapezoid_detour"),
        "7": ("Visualizer 7: Full Multi-UAV Cascade Matrix", "viz_08_full_cascade_scenario"),
    }

    while True:
        clrscr()
        print("="*70)
        print("UAV CONFLICT RESOLVER - TFG VISUALIZATION SUITE")
        print("="*70)
        print("Select a visualizer to run:\n")
        
        for key, (name, _) in scripts.items():
            print(f"  [{key}] {name}")
            
        print("\n  [A] Run All Sequentially")
        print("  [Q] Quit")
        print("="*70)
        
        choice = input("\nEnter your choice: ").strip().upper()
        
        if choice == 'Q':
            print("Exiting...")
            break
        
        if choice == 'A':
            for key in sorted(scripts.keys()):
                run_script(scripts[key][1])
            input("\nAll complete. Press Enter to continue...")
            continue
            
        if choice in scripts:
            run_script(scripts[choice][1])
            input("\nPress Enter to return to menu...")
        else:
            input("Invalid choice. Press Enter to try again...")

def run_script(module_name):
    # Dynamically import and run the module's run_visualizer function
    try:
        if module_name in sys.modules:
            del sys.modules[module_name]  # Force reload
        mod = importlib.import_module(module_name)
        if hasattr(mod, 'run_visualizer'):
            # Close any lingering matplotlib windows
            import matplotlib.pyplot as plt
            plt.close('all')
            mod.run_visualizer()
        else:
            print(f"Error: {module_name} does not have a run_visualizer() function.")
    except Exception as e:
        print(f"\n[ERROR] Failed to run {module_name}: {e}")

if __name__ == "__main__":
    main()
