from dotenv import load_dotenv
import subprocess
import time
import sys
import os

def launch_simulation_cluster():
    """
    Orchestrates the execution of all Digital Twin Block (DTB) scripts.
    Uses subprocess.Popen to run them asynchronously in the background since 
    each script contains an infinite loop or a blocking wait event.
    """
    # Define the exact execution order required by your architecture
    scripts_to_run = [
        "/home/tetemo/Escritorio/UCLM/TRABAJO/navsim/uspace/platgdia/dtblocks/validation/validator.py",
        "/home/tetemo/Escritorio/UCLM/TRABAJO/navsim/uspace/platgdia/dtblocks/uspace_manager/uspace_manager_dtb.py",
        "/home/tetemo/Escritorio/UCLM/TRABAJO/navsim/uspace/platgdia/dtblocks/vertiport_operator/vertiport_operator_dtb.py",
        "/home/tetemo/Escritorio/UCLM/TRABAJO/navsim/uspace/platgdia/dtblocks/uav_operator/uav_operator_dtb.py",
        "/home/tetemo/Escritorio/UCLM/TRABAJO/navsim/uspace/platgdia/dtblocks/mission_manager/mission_manager_dtb.py",
    ]
    
    active_processes = []
    
    try:
        for script in scripts_to_run:
            print(f"[*] Launching {script}...")
            
            # Ensure the script exists to avoid crashing halfway through
            if not os.path.exists(script):
                print(f"[!] Error: Could not find {script}. Check your current directory.")
                continue

            # sys.executable ensures it uses the exact same Python interpreter 
            # (and virtual environment) that is running this master script.
            process = subprocess.Popen([sys.executable, script])
            active_processes.append((script, process))
            
            # A short delay guarantees the OS allocates resources sequentially
            time.sleep(1)
            
        print("\n[+] All U-Space simulation nodes are online.")
        print("[i] Press Ctrl+C at any time to safely terminate all processes.\n")
        
        # This keeps the master script alive, waiting for the child processes.
        # Since they run infinitely, this acts as a permanent anchor.
        for _, process in active_processes:
            process.wait()
            
    except KeyboardInterrupt:
        # Graceful shutdown handler triggered by Ctrl+C
        print("\n[!] Shutdown signal received. Terminating simulation cluster...")
        for script, process in active_processes:
            print(f"[-] Terminating {script}...")
            process.terminate()  # Sends SIGTERM to the process
            process.wait()       # Waits for the OS to confirm the kill
            
        print("[+] All processes terminated successfully. Goodbye.")

if __name__ == "__main__":
    # Load environment variables
    env_path = "/home/tetemo/Escritorio/UCLM/TRABAJO/navsim/uspace/platgdia/dtblocks/.env"
    load_dotenv(env_path)
    
    launch_simulation_cluster()