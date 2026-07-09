$ErrorActionPreference = 'Stop'

Write-Host "Ejecutando Test final para Experimento 32..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command13-Aerotaxi-RANDOM-32 --num_envs 256 --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v13/best_model_ppo_random_command13_32_8196.pt" --record_states --states_csv_dir "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v13/trayectorias_3d"

