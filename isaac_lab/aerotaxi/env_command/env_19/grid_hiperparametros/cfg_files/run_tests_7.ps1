$ErrorActionPreference = 'Stop'

Write-Host "Ejecutando Test final para Experimento 4..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command15-Aerotaxi-RANDOM-4 --num_envs 256 --headless --seed 1001 --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v19/best_model_ppo_random_command15_4_8196.pt" --record_states --states_csv_dir "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v19/trayectorias_3d"

