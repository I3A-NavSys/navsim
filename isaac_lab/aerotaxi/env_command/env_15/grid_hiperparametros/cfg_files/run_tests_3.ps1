$ErrorActionPreference = 'Stop'

Write-Host "Ejecutando Test final para Experimento 11..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command11-Aerotaxi-RANDOM-11 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v11/best_model_ppo_random_command11_11_4096.pt"

