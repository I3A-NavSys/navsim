$ErrorActionPreference = 'Stop'

Write-Host "Ejecutando Test final para Experimento 11..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command9-Aerotaxi-RANDOM-11 --num_envs 1  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v9/best_model_ppo_random_command9_11_4096.pt"

