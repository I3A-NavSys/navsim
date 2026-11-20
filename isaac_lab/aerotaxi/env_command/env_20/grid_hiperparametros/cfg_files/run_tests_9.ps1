$ErrorActionPreference = 'Stop'

Write-Host "Ejecutando Test final para Experimento 4..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command16-Aerotaxi-RANDOM-4 --num_envs 1 --headless --seed 1001 --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v20/best_model_ppo_random_command16_4_8196.pt"

