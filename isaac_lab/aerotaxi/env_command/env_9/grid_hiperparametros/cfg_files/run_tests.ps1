$ErrorActionPreference = 'Stop'

Write-Host "Ejecutando Test final para Experimento 11..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command5-Aerotaxi-RANDOM-11 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v5/best_model_final_ppo_random_command5_11_8196.pt"

Write-Host "Ejecutando Test curriculum para Experimento 11..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command5-Aerotaxi-RANDOM-11 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v5/best_model_curriculum_ppo_random_command5_11_8196.pt"

