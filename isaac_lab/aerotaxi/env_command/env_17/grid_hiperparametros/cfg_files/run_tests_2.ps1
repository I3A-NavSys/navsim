$ErrorActionPreference = 'Stop'

Write-Host "Ejecutando Test final para Experimento 6..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command13-Aerotaxi-RANDOM-6 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v13/best_model_ppo_random_command13_6_8196.pt"

Write-Host "Ejecutando Test final para Experimento 5..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command13-Aerotaxi-RANDOM-5 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v13/best_model_ppo_random_command13_5_8196.pt"

Write-Host "Ejecutando Test final para Experimento 3..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command13-Aerotaxi-RANDOM-3 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v13/best_model_ppo_random_command13_3_8196.pt"

Write-Host "Ejecutando Test final para Experimento 12..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command13-Aerotaxi-RANDOM-12 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v13/best_model_ppo_random_command13_12_8196.pt"

Write-Host "Ejecutando Test final para Experimento 11..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command13-Aerotaxi-RANDOM-11 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v13/best_model_ppo_random_command13_11_8196.pt"

