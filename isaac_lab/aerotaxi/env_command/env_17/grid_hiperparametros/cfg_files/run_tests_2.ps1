$ErrorActionPreference = 'Stop'

Write-Host "Ejecutando Test final para Experimento 5..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command13-Aerotaxi-RANDOM-5 --num_envs 1024  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v13/best_model_ppo_random_command13_5_8196.pt"

Write-Host "Ejecutando Test final para Experimento 21..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command13-Aerotaxi-RANDOM-21 --num_envs 1024  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v13/best_model_ppo_random_command13_21_8196.pt"

Write-Host "Ejecutando Test final para Experimento 6..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command13-Aerotaxi-RANDOM-6 --num_envs 1024  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v13/best_model_ppo_random_command13_6_8196.pt"

Write-Host "Ejecutando Test final para Experimento 32..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command13-Aerotaxi-RANDOM-32 --num_envs 1024  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v13/best_model_ppo_random_command13_32_8196.pt"

Write-Host "Ejecutando Test final para Experimento 25..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command13-Aerotaxi-RANDOM-25 --num_envs 1024  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v13/best_model_ppo_random_command13_25_8196.pt"

