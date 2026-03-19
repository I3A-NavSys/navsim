$ErrorActionPreference = 'Stop'

Write-Host "Ejecutando Test final para Experimento 25..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command12-Aerotaxi-RANDOM-25 --num_envs 1  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v12/best_model_ppo_random_command12_25_2048.pt"

Write-Host "Ejecutando Test final para Experimento 9..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command12-Aerotaxi-RANDOM-9 --num_envs 1  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v12/best_model_ppo_random_command12_9_512.pt"

Write-Host "Ejecutando Test final para Experimento 15..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command12-Aerotaxi-RANDOM-15 --num_envs 1  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v12/best_model_ppo_random_command12_15_512.pt"

Write-Host "Ejecutando Test final para Experimento 7..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command12-Aerotaxi-RANDOM-7 --num_envs 1  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v12/best_model_ppo_random_command12_7_512.pt"

Write-Host "Ejecutando Test final para Experimento 16..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command12-Aerotaxi-RANDOM-16 --num_envs 1  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v12/best_model_ppo_random_command12_16_8196.pt"

