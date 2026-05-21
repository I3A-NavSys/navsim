$ErrorActionPreference = 'Stop'

Write-Host "Ejecutando Test final para Experimento 9..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command8-Aerotaxi-RANDOM-9 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v8/best_model_ppo_random_command8_9_2048.pt"

Write-Host "Ejecutando Test final para Experimento 6..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command8-Aerotaxi-RANDOM-6 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v8/best_model_ppo_random_command8_6_1024.pt"

Write-Host "Ejecutando Test final para Experimento 1..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command8-Aerotaxi-RANDOM-1 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v8/best_model_ppo_random_command8_1_2048.pt"

Write-Host "Ejecutando Test final para Experimento 3..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command8-Aerotaxi-RANDOM-3 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v8/best_model_ppo_random_command8_3_2048.pt"

Write-Host "Ejecutando Test final para Experimento 26..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command8-Aerotaxi-RANDOM-26 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v8/best_model_ppo_random_command8_26_512.pt"

