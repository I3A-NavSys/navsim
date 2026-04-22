$ErrorActionPreference = 'Stop'

Write-Host "Ejecutando Test final para Experimento 5..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command14-Aerotaxi-RANDOM-5 --num_envs 1024  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v14/best_model_ppo_random_command14_5_8196.pt"

Write-Host "Ejecutando Test final para Experimento 31..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command14-Aerotaxi-RANDOM-31 --num_envs 1024  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v14/best_model_ppo_random_command14_31_8196.pt"

Write-Host "Ejecutando Test final para Experimento 6..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command14-Aerotaxi-RANDOM-6 --num_envs 1024  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v14/best_model_ppo_random_command14_6_8196.pt"

Write-Host "Ejecutando Test final para Experimento 23..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command14-Aerotaxi-RANDOM-23 --num_envs 1024  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v14/best_model_ppo_random_command14_23_8196.pt"

Write-Host "Ejecutando Test final para Experimento 22..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command14-Aerotaxi-RANDOM-22 --num_envs 1024  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v14/best_model_ppo_random_command14_22_8196.pt"

