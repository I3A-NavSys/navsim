$ErrorActionPreference = 'Stop'

Write-Host "Ejecutando Test para Experimento 2..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command2-Aerotaxi-RANDOM-2 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v2/best_model_ppo_random_command2_2_8196.pt"

Write-Host "Ejecutando Test para Experimento 21..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command2-Aerotaxi-RANDOM-21 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v2/best_model_ppo_random_command2_21_4096.pt"

Write-Host "Ejecutando Test para Experimento 20..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command2-Aerotaxi-RANDOM-20 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v2/best_model_ppo_random_command2_20_1024.pt"

Write-Host "Ejecutando Test para Experimento 3..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command2-Aerotaxi-RANDOM-3 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v2/best_model_ppo_random_command2_3_2048.pt"

Write-Host "Ejecutando Test para Experimento 31..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command2-Aerotaxi-RANDOM-31 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v2/best_model_ppo_random_command2_31_512.pt"

