$ErrorActionPreference = 'Stop'

Write-Host "Ejecutando Test para Experimento 3..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command3-Aerotaxi-RANDOM-3 --num_envs 1  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v3/best_model_ppo_random_command3_3_2048.pt"

Write-Host "Ejecutando Test para Experimento 6..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command3-Aerotaxi-RANDOM-6 --num_envs 1  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v3/best_model_ppo_random_command3_6_8196.pt"

Write-Host "Ejecutando Test para Experimento 7..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command3-Aerotaxi-RANDOM-7 --num_envs 1  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v3/best_model_ppo_random_command3_7_512.pt"

Write-Host "Ejecutando Test para Experimento 1..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command3-Aerotaxi-RANDOM-1 --num_envs 1  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v3/best_model_ppo_random_command3_1_64.pt"

Write-Host "Ejecutando Test para Experimento 13..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command3-Aerotaxi-RANDOM-13 --num_envs 1  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v3/best_model_ppo_random_command3_13_128.pt"

