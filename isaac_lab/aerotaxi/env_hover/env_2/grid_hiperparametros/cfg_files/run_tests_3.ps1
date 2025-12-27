$ErrorActionPreference = 'Stop'

Write-Host "Ejecutando Test para Experimento 38..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Hover-Aerotaxi-RANDOM-38 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_v2/best_model_ppo_random_hover_38_2048.pt"

Write-Host "Ejecutando Test para Experimento 63..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Hover-Aerotaxi-RANDOM-63 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_v2/best_model_ppo_random_hover_63_1024.pt"

Write-Host "Ejecutando Test para Experimento 93..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Hover-Aerotaxi-RANDOM-93 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_v2/best_model_ppo_random_hover_93_4096.pt"

Write-Host "Ejecutando Test para Experimento 13..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Hover-Aerotaxi-RANDOM-13 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_v2/best_model_ppo_random_hover_13_2048.pt"

Write-Host "Ejecutando Test para Experimento 62..."
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Hover-Aerotaxi-RANDOM-62 --num_envs 256  --headless --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_v2/best_model_ppo_random_hover_62_64.pt"

