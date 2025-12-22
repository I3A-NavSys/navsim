$ErrorActionPreference = 'Stop'

Write-Host "Ejecutando ppo_random_hover_5"
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/train.py" --task Isaac-Hover-Aerotaxi-RANDOM-5 --num_envs 512 --headless 

Write-Host "Ejecutando ppo_random_hover_5"
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/train.py" --task Isaac-Hover-Aerotaxi-RANDOM-5 --num_envs 1024 --headless 

Write-Host "Ejecutando ppo_random_hover_5"
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/train.py" --task Isaac-Hover-Aerotaxi-RANDOM-5 --num_envs 2048 --headless 

Write-Host "Ejecutando ppo_random_hover_5"
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/train.py" --task Isaac-Hover-Aerotaxi-RANDOM-5 --num_envs 4096 --headless 

Write-Host "Ejecutando ppo_random_hover_5"
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/train.py" --task Isaac-Hover-Aerotaxi-RANDOM-5 --num_envs 8196 --headless 