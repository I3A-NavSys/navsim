$ErrorActionPreference = 'Stop'

Write-Host "Evaluando seed_1001 (Experimento 4)..."
Remove-Item Env:\AEROTAXI_TAU_MS -ErrorAction SilentlyContinue
Remove-Item Env:\AEROTAXI_WIND_MS -ErrorAction SilentlyContinue
Remove-Item Env:\AEROTAXI_GUST_STD -ErrorAction SilentlyContinue
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command15-Aerotaxi-RANDOM-90 --num_envs 1024 --headless --seed 1001 --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v19/best_model_ppo_random_command15_4_8196.pt" --tag "seed_1001"

Write-Host "Evaluando seed_1002 (Experimento 4)..."
Remove-Item Env:\AEROTAXI_TAU_MS -ErrorAction SilentlyContinue
Remove-Item Env:\AEROTAXI_WIND_MS -ErrorAction SilentlyContinue
Remove-Item Env:\AEROTAXI_GUST_STD -ErrorAction SilentlyContinue
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command15-Aerotaxi-RANDOM-90 --num_envs 1024 --headless --seed 1002 --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v19/best_model_ppo_random_command15_4_8196.pt" --tag "seed_1002"

Write-Host "Evaluando seed_1003 (Experimento 4)..."
Remove-Item Env:\AEROTAXI_TAU_MS -ErrorAction SilentlyContinue
Remove-Item Env:\AEROTAXI_WIND_MS -ErrorAction SilentlyContinue
Remove-Item Env:\AEROTAXI_GUST_STD -ErrorAction SilentlyContinue
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command15-Aerotaxi-RANDOM-90 --num_envs 1024 --headless --seed 1003 --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v19/best_model_ppo_random_command15_4_8196.pt" --tag "seed_1003"

Write-Host "Evaluando seed_1004 (Experimento 4)..."
Remove-Item Env:\AEROTAXI_TAU_MS -ErrorAction SilentlyContinue
Remove-Item Env:\AEROTAXI_WIND_MS -ErrorAction SilentlyContinue
Remove-Item Env:\AEROTAXI_GUST_STD -ErrorAction SilentlyContinue
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command15-Aerotaxi-RANDOM-90 --num_envs 1024 --headless --seed 1004 --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v19/best_model_ppo_random_command15_4_8196.pt" --tag "seed_1004"

Write-Host "Evaluando seed_1005 (Experimento 4)..."
Remove-Item Env:\AEROTAXI_TAU_MS -ErrorAction SilentlyContinue
Remove-Item Env:\AEROTAXI_WIND_MS -ErrorAction SilentlyContinue
Remove-Item Env:\AEROTAXI_GUST_STD -ErrorAction SilentlyContinue
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command15-Aerotaxi-RANDOM-90 --num_envs 1024 --headless --seed 1005 --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v19/best_model_ppo_random_command15_4_8196.pt" --tag "seed_1005"

Write-Host "Evaluando tau_0ms (Experimento 4)..."
$env:AEROTAXI_TAU_MS = "0.0"
Remove-Item Env:\AEROTAXI_WIND_MS -ErrorAction SilentlyContinue
Remove-Item Env:\AEROTAXI_GUST_STD -ErrorAction SilentlyContinue
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command15-Aerotaxi-RANDOM-90 --num_envs 1024 --headless --seed 1001 --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v19/best_model_ppo_random_command15_4_8196.pt" --tag "tau_0ms"

Write-Host "Evaluando tau_20ms (Experimento 4)..."
$env:AEROTAXI_TAU_MS = "20.0"
Remove-Item Env:\AEROTAXI_WIND_MS -ErrorAction SilentlyContinue
Remove-Item Env:\AEROTAXI_GUST_STD -ErrorAction SilentlyContinue
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command15-Aerotaxi-RANDOM-90 --num_envs 1024 --headless --seed 1001 --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v19/best_model_ppo_random_command15_4_8196.pt" --tag "tau_20ms"

Write-Host "Evaluando tau_40ms (Experimento 4)..."
$env:AEROTAXI_TAU_MS = "40.0"
Remove-Item Env:\AEROTAXI_WIND_MS -ErrorAction SilentlyContinue
Remove-Item Env:\AEROTAXI_GUST_STD -ErrorAction SilentlyContinue
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command15-Aerotaxi-RANDOM-90 --num_envs 1024 --headless --seed 1001 --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v19/best_model_ppo_random_command15_4_8196.pt" --tag "tau_40ms"

Write-Host "Evaluando tau_80ms (Experimento 4)..."
$env:AEROTAXI_TAU_MS = "80.0"
Remove-Item Env:\AEROTAXI_WIND_MS -ErrorAction SilentlyContinue
Remove-Item Env:\AEROTAXI_GUST_STD -ErrorAction SilentlyContinue
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command15-Aerotaxi-RANDOM-90 --num_envs 1024 --headless --seed 1001 --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v19/best_model_ppo_random_command15_4_8196.pt" --tag "tau_80ms"

Write-Host "Evaluando tau_120ms (Experimento 4)..."
$env:AEROTAXI_TAU_MS = "120.0"
Remove-Item Env:\AEROTAXI_WIND_MS -ErrorAction SilentlyContinue
Remove-Item Env:\AEROTAXI_GUST_STD -ErrorAction SilentlyContinue
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command15-Aerotaxi-RANDOM-90 --num_envs 1024 --headless --seed 1001 --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v19/best_model_ppo_random_command15_4_8196.pt" --tag "tau_120ms"

Write-Host "Evaluando wind_0ms (Experimento 4)..."
Remove-Item Env:\AEROTAXI_TAU_MS -ErrorAction SilentlyContinue
$env:AEROTAXI_WIND_MS = "0.0"
$env:AEROTAXI_GUST_STD = "0.0"
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command15-Aerotaxi-RANDOM-90 --num_envs 1024 --headless --seed 1001 --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v19/best_model_ppo_random_command15_4_8196.pt" --tag "wind_0ms"

Write-Host "Evaluando wind_4ms (Experimento 4)..."
Remove-Item Env:\AEROTAXI_TAU_MS -ErrorAction SilentlyContinue
$env:AEROTAXI_WIND_MS = "4.0"
$env:AEROTAXI_GUST_STD = "1.5"
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command15-Aerotaxi-RANDOM-90 --num_envs 1024 --headless --seed 1001 --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v19/best_model_ppo_random_command15_4_8196.pt" --tag "wind_4ms"

Write-Host "Evaluando wind_8ms (Experimento 4)..."
Remove-Item Env:\AEROTAXI_TAU_MS -ErrorAction SilentlyContinue
$env:AEROTAXI_WIND_MS = "8.0"
$env:AEROTAXI_GUST_STD = "2.0"
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command15-Aerotaxi-RANDOM-90 --num_envs 1024 --headless --seed 1001 --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v19/best_model_ppo_random_command15_4_8196.pt" --tag "wind_8ms"

Write-Host "Evaluando wind_12ms (Experimento 4)..."
Remove-Item Env:\AEROTAXI_TAU_MS -ErrorAction SilentlyContinue
$env:AEROTAXI_WIND_MS = "12.0"
$env:AEROTAXI_GUST_STD = "3.5"
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command15-Aerotaxi-RANDOM-90 --num_envs 1024 --headless --seed 1001 --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v19/best_model_ppo_random_command15_4_8196.pt" --tag "wind_12ms"

Write-Host "Evaluando wind_16ms (Experimento 4)..."
Remove-Item Env:\AEROTAXI_TAU_MS -ErrorAction SilentlyContinue
$env:AEROTAXI_WIND_MS = "16.0"
$env:AEROTAXI_GUST_STD = "5.0"
& "C:/Users/Teresa/Desktop/RuralData/IsaacLab/IsaacLab/isaaclab.bat" -p "C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl/play.py" --task Isaac-Command15-Aerotaxi-RANDOM-90 --num_envs 1024 --headless --seed 1001 --checkpoint "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v19/best_model_ppo_random_command15_4_8196.pt" --tag "wind_16ms"

Remove-Item Env:\AEROTAXI_TAU_MS -ErrorAction SilentlyContinue
Remove-Item Env:\AEROTAXI_WIND_MS -ErrorAction SilentlyContinue
Remove-Item Env:\AEROTAXI_GUST_STD -ErrorAction SilentlyContinue
