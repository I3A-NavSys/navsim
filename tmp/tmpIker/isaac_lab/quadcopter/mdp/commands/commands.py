# Standard library imports
from __future__ import annotations
from typing import TYPE_CHECKING

# Related third party imports
import torch
from isaaclab.managers import CommandTerm
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.assets import Articulation

# Local application/library specific imports
if TYPE_CHECKING:
    from . import commands_cfg

class UAVCommandTerm(CommandTerm):
    """Command term for the UAV."""
    _asset: Articulation

    def __init__(self, cfg: commands_cfg.UAVCommandTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._command = torch.zeros(env.num_envs, 4, device=self.device)

    @property
    def command(self) -> torch.Tensor:
        """The command tensor. Shape is (num_envs, command_dim)."""
        return self._command
    
    def _update_metrics(self):
        pass

    def _resample_command(self, env_ids):
        """Resample the command for the given enviroment IDs."""
        self._command[env_ids, :] = torch.zeros(4, device=self.device)

    def _update_command(self):
        pass