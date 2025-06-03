# Standard library imports
from __future__ import annotations
from typing import TYPE_CHECKING
from collections.abc import Sequence

# Related third party imports
import torch
from isaaclab.managers import ActionTerm
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.assets import Articulation
from isaaclab.managers import ActionTerm

# Local application/library specific imports
if TYPE_CHECKING:
    from . import actions_cfg


class QuadcopterMotorAction(ActionTerm):
    """Action term that applies forces to the motors of a quadcopter."""

    _asset: Articulation
    """The articulation asset on which the action term is applied."""

    _scale: float
    """The scaling factor applied to the input action."""

    def __init__(self, cfg: actions_cfg.QuadcopterMotorActionCfg,
                 env: ManagerBasedRLEnv):
        # Initialize the action term
        super().__init__(cfg, env)

        # Create raw actions and forces/torques tensors
        self._raw_actions = torch.zeros(self.num_envs, 5, device=self.device)
        self._forces = torch.zeros(env.num_envs, 5, 3, device=self.device)
        self._torques = torch.zeros(env.num_envs, 5, 3, device=self.device)

        # Tensor with the positions where forces/torques will be applied
        positions = torch.tensor([
            [0, 0, 0],
            [0.075, -0.075, 0],
            [0.075, 0.075, 0],
            [-0.075, -0.075, 0],
            [-0.075, 0.075, 0]
        ], device=self.device)

        # Create a view from position tensor to match environment size
        self._positions = positions.unsqueeze(0).expand(env.num_envs, -1, -1)

        # Create indexes tensor
        self._indices = torch.arange(self.num_envs, device=self.device)

        # Parse scale
        self._scale = float(cfg.scale)

    """
    Properties.
    """

    @property
    def action_dim(self) -> int:
        """Return the action dimension."""
        return self._raw_actions.shape[1]

    @property
    def raw_actions(self) -> torch.Tensor:
        """Return the raw actions."""
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        """Return the processed actions."""
        return torch.cat(self._forces, self._torques)

    """
    Operations.
    """

    def process_actions(self, actions: torch.Tensor):
        # Scale and store raw actions
        self._raw_actions[:] = actions.abs() * self._scale

        # Assign the Z-axis actions to the forces/torques tensors
        self._forces[:, 1:, 2] = self._raw_actions[:, 1:]
        self._torques[:, 0, 2] = self._raw_actions[:, 0]

        # TEST
        # self._forces[:, 1:3, 2] = 1.582533
        # self._forces[:, 3:5, 2] = 1.397467
        # self._torques[:, 0, 2] = 0.005

    def apply_actions(self):
        # Apply forces and torques at the position of the joints
        self._asset.root_physx_view.apply_forces_and_torques_at_position(
            force_data=self._forces,
            torque_data=self._torques,
            position_data=None,
            indices=self._indices,
            is_global=False
        )

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        self._raw_actions[env_ids] = 0.0
