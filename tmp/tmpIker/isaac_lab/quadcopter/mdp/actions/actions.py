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

    _lin_scale: float
    """The scaling factor applied to the linear input action."""

    _ang_scale: float
    """The scaling factor applied to the angular input action."""

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

        # Parse scales
        self._lin_scale = float(cfg.lin_scale)
        self._ang_scale = float(cfg.ang_scale)

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
        # Store raw actions
        self._raw_actions[:] = actions

        # Get absolute value for linear force actions
        self._raw_actions[:, 1:] = self._raw_actions[:, 1:].abs()

        # Assign the Z-axis actions to the forces/torques tensors
        self._forces[:, 1:, 2] = self._raw_actions[:, 1:] * self._lin_scale
        self._torques[:, 0, 2] = self._raw_actions[:, 0] * self._ang_scale

    def apply_actions(self):
        # Apply forces and torques at the position of the joints
        self._asset.root_physx_view.apply_forces_and_torques_at_position(
            force_data=self._forces,
            torque_data=None,
            position_data=self._positions,
            indices=self._indices,
            is_global=False
        )
