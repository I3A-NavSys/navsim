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

    cfg: actions_cfg.QuadcopterMotorActionCfg
    """The configuration of the action term."""

    _asset: Articulation
    """The articulation asset on which the action term is applied."""

    _scale: float
    """The scaling factor applied to the input action."""

    _env: ManagerBasedRLEnv
    """ The environment in which the action term is applied."""

    def __init__(self, cfg: actions_cfg.QuadcopterMotorActionCfg,
                 env: ManagerBasedRLEnv):
        # Initialize the action term
        super().__init__(cfg, env)

        # Resolve the joints over which the action term is applied
        self._joint_ids, self._joint_names = self._asset.find_joints(
            self.cfg.joint_names)
        self._num_joints = len(self._joint_ids)

        # Avoid indexing across all joints for efficiency
        if self._num_joints == self._asset.num_joints:
            self._joint_ids = slice(None)

        self.indices = torch.arange(self.num_envs, device=self.device)

        # Create tensors for raw and processed actions
        self._raw_actions = torch.zeros(self.num_envs, self.action_dim,
                                        device=self.device)
        self._processed_actions = torch.zeros(env.num_envs, 5, 3, 
                                              device=self.device)

        # Parse scale
        self._scale = float(cfg.scale)

    """
    Properties.
    """

    @property
    def action_dim(self) -> int:
        """Return the action dimension."""
        return self._num_joints

    @property
    def raw_actions(self) -> torch.Tensor:
        """Return the raw actions."""
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        """Return the processed actions."""
        return self._processed_actions

    """
    Operations.
    """

    def process_actions(self, actions: torch.Tensor):
        # Scale and store raw actions
        self._raw_actions[:] = actions.abs() * self._scale

        # Assign the scaled raw actions to the processed actions tensor
        self.processed_actions[:, 1:, 2] = self.raw_actions

    def apply_actions(self):
        # Apply forces and torques at the position of the joints
        self._asset.root_physx_view.apply_forces_and_torques_at_position(
            force_data=self.processed_actions,
            torque_data=None,
            position_data=None,
            indices=self.indices,
            is_global=False
        )

    def reset(self, env_ids: Sequence[int] | None = None) -> None:
        self._raw_actions[env_ids] = 0.0
