# Related third party imports
from isaaclab.managers import ActionTerm
from isaaclab.managers import ActionTermCfg
from isaaclab.utils import configclass
from dataclasses import MISSING

# User specific imports
from . import apply_force_actions


@configclass
class QuadcopterMotorActionCfg(ActionTermCfg):
    class_type: type[ActionTerm] = apply_force_actions.QuadcopterMotorAction

    """List of joint names that the action will be mapped to."""
    joint_names: list[str] = MISSING

    """Scale factor for the action"""
    scale: float = 1.0
