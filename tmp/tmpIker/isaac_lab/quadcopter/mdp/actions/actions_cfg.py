# Related third party imports
from isaaclab.managers import ActionTerm
from isaaclab.managers import ActionTermCfg
from isaaclab.utils import configclass
from dataclasses import MISSING

# User specific imports
from . import actions


@configclass
class QuadcopterMotorActionCfg(ActionTermCfg):
    class_type: type[ActionTerm] = actions.QuadcopterMotorAction

    """List of joint names that the action will be mapped to."""
    joint_names: list[str] = MISSING

    """Scale factor for the linear force action"""
    lin_scale: float = 1.0

    """Scale factor for the angular torque action"""
    ang_scale: float = 0.005
