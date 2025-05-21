# Related third party imports
from isaaclab.managers import CommandTermCfg
from isaaclab.utils import configclass
from dataclasses import MISSING

# User specific imports
from . import commands

@configclass
class UAVCommandTermCfg(CommandTermCfg):
    """Command term configuration for the UAV."""

    class_type: type =  commands.UAVCommandTerm
    """Class type of the command term."""