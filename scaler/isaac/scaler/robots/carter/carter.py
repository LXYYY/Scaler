from typing import Dict, Optional, Sequence

from omni.isaac.core.materials import PhysicsMaterial
from pxr import PhysxSchema

import omni.isaac.orbit.utils.kit as kit_utils
from omni.isaac.orbit.robots.robot_base import RobotBase

from .carter_cfg import CarterCfg
from .carter_data import CarterData


class Carter(RobotBase):
    """Carter robot class."""

    cfg: CarterCfg

    def __init__(self, cfg: CarterCfg):
        super().__init__(cfg)
        self._data= CarterData()

    @property
    def data(self) -> CarterCfg:
        """Data related to articulation."""
        return self._data

    def spawn(self, prim_path: str, translation: Sequence[float] = None, orientation: Sequence[float] = None):
        # spawn the robot and set its location
        super().spawn(prim_path, translation, orientation)
        # alter physics collision properties
        kit_utils.set_nested_collision_properties(prim_path, contact_offset=0.02, rest_offset=0.0)

    def initialize(self, prim_paths_expr: Optional[str] = None):
        # initialize parent handles
        super().initialize(prim_paths_expr)