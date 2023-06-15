import gym.spaces
import math
import os
import random
import torch
import carb
from typing import List

import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core import scenes,prims

import omni.isaac.orbit.utils.kit as kit_utils
from omni.isaac.orbit.utils.assets import ISAAC_NUCLEUS_DIR
from omni.isaac.orbit_envs.isaac_env import IsaacEnv, VecEnvIndices, VecEnvObs
from omni.isaac.orbit_envs.isaac_env_cfg import EnvCfg, IsaacEnvCfg
from omni.isaac.orbit.robots.legged_robot import LeggedRobot
from omni.isaac.orbit.robots.config.anymal import ANYMAL_B_CFG, ANYMAL_C_CFG

from .gibson_sample_env_cfg import GibsonSampleEnvCfg
from scaler.isaac.scaler.robots.carter.carter import Carter
from scaler.isaac.scaler.robots.carter.carter_cfg import CarterCfg, CarterV1Cfg


class GibsonSampleEnv(IsaacEnv):
    def __init__(self, cfg: GibsonSampleEnvCfg = None, **kwargs):
        self.cfg = cfg
        # self.robot= LeggedRobot(cfg=ANYMAL_B_CFG)
        robot_cfg = CarterV1Cfg

        from omni.isaac.core.utils.nucleus import get_assets_root_path
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        robot_cfg.meta_info.usd_path = assets_root_path + robot_cfg.meta_info.usd_path
        self.robot = Carter(cfg=robot_cfg)

        self.scene=scenes.Scene()

        super().__init__(cfg, **kwargs)

        # compute the observation space
        self.observation_space = gym.spaces.Box(low=-math.inf, high=math.inf, shape=(4,))
        # compute the action space
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(1,))


        self.sim.reset()

        # self.robot.update_buffers()

    def _design_scene(self) -> List[str]:
        kit_utils.create_ground_plane("/World/defaultGroundPlane", z_position=0.0)
        scene_usd_path = None
        if self.cfg.scene.scene_id == "random":
            # get all the scene paths and pick one at random
            scene_paths = []
            for root, dirs, files in os.walk(self.cfg.scene.dataset_path):
                for file in files:
                    if file.endswith(".usd"):
                        scene_paths.append(os.path.join(root, file))
            scene_usd_path = random.choice(scene_paths)
        else:
            scene_usd_path = os.path.join(
                self.cfg.scene.dataset_path, self.cfg.scene.scene_id + "_converted", "mesh.usd"
            )

        # create prim and put z axis up
        scene=prim_utils.create_prim("/env_0/scene", usd_path=scene_usd_path, orientation=[0, 0, 0.707, 0.707])

        self.robot.spawn(self.template_env_ns+"/robot")

        return ["/World/defaultGroundPlane", "/World/scene"]

    def _reset_idx(self, env_ids: VecEnvIndices):
        pass

    def _step_impl(self, actions: torch.Tensor):
        self.sim.step(render=self.enable_render)

    def _get_observations(self) -> VecEnvObs:
        pass
