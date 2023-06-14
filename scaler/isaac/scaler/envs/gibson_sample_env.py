import gym.spaces
import math
import os
import random
import torch
from typing import List

import omni.isaac.core.utils.prims as prim_utils

import omni.isaac.orbit.utils.kit as kit_utils
from omni.isaac.orbit.utils.assets import ISAAC_NUCLEUS_DIR

from omni.isaac.orbit_envs.isaac_env import IsaacEnv, VecEnvIndices, VecEnvObs
from omni.isaac.orbit_envs.isaac_env_cfg import EnvCfg, IsaacEnvCfg

from .gibson_sample_env_cfg import GibsonSampleEnvCfg


class GibsonSampleEnv(IsaacEnv):
    def __init__(self, cfg: GibsonSampleEnvCfg = None, **kwargs):
        self.cfg = cfg
        super().__init__(cfg, **kwargs)

        self.sim.reset()
        # compute the observation space
        self.observation_space = gym.spaces.Box(low=-math.inf, high=math.inf, shape=(4,))
        # compute the action space
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(1,))

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
        prim_utils.create_prim("/env_0/scene", usd_path=scene_usd_path, orientation=[0, 0, 0.707, 0.707])

        return ["/World/defaultGroundPlane", "/env_0/scene"]

    def _reset_idx(self, env_ids: VecEnvIndices):
        pass

    def _step_impl(self, actions: torch.Tensor):
        self.sim.step(render=self.enable_render)

    def _get_observations(self) -> VecEnvObs:
        pass
