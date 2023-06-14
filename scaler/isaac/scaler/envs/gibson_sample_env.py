import gym.spaces
import math

from omni.isaac.orbit_envs.isaac_env import IsaacEnv, VecEnvIndices, VecEnvObs
from omni.isaac.orbit_envs.isaac_env_cfg import EnvCfg, IsaacEnvCfg

class GibsonSampleEnv(IsaacEnv):
    def __init__(self, cfg: dict, **kwargs):
        self.cfg_dict = cfg.copy()
        isaac_cfg = IsaacEnvCfg(
            env=EnvCfg(num_envs=self.cfg_dict["env"]["num_envs"], env_spacing=self.cfg_dict["env"]["env_spacing"])
        )
        isaac_cfg.sim.from_dict(self.cfg_dict["sim"])
        super().__init__(isaac_cfg, **kwargs)

        self.sim.reset()
        # compute the observation space
        self.observation_space = gym.spaces.Box(low=-math.inf, high=math.inf, shape=(4,))
        # compute the action space
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(1,))
        # store maximum episode length
        self.max_episode_length = self.cfg_dict["env"]["episode_length"]

