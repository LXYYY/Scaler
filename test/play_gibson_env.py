import argparse

from omni.isaac.kit import SimulationApp

# add argparse arguments
parser = argparse.ArgumentParser("Welcome to Orbit: Omniverse Robotics Environments!")
parser.add_argument("--headless", action="store_true", default=False, help="Force display off at all times.")
parser.add_argument("--cpu", action="store_true", default=False, help="Use CPU pipeline.")
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
args_cli = parser.parse_args()

# launch the simulator
config = {"headless": args_cli.headless}
simulation_app = SimulationApp(config)

from scaler.isaac.scaler.envs.gibson_sample_env import GibsonSampleEnv
from scaler.isaac.scaler.envs.gibson_sample_env_cfg import GibsonSampleEnvCfg


if __name__ == "__main__":
    cfg= GibsonSampleEnvCfg()

    # create the environment
    env = GibsonSampleEnv(cfg)

    # reset the environment
    env.reset()

    # run the environment for a few steps
    while True:
        # take a random action
        action = env.action_space.sample()
        obs, rewards, dones, info = env.step(action)

        if dones.any():
            env.reset()

    # close the environment
    env.close()

    # close the simulation
    simulation_app.close()

