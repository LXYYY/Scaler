from omni.isaac.orbit.utils import configclass

from omni.isaac.orbit_envs.isaac_env_cfg import EnvCfg, IsaacEnvCfg, PhysxCfg, SimCfg, ViewerCfg

from scaler.isaac.scaler.robots.carter.carter_cfg import CarterCfg, CarterV1Cfg


@configclass
class SceneCfg:
    """Scene properties."""

    dataset_path: str = "/workspaces/data/gibson/gibson_fullplus"
    scene_id: str = "random"


@configclass
class GibsonSampleEnvCfg(IsaacEnvCfg):
    env: EnvCfg = EnvCfg(num_envs=10, env_spacing=1, episode_length_s=5.0)
    scene: SceneCfg = SceneCfg()
    viewer: ViewerCfg = ViewerCfg(debug_vis=True, eye=(7.5, 7.5, 7.5), lookat=(0.0, 0.0, 0.0))
    sim: SimCfg = SimCfg(
        dt=0.01,
        substeps=1,
        physx=PhysxCfg(
            gpu_found_lost_aggregate_pairs_capacity=1024 * 1024 * 4,
            gpu_total_aggregate_pairs_capacity=16 * 1024,
            friction_correlation_distance=0.00625,
            friction_offset_threshold=0.01,
            bounce_threshold_velocity=0.2,
        ),
    )

    robot: CarterCfg = CarterV1Cfg
