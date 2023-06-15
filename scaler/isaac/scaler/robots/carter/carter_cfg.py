from omni.isaac.orbit.utils import configclass
from omni.isaac.orbit.utils.assets import ISAAC_ORBIT_NUCLEUS_DIR

from omni.isaac.orbit.robots.robot_base_cfg import RobotBaseCfg
from omni.isaac.orbit.actuators.group import ActuatorControlCfg, ActuatorGroupCfg
from omni.isaac.orbit.actuators.model import DCMotorCfg


@configclass
class CarterCfg(RobotBaseCfg):
    pass

CARTER_V1_ACTUATOR_CFG = DCMotorCfg(
    peak_motor_torque=120.0, motor_torque_limit=80.0, motor_velocity_limit=7.5, gear_ratio=1.0
)

CARTER_V1_DEFAULT_GROUP_CFG = ActuatorGroupCfg(
    dof_names=["base"], model_cfg=CARTER_V1_ACTUATOR_CFG, control_cfg=ActuatorControlCfg(command_types=["p_rel"])
)

CarterV1Cfg = CarterCfg(
    meta_info=CarterCfg.MetaInfoCfg(usd_path="/Isaac/Robots/Carter/carter_v1.usd"),
    articulation_props=CarterCfg.ArticulationRootPropertiesCfg(
        enable_self_collisions=True, solver_position_iteration_count=4, solver_velocity_iteration_count=1
    ),
    actuator_groups={"base": CARTER_V1_DEFAULT_GROUP_CFG},
)
