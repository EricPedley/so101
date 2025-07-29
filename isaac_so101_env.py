"""

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p isaac_lab_main.py

"""

"""Launch Isaac Sim Simulator first."""


import argparse

from isaaclab.app import AppLauncher

# create argparser
parser = argparse.ArgumentParser(description="Tutorial on creating an empty stage.")
parser.add_argument("--num_envs", type=int, default=16, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import AssetBaseCfg, Articulation
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.sim import SimulationCfg, SimulationContext
import isaacsim.core.utils.prims as prim_utils
from isaaclab.sim.utils import attach_stage_to_usd_context

import isaaclab.sim as sim_utils
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
import isaacsim.core.utils.stage as stage_utils
import torch

from pathlib import Path
from isaaclab.utils import configclass
import isaaclab.envs.mdp as mdp
from isaaclab.envs import ManagerBasedRLEnv, ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.vec_env import VecNormalize

from datetime import datetime
import os
from isaaclab_rl.sb3 import Sb3VecEnvWrapper, process_sb3_cfg

so101_usd_path = Path(__file__).parent / 'models/SO101/so101_new_calib/so101_new_calib.usd'

joint_names = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

SO101_CONFIG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=str(so101_usd_path),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            key: 0.0 for key in joint_names
        },
        pos=(0.0, 0.0, 0.0),
    ),
    actuators={
        'defualt_config': ImplicitActuatorCfg(
            joint_names_expr=joint_names,
            effort_limit_sim=100.0,
            velocity_limit_sim=100.0,
            stiffness=10000.0,
            damping=100.0,
        ) 
    },
)

@configclass
class ActionsCfg:
    """Action specifications for the environment."""

    # joint_efforts = mdp.JointEffortActionCfg(asset_name="robot", joint_names=joint_names, scale=5.0)
    joint_efforts = mdp.JointPositionActionCfg(asset_name="robot", joint_names=joint_names, scale=1.0)

@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    # (1) Time out
    time_out = DoneTerm(func=mdp.time_out, time_out=True)

@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    # on reset
    reset_joint_positions = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=joint_names),
            "position_range": (0.0, 0.0),
            "velocity_range": (0.0, 0.0),
        },
    )

def reward_fn(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    gripper_index = asset.find_bodies('gripper_link')[0]
    base_idx = asset.find_bodies('base_link')[0]

    # Get the position in world frame
    gripper_link_position = asset.data.body_pos_w[:, gripper_index, :]  # Shape: [num_envs, 3]
    base_link_position = asset.data.body_pos_w[:, base_idx, :]  # Shape: [num_envs, 3]

    # print(f"Gripper link position: {gripper_link_position.shape}")
    return gripper_link_position[:,0,2]#-torch.linalg.vector_norm(gripper_link_position - base_link_position - torch.tensor([0.0, 0.0, 1.0], device=env.device))

@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # (2) Failure penalty
    # terminating = RewTerm(func=mdp.is_terminated, weight=-2.0)
    # (3) Primary task: keep pole upright
    gripper_pos = RewTerm(
        func=reward_fn,
        weight=1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=joint_names)},
    )

class SO101SceneCfg(InteractiveSceneCfg):
    """Designs the scene by spawning ground plane, light, objects and meshes from usd files."""
    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100.0, 100.0)),
    )

    # cartpole
    robot: ArticulationCfg = SO101_CONFIG.replace(prim_path="{ENV_REGEX_NS}/Robot")

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )


@configclass
class SO101EnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the cartpole environment."""

    # Scene settings
    scene = SO101SceneCfg(num_envs=1024, env_spacing=2.5)
    # Basic settings
    observations = ObservationsCfg()
    actions = ActionsCfg()
    events = EventCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        """Post initialization."""
        # viewer settings
        self.viewer.eye = [4.5, 0.0, 6.0]
        self.viewer.lookat = [0.0, 0.0, 2.0]
        # general settings
        self.decimation = 2
        self.episode_length_s = 3
        # simulation settings
        self.sim.dt = 1 / 60
        self.sim.render_interval = self.decimation


def main():
    """Main function."""
    # parse the arguments
    env_cfg = SO101EnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.sim.device = args_cli.device
    env_cfg.sim.create_stage_in_memory = True
    # setup base environment
    env = ManagerBasedRLEnv(cfg=env_cfg)

    env = Sb3VecEnvWrapper(env)

    # create agent from stable baselines
    agent = PPO('MlpPolicy', env, verbose=1, device='cpu')

    run_info = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_root_path = os.path.abspath(os.path.join("logs", "sb3", 'so101'))
    print(f"[INFO] Logging experiment in directory: {log_root_path}")
    # The Ray Tune workflow extracts experiment name using the logging line below, hence, do not change it (see PR #2346, comment-2819298849)
    print(f"Exact experiment name requested from command line: {run_info}")
    log_dir = os.path.join(log_root_path, run_info)
    new_logger = configure(log_dir, ["stdout", "tensorboard"])
    agent.set_logger(new_logger)
    checkpoint_callback = CheckpointCallback(save_freq=100, save_path=log_dir, name_prefix="model", verbose=2)
    agent.learn(total_timesteps=1_000_000, callback=checkpoint_callback, log_interval=1)
   
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
