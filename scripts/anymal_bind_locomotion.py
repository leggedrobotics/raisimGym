from ruamel.yaml import YAML, dump, RoundTripDumper
from raisim_gym.env.RaisimGymVecEnv import RaisimGymVecEnv as Environment
from raisim_gym.env.env.ANYmal import __ANYMAL_RESOURCE_DIRECTORY__ as __RSCDIR__
from raisim_gym.algo.ppo2 import PPO2
from raisim_gym.archi.policies import MlpPolicy
from raisim_gym.helper.raisim_gym_helper import ConfigurationSaver, TensorboardLauncher
from _raisim_gym import RaisimGymEnv
import os
import math
import argparse

# configuration
parser = argparse.ArgumentParser()
parser.add_argument('--cfg', type=str, default=os.path.abspath(__RSCDIR__ + "/default_cfg.yaml"),
                    help='configuration file')
cfg_abs_path = parser.parse_args().cfg
cfg = YAML().load(open(cfg_abs_path, 'r'))

# save the configuration and other files
rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../'
log_dir = rsg_root + '/data'
saver = ConfigurationSaver(log_dir=log_dir+'/ANYmal_blind_locomotion',
                           save_items=[rsg_root+'raisim_gym/env/env/ANYmal/Environment.hpp', cfg_abs_path])

# create environment from the configuration file
env = Environment(RaisimGymEnv(__RSCDIR__, dump(cfg['environment'], Dumper=RoundTripDumper)))

# Get algorithm
model = PPO2(
    tensorboard_log=saver.data_dir,
    policy=MlpPolicy,
    policy_kwargs=dict(net_arch=[dict(pi=[128, 128], vf=[128, 128])]),
    env=env,
    gamma=0.998,
    n_steps=math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt']),
    ent_coef=0,
    learning_rate=1e-3,
    vf_coef=0.5,
    max_grad_norm=0.5,
    lam=0.95,
    nminibatches=1,
    noptepochs=10,
    cliprange=0.2,
    verbose=1,
)

# tensorboard
# Make sure that your chrome browser is already on.
TensorboardLauncher(saver.data_dir + '/PPO2_1')

# PPO run
model.learn(total_timesteps=400000000, eval_every_n=50, log_dir=saver.data_dir, record_video=cfg['record_video'])

# Need this line if you want to keep tensorflow alive after training
input("Press Enter to exit... Tensorboard will be closed after exit\n")
