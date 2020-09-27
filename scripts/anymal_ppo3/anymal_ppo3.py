import os
import math
import argparse
import time
import tensorflow as tf
from ruamel.yaml import YAML, dump, RoundTripDumper
from raisim_gym.env.RaisimGymVecEnv import RaisimGymVecEnv as Environment
from raisim_gym.env.env.ANYmal import __ANYMAL_RESOURCE_DIRECTORY__ as __RSCDIR__
from raisim_gym.algo.ppo3 import PPO3
from raisim_gym.helper.raisim_gym_helper import ConfigurationSaver, TensorboardLauncher
from stable_baselines.common import SetVerbosity, TensorboardWriter
from _raisim_gym import RaisimGymEnv
from model import model as model
from stable_baselines.common import tf_util
import multiprocessing


tf.logging.set_verbosity(tf.logging.ERROR)

# this task path
task_path = os.path.dirname(os.path.abspath(__file__))

# configuration
parser = argparse.ArgumentParser()
parser.add_argument('--cfg', type=str, default=os.path.abspath(task_path + "/default_cfg.yaml"),
                    help='configuration file')
cfg_abs_path = parser.parse_args().cfg
cfg = YAML().load(open(cfg_abs_path, 'r'))

# save the configuration and other files
rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../../'
log_dir = rsg_root + '/data'
saver = ConfigurationSaver(log_dir=log_dir+'/ANYmal_blind_locomotion',
                           save_items=[rsg_root+'raisim_gym/env/env/ANYmal/Environment.hpp', cfg_abs_path])

# create environment from the configuration file
num_envs = cfg['environment']['num_envs']
num_steps = math.floor(cfg['environment']['max_time'] / cfg['environment']['control_dt'])
env = Environment(RaisimGymEnv(__RSCDIR__, dump(cfg['environment'], Dumper=RoundTripDumper)))

tf_graph = tf.Graph()

with tf_graph.as_default():
    sess = tf_util.make_session(num_cpu=multiprocessing.cpu_count(), graph=tf_graph)

train_graph = model.AnymalArchitecture(cfg=cfg, env=env, graph=tf_graph, sess=sess, arch_type='train')
act_graph = model.AnymalArchitecture(cfg=cfg, env=env, graph=tf_graph, sess=sess, arch_type='act')

train_model = model.AnymalControllerModel(train_graph)
act_model = model.AnymalControllerModel(act_graph)

# Get algorithm
model = PPO3(
    tensorboard_log=saver.data_dir,
    num_envs=num_envs,
    graph=tf_graph,
    sess=sess,
    train_model=train_model,
    act_model=act_model,
    gamma=0.998,
    n_steps=num_steps,
    ent_coef=0,
    learning_rate=1e-3,
    vf_coef=0.5,
    max_grad_norm=0.5,
    lam=0.95,
    nminibatches=5,
    noptepochs=10,
    cliprange=0.2,
    verbose=1,
)

# tensorboard
# Make sure that your chrome browser is already on.
TensorboardLauncher(saver.data_dir + '/_1')

# PPO run
total_steps = model.n_steps * model.n_envs

# extra variables for tensorboard

with SetVerbosity(3), TensorboardWriter(tf_graph, saver.data_dir, "", True) as writer:
    for update in range(10000):
        start = time.time()
        obs = env.reset()
        reward_sum = 0

        for step in range(model.n_steps):
            action = model.get_next_action(obs)
            obs, rewards, dones, info = env.step(action)
            reward_sum += sum(rewards)
            model.collect(obs, rewards, dones)

        model.learn(update=update, obs=obs, writer=writer, nupdates=10000)
        end = time.time()

        average_performance = reward_sum / total_steps

        print('----------------------------------------------------')
        print('{:>6}th iteration'.format(update))
        print('{:<40} {:>6}'.format("average reward: ", '{:6.4f}'.format(average_performance)))
        print('{:<40} {:>6}'.format("time elapsed in this iteration: ", '{:6.4f}'.format(end-start)))
        print('{:<40} {:>6}'.format("fps: ", '{:6.0f}'.format(num_envs*num_steps/(end-start))))
        print('----------------------------------------------------\n')
