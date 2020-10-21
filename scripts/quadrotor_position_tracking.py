from ruamel.yaml import YAML, dump, RoundTripDumper
from raisim_gym.env.RaisimGymVecEnv import RaisimGymVecEnv as Environment
from raisim_gym.env.env.hummingbird import __HUMMINGBIRD_RESOURCE_DIRECTORY__ as __RSCDIR__
from raisim_gym.algo.ppo2 import PPO2
from raisim_gym.archi.policies import MlpPolicy
from raisim_gym.helper.raisim_gym_helper import ConfigurationSaver, TensorboardLauncher
from _raisim_gym import RaisimGymEnv
import os
import math
import argparse
 
# Function to map the action value to true thrust range none negative to the range between 
# 0.5 to 4.5 scale

def action2Thrust(mylist):       
    _actionMean = 2.5
    _actionStd = 2
        
    Newthrust = mylist.astype(float)*_actionStd
    Newthrust += _actionMean
    return Newthrust

# configuration
parser = argparse.ArgumentParser()
parser.add_argument('--cfg', type=str, default=os.path.abspath(__RSCDIR__ + "/default_cfg.yaml"),
                    help='configuration file')
parser.add_argument('-m', '--mode', help='set mode either train or test', type=str, default='train')
parser.add_argument('-w', '--weight', help='trained weight path', type=str, default='')
args = parser.parse_args()
mode = args.mode
cfg_abs_path = parser.parse_args().cfg
cfg = YAML().load(open(cfg_abs_path, 'r'))

# save the configuration and other files
rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../'
log_dir = rsg_root + '/QuadrotorTrainingdata'
saver = ConfigurationSaver(log_dir=log_dir+'/quadrotor_position_tracking',
                           save_items=[rsg_root+'raisim_gym/env/env/hummingbird/Environment.hpp', cfg_abs_path])

# create environment from the configuration file
if args.mode == "test": # for test mode, force # of env to 1
    cfg['environment']['num_envs'] = 1
env = Environment(RaisimGymEnv(__RSCDIR__, dump(cfg['environment'], Dumper=RoundTripDumper)))


if mode == 'train':

# Get algorithm
    model = PPO2(
        tensorboard_log=saver.data_dir,
        policy=MlpPolicy,
        policy_kwargs=dict(net_arch=[dict(pi=[96, 64], vf=[96, 64])]),
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
    # Originally the total timestep is 500000000
    # 10 zeros for nupdates to be 4000
    # 1000000000 is 2000 iterations and so 
    # 2000000000 is 4000 iterations. 
    model.learn(
        total_timesteps=10000000000,  
        eval_every_n=50, 
        log_dir=saver.data_dir, 
        record_video=cfg['record_video']
    )
    model.save(saver.data_dir)
        # Need this line if you want to keep tensorflow alive after training
    input("Press Enter to exit... Tensorboard will be closed after exit\n")

# Testing mode with a trained weight
else:
    weight_path = args.weight
    if weight_path == "":
        print("Can't find trained weight, please provide a trained weight with --weight switch\n")
    else:
        print("Loaded weight from {}\n".format(weight_path))
        model = PPO2.load(weight_path)
    obs = env.reset()
    running_reward = 0.0
    ep_len = 0
    for _ in range(100000):
        env.wrapper.showWindow()
        action, _ = model.predict(obs)
        print("Action: {}".format(action))
        
        Thrust = action2Thrust(action)
        print("Thrust: {}".format(Thrust))
        obs, reward, done, infos = env.step(action, visualize=True)
        print("obs: {}".format(obs))
        print("reward: {}".format(reward))
        print("done: {}".format(done))
        print("infos: {}".format(infos))
        running_reward += reward[0]
        ep_len += 1
        if done:
            print("Episode Reward: {:.2f}".format(running_reward))
            print("Episode Length", ep_len)
            running_reward = 0.0
            ep_len = 0

