from raisim_gym.archi.policies import linear
import tensorflow as tf
import math
import numpy as np
from stable_baselines.common.input import observation_input
from stable_baselines.common.distributions import make_proba_dist_type


class AnymalArchitecture:

    def __init__(self, cfg, env, arch_type, graph, sess):

        assert arch_type is 'train' or 'act', 'type should be either "train" or "act"'

        cfg_env = cfg['environment']
        cfg_arch = cfg['architecture']

        if arch_type is 'train':
            self.num_steps = math.floor(cfg_env['max_time'] / cfg_env['control_dt'])
        else:
            self.num_steps = 1

        self.observation_space = env.observation_space
        self.action_space = env.action_space
        self.pdtype = make_proba_dist_type(self.action_space)
        self.n_env = cfg["environment"]["num_envs"]
        self.graph = graph

        with self.graph.as_default():
            with tf.variable_scope("model", reuse=tf.AUTO_REUSE):

                batch_size = self.num_steps*self.n_env

                if arch_type is 'train':
                    batch_size /= cfg["algorithm"]["minibatch"]

                self.obs_ph, self.processed_obs = observation_input(self.observation_space, batch_size, scale=False)

                act_fun = tf.nn.relu

                pi_latent = self.obs_ph
                vi_latent = self.obs_ph

                for idx, dec_layer_size in enumerate(cfg_arch["pi_net"]):
                    pi_latent = act_fun(linear(pi_latent, "pi_net_fc{}".format(idx), dec_layer_size, init_scale=np.sqrt(2)))

                for idx, dec_layer_size in enumerate(cfg_arch["vi_net"]):
                    vi_latent = act_fun(linear(vi_latent, "vi_net_fc{}".format(idx), dec_layer_size, init_scale=np.sqrt(2)))

                self.value_fn = linear(vi_latent, 'vf', 1)
                self.value = self.value_fn[:, 0]
                self.proba_distribution, self.policy, self.q_value = \
                    self.pdtype.proba_distribution_from_latent(pi_latent, vi_latent, init_scale=0.01)
                self.action_ph = self.pdtype.sample_placeholder([None], name="action_ph")
                self.masks_ph = tf.placeholder(tf.float32, [None], "masks_ph")
                self.action = self.proba_distribution.sample()
                self.neglogp = self.proba_distribution.neglogp(self.action)

        self.initial_state = None
        self.sess = sess

        # continuous action diagonal covariance
        self.policy_proba = [self.proba_distribution.mean, self.proba_distribution.std]


class AnymalControllerModel:

    def __init__(self, arch):
        self.arch = arch
        self.sess = arch.sess
        self.n_env = arch.n_env
        self.action_ph = arch.action_ph
        self.obs_ph = arch.obs_ph
        self.proba_distribution = arch.proba_distribution
        # self._value = graph.value  # predicted value
        self.initial_state = None
        self.action_space = arch.action_space
        self.observation_space = arch.observation_space
        self.value = arch.value
        self.masks_ph = arch.masks_ph

    def step(self, obs, states, dones, deterministic=False):
        if deterministic:
            action, value, neglogp = self.sess.run([self.proba_distribution.mode(), self.value, self.arch.neglogp],
                                                   {self.obs_ph: obs})
        else:
            action, value, neglogp = self.sess.run([self.arch.action, self.value, self.arch.neglogp],
                                                   {self.obs_ph: obs})
        return action, value, self.initial_state, neglogp

    def proba_step(self, obs, state=None, mask=None):
        return self.sess.run(self.arch.policy_proba, {self.obs_ph: obs})

    def value_eval(self, obs, states, dones):
        return self.sess.run(self.value, {self.obs_ph: obs})
