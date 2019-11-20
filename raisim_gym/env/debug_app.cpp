//
// Created by jemin on 11/12/19.
// MIT License
//
// Copyright (c) 2019-2019 Robotic Systems Lab, ETH Zurich
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "Environment.hpp"
#include "VectorizedEnvironment.hpp"

using namespace raisim;

int main(int argc, char *argv[]) {
  RSFATAL_IF(argc != 4, "got "<<argc<<" arguments. "<<"This executable takes three arguments: 1. resource directory, 2. configuration file, 3. render or no_render to control rendering")

  std::string resourceDir(argv[1]), cfgFile(argv[2]);
  YAML::Node config = YAML::LoadFile(cfgFile);
  std::stringstream config_str;

  if(std::string(argv[3]) == "no_render") {
    config["environment"]["render"] = false;
    std::cout<<argv[3]<<std::endl;
  } else {
    std::cout<<argv[3]<<std::endl;
    config["environment"]["render"] = true;
  }

  config_str << config["environment"];

  VectorizedEnvironment<ENVIRONMENT> vecEnv(resourceDir, config_str.str());
  vecEnv.init();

  EigenRowMajorMat observation(config["environment"]["num_envs"].as<int>(), vecEnv.getObDim());
  EigenRowMajorMat action(config["environment"]["num_envs"].as<int>(), vecEnv.getActionDim());
  EigenVec reward(config["environment"]["num_envs"].as<int>(), 1);
  EigenBoolVec dones(config["environment"]["num_envs"].as<int>(), 1);
  EigenRowMajorMat extra_info(config["environment"]["num_envs"].as<int>(), vecEnv.getExtraInfoDim());
  action.setZero();

  Eigen::Ref<EigenRowMajorMat> ob_ref(observation), action_ref(action), extra_info_ref(extra_info);
  Eigen::Ref<EigenVec> reward_ref(reward);
  Eigen::Ref<EigenBoolVec> dones_ref(dones);

  vecEnv.reset(ob_ref);
  vecEnv.step(action_ref, ob_ref, reward_ref, dones_ref, extra_info_ref);

  return 0;
}