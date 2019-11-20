//
// Created by jemin on 3/27/19.
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

#ifndef SRC_RAISIMGYMENV_HPP
#define SRC_RAISIMGYMENV_HPP

#include <vector>
#include <memory>
#include <unordered_map>
#include <Eigen/Core>
#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "yaml-cpp/yaml.h"
#include "raisim/OgreVis.hpp"

#define __RSG_MAKE_STR(x) #x
#define _RSG_MAKE_STR(x) __RSG_MAKE_STR(x)
#define RSG_MAKE_STR(x) _RSG_MAKE_STR(x)

#define READ_YAML(a, b, c) RSFATAL_IF(!c, "Node "<<RSG_MAKE_STR(c)<<" doesn't exist") \
                           b = c.as<a>();

namespace raisim {

using Dtype=float;
using EigenRowMajorMat=Eigen::Matrix<Dtype, -1, -1, Eigen::RowMajor>;
using EigenVec=Eigen::Matrix<Dtype, -1, 1>;
using EigenBoolVec=Eigen::Matrix<bool, -1, 1>;

class RaisimGymEnv {

 public:
  explicit RaisimGymEnv (std::string resourceDir, const YAML::Node& cfg) :
      resourceDir_(std::move(resourceDir)), cfg_(cfg) {
    world_ = std::make_unique<raisim::World>();
  }

  virtual ~RaisimGymEnv() = default;

  /////// implement these methods ///////
  virtual void init() = 0;
  virtual void reset() = 0;
  virtual void setSeed(int seed) = 0;
  virtual void observe(Eigen::Ref<EigenVec> ob) = 0;
  virtual float step(const Eigen::Ref<EigenVec>& action) = 0;
  virtual bool isTerminalState(float& terminalReward) = 0;
  ////////////////////////////////////////

  /////// optional methods ///////
  virtual void curriculumUpdate() {};
  virtual void close() {};
  virtual void updateExtraInfo() {};
  ////////////////////////////////

  void setSimulationTimeStep(double dt) {
    simulation_dt_ = dt;
    world_->setTimeStep(dt);
  }

  virtual void startRecordingVideo(const std::string& fileName) {
    raisim::OgreVis::get()->startRecordingVideo(fileName);
  }

  virtual void stopRecordingVideo() {
    raisim::OgreVis::get()->stopRecordingVideoAndSave();
  }

  void setControlTimeStep(double dt) { control_dt_ = dt; }
  int getObDim() { return obDim_; }
  int getActionDim() { return actionDim_; }
  double getControlTimeStep() { return control_dt_; }
  double getSimulationTimeStep() { return simulation_dt_; }
  int getExtraInfoDim() { return extraInfo_.size(); }
  raisim::World* getWorld() { return world_.get(); }
  void turnOnVisualization() { visualizeThisStep_ = true; }
  void turnOffvisualization() { visualizeThisStep_ = false; }

  std::unordered_map<std::string, float> extraInfo_;

 protected:
  std::unique_ptr<raisim::World> world_;
  double simulation_dt_ = 0.001;
  double control_dt_ = 0.01;
  std::string resourceDir_;
  bool visualizeThisStep_=false;
  YAML::Node cfg_;
  int obDim_=0, actionDim_=0;
};

}

#endif //SRC_RAISIMGYMENV_HPP
