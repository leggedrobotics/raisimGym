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

/* Convention
*
*   observation space = [ height                                                      n =  1, si =  0
*                         z-axis in world frame expressed in body frame (R_b.row(2))  n =  3, si =  1
*                         joint angles,                                               n = 12, si =  4
*                         body Linear velocities,                                     n =  3, si = 16
*                         body Angular velocities,                                    n =  3, si = 19
*                         joint velocities,                                           n = 12, si = 22 ] total 34
*
*/


#include <stdlib.h>
#include <cstdint>
#include <set>
#include <raisim/OgreVis.hpp>
#include "RaisimGymEnv.hpp"
#include "visSetupCallback.hpp"

#include "visualizer/raisimKeyboardCallback.hpp"
#include "visualizer/helper.hpp"
#include "visualizer/guiState.hpp"
#include "visualizer/raisimBasicImguiPanel.hpp"


namespace raisim {

class ENVIRONMENT : public RaisimGymEnv {

 public:

  explicit ENVIRONMENT(const std::string& resourceDir, const YAML::Node& cfg, bool visualizable) :
      RaisimGymEnv(resourceDir, cfg), distribution_(0.0, 0.2), visualizable_(visualizable) {

    /// add objects
    laikago_ = world_->addArticulatedSystem(resourceDir_+"/laikago.urdf");
    laikago_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    ground_ = world_->addGround();
    world_->setERP(0,0);

    /// get robot data
    gcDim_ = laikago_->getGeneralizedCoordinateDim();
    gvDim_ = laikago_->getDOF();
    nJoints_ = 12;

    /// initialize containers
    gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);
    torque_.setZero(gvDim_);
    pTarget_.setZero(gcDim_); vTarget_.setZero(gvDim_); pTarget12_.setZero(nJoints_);

    /// this is nominal configuration of anymal
    gc_init_ << 0, 0, 0.46, 1, 0.0, 0.0, 0.0, 0.0, 0.5, -1, 0, 0.5, -1, 0.00, 0.5, -1, 0, 0.5, -0.7;

    /// set pd gains
    Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_);
    jointPgain.setZero(); jointPgain.tail(nJoints_).setConstant(40.0);
    jointDgain.setZero(); jointDgain.tail(nJoints_).setConstant(1.0);
    laikago_->setPdGains(jointPgain, jointDgain);
    laikago_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 34; /// convention described on top
    actionDim_ = nJoints_;
    actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    obMean_.setZero(obDim_); obStd_.setZero(obDim_);

    /// action & observation scaling
    actionMean_ = gc_init_.tail(nJoints_);
    actionStd_.setConstant(0.6);

    obMean_ << 0.44, /// average height
        0.0, 0.0, 0.0, /// gravity axis 3
        gc_init_.tail(12), /// joint position 12
        Eigen::VectorXd::Constant(6, 0.0), /// body lin/ang vel 6
        Eigen::VectorXd::Constant(12, 0.0); /// joint vel history

    obStd_ << 0.12, /// average height
        Eigen::VectorXd::Constant(3, 0.7), /// gravity axes angles
        Eigen::VectorXd::Constant(12, 1.0 / 1.0), /// joint angles
        Eigen::VectorXd::Constant(3, 2.0), /// linear velocity
        Eigen::VectorXd::Constant(3, 4.0), /// angular velocities
        Eigen::VectorXd::Constant(12, 10.0); /// joint velocities

    /// Reward coefficients
    READ_YAML(double, forwardVelRewardCoeff_, cfg["forwardVelRewardCoeff"])
    READ_YAML(double, torqueRewardCoeff_, cfg["torqueRewardCoeff"])

    gui::rewardLogger.init({"forwardVelReward", "torqueReward"});

    /// indices of links that should not make contact with ground
    bodyIndex_ = laikago_->getBodyIdx("trunk");

    /// visualize if it is the first environment
    if (visualizable_) {
      auto vis = raisim::OgreVis::get();

      /// these method must be called before initApp
      vis->setWorld(world_.get());
      vis->setWindowSize(1800, 1200);
      vis->setImguiSetupCallback(imguiSetupCallback);
      vis->setImguiRenderCallback(imguiRenderCallBack);
      vis->setKeyboardCallback(raisimKeyboardCallback);
      vis->setSetUpCallback(setupCallback);
      vis->setAntiAliasing(2);

      /// starts visualizer thread
      vis->initApp();

      laikagoVisual_ = vis->createGraphicalObject(laikago_, "Laikago");
      vis->createGraphicalObject(ground_, 20, "floor", "checkerboard_green");
      desired_fps_ = 50.;
      vis->setDesiredFPS(desired_fps_);
    }
  }

  ~ENVIRONMENT() final = default;

  void init() final { }

  void reset() final {
    laikago_->setState(gc_init_, gv_init_);
    updateObservation();
    if(visualizable_)
      gui::rewardLogger.clean();
  }

  float step(const Eigen::Ref<EigenVec>& action) final {
    /// action scaling
    pTarget12_ = action.cast<double>();
    pTarget12_ = pTarget12_.cwiseProduct(actionStd_);
    pTarget12_ += actionMean_;
    pTarget_.tail(nJoints_) = pTarget12_;

    laikago_->setPdTarget(pTarget_, vTarget_);
    auto loopCount = int(control_dt_ / simulation_dt_ + 1e-10);
    auto visDecimation = int(1. / (desired_fps_ * simulation_dt_) + 1e-10);

    for(int i=0; i<loopCount; i++) {
      world_->integrate();

      if (visualizable_ && visualizeThisStep_ && visualizationCounter_ % visDecimation == 0)
        raisim::OgreVis::get()->renderOneFrame();

      visualizationCounter_++;
    }

    updateObservation();

    torqueReward_ = torqueRewardCoeff_ * laikago_->getGeneralizedForce().squaredNorm();
    forwardVelReward_ = forwardVelRewardCoeff_ * bodyLinearVel_[0];

    if(visualizeThisStep_) {
      gui::rewardLogger.log("torqueReward", torqueReward_);
      gui::rewardLogger.log("forwardVelReward", forwardVelReward_);

      /// set camera
      auto vis = raisim::OgreVis::get();
      vis->select(laikagoVisual_->at(0), false);
      vis->getCameraMan()->setYawPitchDist(Ogre::Radian(0), Ogre::Radian(-1.0), 3);
    }

    return torqueReward_ + forwardVelReward_;
  }

  void updateExtraInfo() final {
    extraInfo_["forward vel reward"] = forwardVelReward_;
    extraInfo_["base height"] = gc_[2];
  }

  void updateObservation() {
    laikago_->getState(gc_, gv_);
    obDouble_.setZero(obDim_); obScaled_.setZero(obDim_);

    /// body height
    obDouble_[0] = gc_[2];

    /// body orientation
    raisim::Vec<4> quat;
    raisim::Mat<3,3> rot;
    quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
    raisim::quatToRotMat(quat, rot);
    obDouble_.segment(1, 3) = rot.e().row(2);

    /// joint angles
    obDouble_.segment(4, 12) = gc_.tail(12);

    /// body velocities
    bodyLinearVel_ = rot.e().transpose() * gv_.segment(0, 3);
    bodyAngularVel_ = rot.e().transpose() * gv_.segment(3, 3);
    obDouble_.segment(16, 3) = bodyLinearVel_;
    obDouble_.segment(19, 3) = bodyAngularVel_;

    /// joint velocities
    obDouble_.tail(12) = gv_.tail(12);
    obScaled_ = (obDouble_-obMean_).cwiseQuotient(obStd_);
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    /// convert it to float
    ob = obScaled_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);

    /// if the contact body is not feet
    for(auto& contact: laikago_->getContacts())
      if(contact.getPairObjectIndex() == ground_->getIndexInWorld() &&
         contact.getlocalBodyIndex() == bodyIndex_) {
        return true;
      }

    terminalReward = 0.f;
    return false;
  }

  void setSeed(int seed) final {
    std::srand(seed);
  }

  void close() final {
  }

 private:
  int gcDim_, gvDim_, nJoints_;
  bool visualizable_ = false;
  std::normal_distribution<double> distribution_;
  raisim::ArticulatedSystem* laikago_;
  std::vector<GraphicObject> * laikagoVisual_;
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, pTarget_, pTarget12_, vTarget_, torque_;
  double terminalRewardCoeff_ = -10.;
  double forwardVelRewardCoeff_ = 0., forwardVelReward_ = 0.;
  double torqueRewardCoeff_ = 0., torqueReward_ = 0.;
  double desired_fps_ = 60.;
  int visualizationCounter_=0;
  Eigen::VectorXd actionMean_, actionStd_, obMean_, obStd_;
  Eigen::VectorXd obDouble_, obScaled_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_;
  size_t bodyIndex_;
  raisim::Ground* ground_;
};

}

