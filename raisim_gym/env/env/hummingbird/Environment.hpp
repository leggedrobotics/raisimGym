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
*   observation space = [ rotation matrix,                                            n = 9, si = 0
*                         robot position,                                             n = 3, si = 9
*                         body Linear velocities,                                     n = 3, si = 12
*                         body Angular velocities,                                    n = 3, si = 15] total 18
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
      RaisimGymEnv(resourceDir, cfg),
      normDist_(0., 1.),
      visualizable_(visualizable),
      gen_(rd_()) {

    /// add objects
    hummingbird_ = world_->addBox(0.35, 0.35, 0.08, 0.6);
    auto ground = world_->addGround(-5);

    /// get robot data
    nRotors_ = 4;

    /// initialize containers
    thrusts_.setZero(nRotors_);

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 18; /// convention described on top
    actionDim_ = nRotors_;
    actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    obMean_.setZero(obDim_); obStd_.setZero(obDim_);
    obDouble_.setZero(obDim_); obScaled_.setZero(obDim_);

    /// action & observation scaling
    actionMean_.setConstant(2.5);
    actionStd_.setConstant(2.0);

    obMean_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, /// rotation matrix
        0.0, 0.0, 0.0, /// target position
        Eigen::VectorXd::Constant(6, 0.0); /// body lin/ang vel 6
    obStd_ << Eigen::VectorXd::Constant(9, 0.7), /// rotation matrix
        Eigen::VectorXd::Constant(3, 0.5), /// target position
        Eigen::VectorXd::Constant(3, 3.0), /// linear velocity
        Eigen::VectorXd::Constant(3, 8.0); /// angular velocities

    /// Reward coefficients
    READ_YAML(double, positionRewardCoeff_, cfg["positionRewardCoeff"])
    READ_YAML(double, thrustRewardCoeff_, cfg["thrustRewardCoeff"])
    READ_YAML(double, orientationRewardCoeff_, cfg["orientationRewardCoeff"])
    READ_YAML(double, angVelRewardCoeff_, cfg["angVelRewardCoeff"])

    Eigen::Matrix3d inertia;
    inertia << 0.007, 0.0, 0.0, 0.0, 0.007, 0.0, 0.0, 0.0, 0.012;
    hummingbird_->setInertia(inertia);
    hummingbird_->setAngularDamping({0.003, 0.003, 0.003});
    hummingbird_->setLinearDamping(0.01);

    transsThrust2GenForce_ <<          0,          0,     length_,    -length_,
                                -length_,    length_,           0,           0,
                              dragCoeff_, dragCoeff_, -dragCoeff_, -dragCoeff_,
                                       1,          1,           1,           1;

    gui::rewardLogger.init({"positionReward", "thrustReward", "orientationReward", "angVelReward"});

    /// visualize if it is the first environment
    if (visualizable_) {
      auto vis = raisim::OgreVis::get();

      /// these method must be called before initApp
      vis->setWorld(world_.get());
      vis->setWindowSize(1280, 720);
      vis->setImguiSetupCallback(imguiSetupCallback);
      vis->setImguiRenderCallback(imguiRenderCallBack);
      vis->setKeyboardCallback(raisimKeyboardCallback);
      vis->setSetUpCallback(setupCallback);
      vis->setAntiAliasing(2);

      /// starts visualizer thread
      vis->initApp();

      fullFilePath_ = resourceDir + "/mesh.dae";
      vis->loadMeshFile(fullFilePath_, fullFilePath_);
      raisim::Mat<3,3> identity; identity.setIdentity();

      hummingbirdVis_ = vis->registerSet("quad",
                                         hummingbird_,
                                         {vis->createSingleGraphicalObject(fullFilePath_,
                                                                           fullFilePath_,
                                                                           "default",
                                                                           {1.,1.,1.},
                                                                           {0.,0.,0.},
                                                                           identity,
                                                                           0)});
      vis->createGraphicalObject(ground, 20, "floor", "checkerboard_green");

      /// add a goal point visualization
      vis->addVisualObject("goal",
                           "sphereMesh",
                           "red",
                           {0.025, 0.025, 0.025},
                           false,
                           OgreVis::RAISIM_OBJECT_GROUP);
      auto& list = raisim::OgreVis::get()->getVisualObjectList();
      list["goal"].offset = {0,0,0};

      desired_fps_ = 60.;
      vis->setDesiredFPS(desired_fps_);
    }
  }

  ~ENVIRONMENT() final = default;

  void init() final { }

  void reset() final {

    float dummy;
    do{
      position_[0] = 1.0 * normDist_(gen_);
      position_[1] = 1.0 * normDist_(gen_);
      position_[2] = 1.0 * normDist_(gen_);

      linVel_W_[0] = 2.0 * normDist_(gen_);
      linVel_W_[1] = 2.0 * normDist_(gen_);
      linVel_W_[2] = 2.0 * normDist_(gen_);

      angVel_W_[0] = 5.0 * normDist_(gen_);
      angVel_W_[1] = 5.0 * normDist_(gen_);
      angVel_W_[2] = 5.0 * normDist_(gen_);
    } while( isTerminalState(dummy) );

    // sampling random orientation
    quaternion_[0] = normDist_(gen_);
    quaternion_[1] = normDist_(gen_);
    quaternion_[2] = normDist_(gen_);
    quaternion_[3] = normDist_(gen_);
    quaternion_ /= quaternion_.norm();

    hummingbird_->setPosition(position_);
    hummingbird_->setOrientation(quaternion_);
    hummingbird_->setVelocity(linVel_W_, angVel_W_);

    updateObservation();

    if(visualizable_)
      gui::rewardLogger.clean();
  }

  float step(const Eigen::Ref<EigenVec>& action) final {

    /// This part should include a Control Input from PD stabilisation
    
    ///obScaled gives 18 dimensions first 9 is the rotational matrix, 
    ///second triplet is position, third triplet is linear velocity 
    ///fourth triplet is the angular velocity 
    /// Calculate the q0 to obtain simple rotational angle
    /// Get the rotational matrix
    hummingbird_->getRotationMatrix(rot_);

    /// Convert rot_ which is a 3x3 matrix into a quaternion 

    Eigen::Quaternionf quatFromRot(rot_)
    Eigen::Vector3d Orientation
    /// quatFromRot basically houses the quaternion in w + xi +yj + zk form
     
    Orientation = quatFromRot.toRotationMatrix().eulerAngles(0, 1, 2);
    
    /// Define a Vector 
    
    double rot_angle = 2.0 * std::acos(quatFromRot.w());  //Angle of rotation obtain q0 is quatFromRot.w()
    
    hummingbird_->getAngularVelocity(angVel_W_);

    angVel_B_ = rot_.transpose()*angVel_W_;

    /// action scaling
    thrusts_ = action.cast<double>();
    thrusts_ = thrusts_.cwiseProduct(actionStd_);
    thrusts_ += actionMean_;

    Eigen::Vector4d genForce = transsThrust2GenForce_ * thrusts_;
    Eigen::Vector3d torque_B = genForce.segment(0,3);
    Eigen::Vector3d force_B;
    force_B << 0.0, 0.0, genForce(3);

    double kp_rot = -0.2, kd_rot = -0.06;
    Eigen::Vector3d fbtorque_B 

    if (rot_angle > 1e-6){
        fbtorque_B = kp_rot * rot_angle * (rot_.transpose() * Orientation) 
            / sin(rot_angle) + kd_rot * angVel_B_;
    }
    else
    {
      fbtorque_B = kd_rot * angVel_B_;
      fbtorque_B(2) = fbtorque_B(2) * 0.15; 
    }

    torque_B += fbtorque_B;  /// sum of torque inputs

    // clip inputs
    genForce << torque_B, force_B(2);
    thrust_ = transsThrust2GenForce_.inverse() * genForce;
    thrust_ = thrust_.array().cwiseMax(1e-8);   // clip for min value
    genForce = transsThrust2GenForce_ * thrust_;
    torque_B = genForce.segment(0,3);
    force_B << 0.0, 0.0, genForce(3);

////////////////////////
    raisim::Vec<3> torque_W, force_W;
    torque_W.e() = rot_.e() * torque_B;
    force_W.e() = rot_.e() * force_B;

    auto loopCount = int(control_dt_ / simulation_dt_ + 1e-10);
    auto visDecimation = int(1. / (desired_fps_ * simulation_dt_) + 1e-10);

    for(int i=0; i<loopCount; i++) {
      /// in raisim, external force/torque is reset to zero after integrate(). So this has to be called inside the for loop
      hummingbird_->setExternalForce(0, force_W);
      hummingbird_->setExternalTorque(0, torque_W);

      world_->integrate();

      if (visualizable_ && visualizeThisStep_ && visualizationCounter_ % visDecimation == 0)
        raisim::OgreVis::get()->renderOneFrame();

      visualizationCounter_++;
    }

    updateObservation();

    positionReward_ = positionRewardCoeff_ * std::sqrt(position_.norm());
    thrustReward_ = thrustRewardCoeff_ * thrusts_.squaredNorm();
    orientationReward_ = orientationRewardCoeff_ * std::acos(rot_[8]); // acos(rot_[8]) is the angle between z axis of the robot frame and that of the world frame
    angVelReward_ = angVelRewardCoeff_ * angVel_W_.norm();

    if(visualizeThisStep_) {
      gui::rewardLogger.log("positionReward", positionReward_);
      gui::rewardLogger.log("thrustReward", thrustReward_);
      gui::rewardLogger.log("orientationReward", orientationReward_);
      gui::rewardLogger.log("angVelReward", angVelReward_);

      /// reset camera
      auto vis = raisim::OgreVis::get();

      vis->select(hummingbirdVis_->at(0), false);
      vis->getCameraMan()->setYawPitchDist(Ogre::Radian(3.14), Ogre::Radian(-1.3), 3, true);
    }

    return positionReward_ + thrustReward_ + orientationReward_;
  }

  void updateExtraInfo() final {
  }

  void updateObservation() {

    hummingbird_->getPosition(position_);
    hummingbird_->getRotationMatrix(rot_);
    hummingbird_->getLinearVelocity(linVel_W_);
    hummingbird_->getAngularVelocity(angVel_W_);

    /// this orientation ignores yaw. Yaw is irrelevant for this task since the target position is provided
    for(size_t i=0; i<9; i++)
      obDouble_[i] = rot_[i];

    /// target position. the target position is always 0,0,0 in the world frame but not in the hummingbird frame
    for(size_t i=0; i<3; i++)
      obDouble_[i+9] = position_[i];

    /// body velocities
    for(size_t i=0; i<3; i++)
      obDouble_[i+12] = linVel_W_[i];

    for(size_t i=0; i<3; i++)
      obDouble_[i+15] = angVel_W_[i];

    /// scaling
    obScaled_ = (obDouble_-obMean_).cwiseQuotient(obStd_);
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    /// convert it to float
    ob = obScaled_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
//    terminalReward = float(terminalRewardCoeff_);
//
//    /// if the quad is out of the box constraint
//    for(int i=0; i<3; i++) /// for x, y, z axes
//      if(position_[i]>5. or position_[i]<-5.)
//        return true;
//
//    /// too high angular velocity is undesirable
//    if(angVel_W_.norm() > 50.)
//      return true;

    terminalReward = 0.f;
    return false;
  }

  void setSeed(int seed) final {
    std::srand(seed);
  }

  void close() final {
  }

 private:
  int nRotors_;
  bool visualizable_ = false;
  std::normal_distribution<double> distribution_;
  raisim::SingleBodyObject* hummingbird_;
  Eigen::VectorXd thrusts_;
  double terminalRewardCoeff_ = -2.;
  double positionRewardCoeff_ = 0., positionReward_ = 0.;
  double thrustRewardCoeff_ = 0., thrustReward_ = 0.;
  double orientationRewardCoeff_ = 0., orientationReward_ = 0.;
  double angVelRewardCoeff_ = 0., angVelReward_ = 0.;

  double desired_fps_ = 60.;
  int visualizationCounter_=0;
  std::set<size_t> footIndices_;
  std::string fullFilePath_;
  std::vector<GraphicObject> *hummingbirdVis_;
  raisim::Mat<3,3> rot_;
  raisim::Vec<4> quaternion_;
  raisim::Vec<3> position_;
  raisim::Vec<3> linVel_W_, angVel_W_;

  Eigen::VectorXd actionMean_, actionStd_, obMean_, obStd_;
  Eigen::VectorXd obDouble_, obScaled_;

  Eigen::Matrix4d transsThrust2GenForce_;

  /// quadrotor model parameters
  double length_ = 0.17, dragCoeff_= 0.016;

  std::normal_distribution<double> normDist_;
  std::random_device rd_;
  std::mt19937 gen_;
};

}

