#include <mc_control/MCController.h>
#include <mc_observers/ObserverMacros.h>
#include <iostream>
#include <mc_state_observation/TiltObserver.h>
#include <mc_state_observation/gui_helpers.h>
#include <mc_state_observation/kinematicsTools.h>

namespace mc_state_observation
{

namespace so = stateObservation;

TiltObserver::TiltObserver(const std::string & type, double dt)
: mc_observers::Observer(type, dt), estimator_(alpha_, beta_, gamma_)
{
  estimator_.setSamplingTime(dt_);
  xk_.resize(9);
  xk_ << so::Vector3::Zero(), so::Vector3::Zero(), so::Vector3(0, 0, 1); // so::Vector3(0.49198, 0.66976, 0.55622);
  estimator_.setState(xk_, 0);
}

void TiltObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  updateRobotName_ = robot_;
  imuSensor_ = config("imuSensor", ctl.robot().bodySensor().name());
  updateSensorName_ = imuSensor_;
  config("alpha", alpha_);
  config("beta", beta_);
  config("gamma", gamma_);
  // anchorFrameFunction_ = config("anchorFrameFunction", name() + "::" + ctl.robot(robot_).name());
  anchorFrameFunction_ = "KinematicAnchorFrame::" + ctl.robot(robot_).name();
  config("maxAnchorFrameDiscontinuity", maxAnchorFrameDiscontinuity_);
  config("updateRobot", updateRobot_);
  config("updateRobotName", updateRobotName_);
  config("updateSensor", updateSensor_);
  config("updateSensorName", updateSensorName_);
  desc_ = fmt::format("{}", name_);
}

void TiltObserver::reset(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);

  my_robots_ = mc_rbdyn::Robots::make();
  my_robots_->robotCopy(robot, robot.name());
  my_robots_->robotCopy(realRobot, "inputRobot");
  ctl.gui()->addElement({"Robots"}, mc_rtc::gui::Robot("TiltEstimator", [this]() -> const mc_rbdyn::Robot & {
                          return my_robots_->robot();
                        }));
  poseW_ = ctl.realRobot(robot_).posW();
  velW_ = ctl.realRobot(robot_).velW();
}

bool TiltObserver::run(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);

  estimator_.setAlpha(alpha_);
  estimator_.setBeta(beta_);
  estimator_.setGamma(gamma_);

  
  if(!ctl.datastore().has(anchorFrameFunction_))
  {
    error_ = fmt::format(
        "Observer {} requires a \"{}\" function in the datastore to provide the observer's kinematic anchor frame.\n"
        "Please refer to https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/observers.html for further details.",
        name(), anchorFrameFunction_);
    return false;
  }

  anchorFrameJumped_ = false;
  
  double leftFootRatio = robot.indirectSurfaceForceSensor("LeftFootCenter").force().z()
                         / (robot.indirectSurfaceForceSensor("LeftFootCenter").force().z()
                            + robot.indirectSurfaceForceSensor("RightFootCenter").force().z());
  /* If the anchor frame has problems, one can use this   */
  X_0_C_ =
      sva::interpolate(robot.surfacePose("RightFootCenter"), robot.surfacePose("LeftFootCenter"), leftFootRatio);
  X_0_C_real_ =
      sva::interpolate(realRobot.surfacePose("RightFootCenter"), realRobot.surfacePose("LeftFootCenter"), leftFootRatio);

  //X_0_C_ = ctl.datastore().call<sva::PTransformd>(anchorFrameFunction_, ctl.robot(robot_));
  //X_0_C_real_ = ctl.datastore().call<sva::PTransformd>(anchorFrameFunction_, ctl.realRobot(robot_));

  if(firstIter_)
  { // Ignore anchor frame check on first iteration
    firstIter_ = false;
  }
  else
  { // Check whether anchor frame jumped
    auto error = (X_0_C_.translation() - worldAnchorKine.position()).norm();
    if(error > maxAnchorFrameDiscontinuity_)
    {
      mc_rtc::log::warning("[{}] Control anchor frame jumped from [{}] to [{}] (error norm {} > threshold {})", name(),
                           MC_FMT_STREAMED(worldAnchorKine.position().transpose()),
                           MC_FMT_STREAMED(X_0_C_.translation().transpose()), error, maxAnchorFrameDiscontinuity_);
      anchorFrameJumped_ = true;
    }
    auto errorReal = (X_0_C_.translation() - worldAnchorKine.position()).norm();
    if(errorReal > maxAnchorFrameDiscontinuity_)
    {
      mc_rtc::log::warning("[{}] Real anchor frame jumped from [{}] to [{}] (error norm {:.3f} > threshold {:.3f})",
                           name(), MC_FMT_STREAMED(X_0_C_real_previous_.translation().transpose()),
                           MC_FMT_STREAMED(X_0_C_real_.translation().transpose()), errorReal,
                           maxAnchorFrameDiscontinuity_);
      anchorFrameJumped_ = true;
    }
  }

  X_0_C_real_previous_ = X_0_C_real_;

  // Anchor frame defined w.r.t control robot
  // XXX what if the feet are being moved by the stabilizer?


  // we want in the anchor frame:
  // - position of the IMU
  // - orientation of the IMU
  // - linear velocity of the imu
  // - angular velocity of the imu
  // - linear velocity of the anchor frame in the world of the control robot (derivative?)

  const auto & imu = realRobot.bodySensor(imuSensor_);
  auto X_0_FB = realRobot.posW();

  so::kine::Kinematics worldFBKine =
      kinematicsTools::poseFromSva(X_0_FB, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vels);

  const sva::PTransformd & imuXbs = imu.X_b_s();
  so::kine::Kinematics bodyImuKine =
      kinematicsTools::poseFromSva(imuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vels);

  const sva::PTransformd & bodyW = robot.bodyPosW(imu.parentBody());

  // auto RF = anchorFrame.oriention().transpose(); // orientation of the control anchor frame
  // auto RB = robot.posW().orientation().transpose(); // orientation of the control floating base

  // Compute velocity of the imu in the control frame
  auto v_0_imuParent = robot.mbc().bodyVelW[robot.bodyIndexByName(imu.parentBody())];
  so::kine::Kinematics worldBodyKine = kinematicsTools::poseAndVelFromSva(bodyW, v_0_imuParent, true);

  so::kine::Kinematics worldImuKine = worldBodyKine * bodyImuKine;
  /* If the anchor frame has problems, one can use this   
  double leftFootRatio = robot.indirectSurfaceForceSensor("LeftFootCenter").force().z()
                         / (robot.indirectSurfaceForceSensor("LeftFootCenter").force().z()
                            + robot.indirectSurfaceForceSensor("RightFootCenter").force().z());

  newWorldAnchorPose =
      sva::interpolate(robot.surfacePose("RightFootCenter"), robot.surfacePose("LeftFootCenter"), leftFootRatio);
  */
  so::kine::Kinematics newWorldAnchorKine =
      kinematicsTools::poseFromSva(X_0_C_, so::kine::Kinematics::Flags::pose);
  worldAnchorKine.update(newWorldAnchorKine, ctl.timeStep, flagPoseVels_);

  // Pose of the imu in the control frame
  so::kine::Kinematics anchorImuKine = worldAnchorKine.getInverse() * worldImuKine;

  fbImuKine_ = worldFBKine.getInverse() * worldImuKine;

  BOOST_ASSERT((anchorImuKine.linVel.isSet() || anchorImuKine.angVel.isSet()) && "vels were not computed");

  estimator_.setSensorPositionInC(anchorImuKine.position());
  estimator_.setSensorOrientationInC(anchorImuKine.orientation.toMatrix3());
  estimator_.setSensorLinearVelocityInC(anchorImuKine.linVel());
  estimator_.setSensorAngularVelocityInC(anchorImuKine.angVel());
  estimator_.setControlOriginVelocityInW(worldAnchorKine.orientation.toMatrix3().transpose()
                                         * worldAnchorKine.linVel());

  auto k = estimator_.getCurrentTime();

  estimator_.setMeasurement(imu.linearAcceleration(), imu.angularVelocity(), k + 1);

  xk_ = estimator_.getEstimatedState(k + 1);

  so::Vector3 tilt = xk_.tail(3);

  // Orientation of the imu in the world obtained from the estimated tilt and the yaw of the control robot.
  estimatedRotationIMU_ = so::kine::mergeTiltWithYaw(tilt, worldImuKine.orientation.toMatrix3());

  // Estimated orientation of the floating base in the world
  R_0_fb_ = estimatedRotationIMU_ * fbImuKine_.orientation.toMatrix3();
  updatePoseAndVel(ctl, xk_.head(3), imu.angularVelocity());

  // update the velocities as MotionVecd for the logs
  imuVelC_.linear() = anchorImuKine.linVel();
  imuVelC_.angular() = anchorImuKine.angVel();

  // update the pose as PTransformd for the logs
  X_C_IMU_.translation() = anchorImuKine.position();
  X_C_IMU_.rotation() = anchorImuKine.orientation.toMatrix3().transpose();

  /* Update of the observed robot */
  my_robots_->robot().mbc().q = ctl.realRobot().mbc().q;
  update(my_robots_->robot(), ctl);

  return true;
}

void TiltObserver::updatePoseAndVel(const mc_control::MCController & ctl, const so::Vector3 & localWorldImuLinVel, const so::Vector3 & localWorldImuAngVel)
{
  poseW_.rotation() = R_0_fb_.transpose();
  // Relative transformation between the real anchor frame and the floating base
  // Uses the FK with the real robot encoder measurements
  const sva::PTransformd X_real_s = X_0_C_real_ * ctl.realRobot(robot_).posW().inv();
  // Position of the control anchor frame (world)
  const Eigen::Vector3d & r_c_0 = X_0_C_.translation();
  // Position of the real anchor frame (world)
  const Eigen::Vector3d & r_s_real = X_real_s.translation();
  // The floating base position is estimated by applying the kinematics
  // transformation between the real robot's anchor frame and its floating base
  // (kinematics only), rotated by the estimated rotation of the real floating
  // base from the IMU (see estimateOrientation)
  poseW_.translation() = r_c_0 - R_0_fb_ * r_s_real;

  
  
  if(!anchorFrameJumped_)
  {
    sva::MotionVecd errVel = sva::transformError(prevPoseW_, poseW_) / ctl.timeStep;

    so::kine::Kinematics correctedWorldFbKine;
    correctedWorldFbKine.position = poseW_.translation();
    correctedWorldFbKine.orientation = R_0_fb_;

    so::kine::Kinematics correctedWorldImuKine = correctedWorldFbKine * fbImuKine_; // corrected pose of the imu in the world. This step is used only to get the pose of the IMU in the world that is required for the kinematics composition. 
    correctedWorldImuKine.linVel = correctedWorldImuKine.orientation * localWorldImuLinVel;
    correctedWorldImuKine.angVel = correctedWorldImuKine.orientation * localWorldImuAngVel;
    correctedWorldFbKine = correctedWorldImuKine * fbImuKine_.getInverse();

    velW_.linear() = correctedWorldFbKine.linVel();
    velW_.angular() = correctedWorldFbKine.angVel();
  }
  else { mc_rtc::log::warning("[{}] Skipping velocity update (anchor frame jumped)", name()); }

  prevPoseW_ = poseW_;
}

void TiltObserver::update(mc_control::MCController & ctl)
{
  if(updateRobot_)
  {
    auto & realRobot = ctl.realRobots().robot(updateRobot_);    
    mc_rtc::log::info("update robot: {}", mc_rbdyn::rpyFromMat(poseW_.rotation()));
    realRobot.posW(poseW_);
    realRobot.velW(velW_);
    realRobot.forwardKinematics();
  }

  if(updateSensor_)
  {
    // auto & imu = ctl.robot(robot_).bodySensor(imuSensor_);
    auto & imu = const_cast<mc_rbdyn::BodySensor &>(ctl.robot(robot_).bodySensor(imuSensor_));
    auto & rimu = const_cast<mc_rbdyn::BodySensor &>(ctl.realRobot(robot_).bodySensor(imuSensor_));

    imu.orientation(Eigen::Quaterniond{estimatedRotationIMU_});
    rimu.orientation(Eigen::Quaterniond{estimatedRotationIMU_});
  }
}

void TiltObserver::update(mc_rbdyn::Robot & robot, const mc_control::MCController & ctl)
{
  robot.posW(poseW_);
}

void TiltObserver::addToLogger(const mc_control::MCController &, mc_rtc::Logger & logger, const std::string & category)
{
  logger.addLogEntry(category + "_controlAnchorFrame",
                     [this]() -> const sva::PTransformd & { return X_0_C_; });

  logger.addLogEntry(category + "_IMU_world_orientation", [this]() { return Eigen::Quaterniond{estimatedRotationIMU_}; });
  logger.addLogEntry(category + "_IMU_AnchorFrame_pose", [this]()  -> const sva::PTransformd & { return X_C_IMU_; });
  logger.addLogEntry(category + "_IMU_AnchorFrame_linVel", [this]() -> const sva::MotionVecd & { return imuVelC_; });
  logger.addLogEntry(category + "_AnchorFrame_world_linVel_global", [this]() -> so::Vector3 { return worldAnchorKine.linVel(); });
  logger.addLogEntry(category + "_AnchorFrame_world_linVel_local", [this]() -> so::Vector3 {
    return worldAnchorKine.orientation.toMatrix3().transpose() * worldAnchorKine.linVel(); });
  logger.addLogEntry(category + "_AnchorFrame_world_angVel", [this]() -> const so::Vector3 & { return worldAnchorKine.angVel(); });
  logger.addLogEntry(category + "_FloatingBase_world_pose", [this]() -> const sva::PTransformd & { return poseW_; });
  logger.addLogEntry(category + "_FloatingBase_world_vel", [this]() -> const sva::MotionVecd & { return velW_; });
  
}

void TiltObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_imuVelC");
  logger.removeLogEntry(category + "_imuPoseC");
  logger.removeLogEntry(category + "_imuEstRotW");
  logger.removeLogEntry(category + "_controlAnchorFrame");
}

void TiltObserver::addToGUI(const mc_control::MCController & ctl,
                            mc_rtc::gui::StateBuilder & gui,
                            const std::vector<std::string> & category)
{
  using namespace mc_state_observation::gui;
  gui.addElement(category, make_input_element("alpha", alpha_), make_input_element("beta", beta_),
                 make_input_element("gamma", gamma_));
}

} // namespace mc_state_observation
EXPORT_OBSERVER_MODULE("Tilt", mc_state_observation::TiltObserver)
