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
  ctl.gui()->addElement(
      {"Robots"},
      mc_rtc::gui::Robot("TiltEstimator", [this]() -> const mc_rbdyn::Robot & { return my_robots_->robot(); }));
  poseW_ = ctl.realRobot(robot_).posW();
  velW_ = ctl.realRobot(robot_).velW();

  oldRealRobotWorldAnchorKine_ =
      kinematicsTools::poseFromSva(ctl.datastore().call<sva::PTransformd>(anchorFrameFunction_, ctl.realRobot(robot_)),
                                   so::kine::Kinematics::Flags::pose);
  oldRealRobotBasedAnchorKine_ =
      kinematicsTools::poseFromSva(ctl.datastore().call<sva::PTransformd>(anchorFrameFunction_, ctl.realRobot(robot_))
                                       * ctl.realRobot(robot_).posW().inv(),
                                   so::kine::Kinematics::Flags::pose);
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

  /* If the anchor frame has problems, one can use this
  double leftFootRatio = robot.indirectSurfaceForceSensor("LeftFootCenter").force().z()
                         / (robot.indirectSurfaceForceSensor("LeftFootCenter").force().z()
                            + robot.indirectSurfaceForceSensor("RightFootCenter").force().z());

  X_0_C_ = sva::interpolate(robot.surfacePose("RightFootCenter"), robot.surfacePose("LeftFootCenter"), leftFootRatio);
  X_0_C_real_ = sva::interpolate(realRobot.surfacePose("RightFootCenter"), realRobot.surfacePose("LeftFootCenter"),
  leftFootRatio);
  */

  X_0_C_ = ctl.datastore().call<sva::PTransformd>(anchorFrameFunction_, ctl.robot(robot_));
  X_0_C_real_ = ctl.datastore().call<sva::PTransformd>(anchorFrameFunction_, ctl.realRobot(robot_));

  if(firstIter_)
  { // Ignore anchor frame check on first iteration
    firstIter_ = false;
  }
  else
  { // Check whether anchor frame jumped
    auto error = (X_0_C_.translation() - worldAnchorKine_.position()).norm();
    if(error > maxAnchorFrameDiscontinuity_)
    {
      mc_rtc::log::warning("[{}] Control anchor frame jumped from [{}] to [{}] (error norm {} > threshold {})", name(),
                           MC_FMT_STREAMED(worldAnchorKine_.position().transpose()),
                           MC_FMT_STREAMED(X_0_C_.translation().transpose()), error, maxAnchorFrameDiscontinuity_);
      anchorFrameJumped_ = true;
    }
    auto errorReal = (X_0_C_.translation() - worldAnchorKine_.position()).norm();
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

  const auto & imu = robot.bodySensor(imuSensor_);
  const auto & rimu = realRobot.bodySensor(imuSensor_);

  auto X_0_FB = robot.posW();
  auto v_0_FB = robot.velW();
  auto acc_0_FB = robot.accW();
  worldFbKine_ = kinematicsTools::kinematicsFromSva(X_0_FB, v_0_FB, acc_0_FB, true, true);

  auto X_0_FB_real = realRobot.posW();
  auto v_0_FB_real = realRobot.velW();
  auto acc_0_FB_real = realRobot.accW();
  realWorldFbKine_ = kinematicsTools::kinematicsFromSva(X_0_FB_real, v_0_FB_real, acc_0_FB_real, true, true);

  const sva::PTransformd & imuXbs = imu.X_b_s();
  const sva::PTransformd & rimuXbs = rimu.X_b_s();

  so::kine::Kinematics parentImuKine =
      kinematicsTools::poseFromSva(imuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vels);
  so::kine::Kinematics realParentImuKine =
      kinematicsTools::poseFromSva(rimuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vels);

  const sva::PTransformd & parentPoseW = robot.bodyPosW(imu.parentBody());
  const sva::PTransformd & realParentPoseW = realRobot.bodyPosW(rimu.parentBody());

  // auto RF = anchorFrame.oriention().transpose(); // orientation of the control anchor frame
  // auto RB = robot.posW().orientation().transpose(); // orientation of the control floating base

  // Compute velocity of the imu in the control frame
  auto v_0_imuParent = robot.mbc().bodyVelW[robot.bodyIndexByName(imu.parentBody())];
  auto real_v_0_imuParent = realRobot.mbc().bodyVelW[realRobot.bodyIndexByName(imu.parentBody())];

  so::kine::Kinematics worldParentKine = kinematicsTools::poseAndVelFromSva(parentPoseW, v_0_imuParent, true);
  so::kine::Kinematics realWorldParentKine =
      kinematicsTools::poseAndVelFromSva(realParentPoseW, real_v_0_imuParent, true);

  so::kine::Kinematics worldImuKine = worldParentKine * parentImuKine;
  so::kine::Kinematics realWorldImuKine = realWorldParentKine * realParentImuKine;

  /* If the anchor frame has problems, one can use this
  double leftFootRatio = robot.indirectSurfaceForceSensor("LeftFootCenter").force().z()
                         / (robot.indirectSurfaceForceSensor("LeftFootCenter").force().z()
                            + robot.indirectSurfaceForceSensor("RightFootCenter").force().z());

  newWorldAnchorPose =
      sva::interpolate(robot.surfacePose("RightFootCenter"), robot.surfacePose("LeftFootCenter"), leftFootRatio);
  */
  so::kine::Kinematics newWorldAnchorKine = kinematicsTools::poseFromSva(X_0_C_, so::kine::Kinematics::Flags::pose);
  worldAnchorKine_.update(newWorldAnchorKine, ctl.timeStep, flagPoseVels_);

  realWorldAnchorKine_.update(newWorldAnchorKine, ctl.timeStep, flagPoseVels_);

  // Pose of the imu in the control frame
  so::kine::Kinematics anchorImuKine = worldAnchorKine_.getInverse() * worldImuKine;

  fbImuKine_ = realWorldFbKine_.getInverse() * realWorldImuKine;

  std::cout << std::endl
            << "anchorIMUPositionTest: " << std::endl
            << worldAnchorKine_.orientation.toMatrix3().transpose()
                   * (worldImuKine.position() - worldAnchorKine_.position())
            << std::endl;

  std::cout << std::endl
            << "anchorIMUVelTest: " << std::endl
            << -(worldAnchorKine_.orientation.toMatrix3().transpose() * worldAnchorKine_.angVel())
                       .cross(worldAnchorKine_.orientation.toMatrix3().transpose()
                              * (worldImuKine.position() - worldAnchorKine_.position()))
                   + worldAnchorKine_.orientation.toMatrix3().transpose()
                         * (worldImuKine.linVel() - worldAnchorKine_.linVel())
            << std::endl;

  std::cout << std::endl << "worldAnchorKine_: " << std::endl << worldAnchorKine_ << std::endl;
  std::cout << std::endl << "worldImuKine: " << std::endl << worldImuKine << std::endl;

  BOOST_ASSERT((anchorImuKine.linVel.isSet() || anchorImuKine.angVel.isSet()) && "vels were not computed");

  std::cout << std::endl << "anchorImuKine: " << std::endl << anchorImuKine << std::endl;
  std::cout << std::endl
            << "ControlOriginVelocityInW: " << std::endl
            << worldAnchorKine_.orientation.toMatrix3().transpose() * worldAnchorKine_.linVel() << std::endl;

  estimator_.setSensorPositionInC(anchorImuKine.position());
  estimator_.setSensorOrientationInC(anchorImuKine.orientation.toMatrix3());
  estimator_.setSensorLinearVelocityInC(anchorImuKine.linVel());
  estimator_.setSensorAngularVelocityInC(anchorImuKine.angVel());
  estimator_.setControlOriginVelocityInW(worldAnchorKine_.orientation.toMatrix3().transpose()
                                         * worldAnchorKine_.linVel());

  auto k = estimator_.getCurrentTime();

  x1_ = worldImuKine.orientation.toMatrix3().transpose()
            * (-worldImuKine.angVel().cross(worldAnchorKine_.position()) + worldAnchorKine_.linVel())
        + anchorImuKine.orientation.toMatrix3().transpose() * anchorImuKine.linVel();

  std::cout << std::endl << "x1_: " << std::endl << x1_ << std::endl;

  estimator_.setMeasurement(imu.linearAcceleration(), imu.angularVelocity(), k + 1);
  estimator_.setExplicitX1(x1_); // we directly give the virtual measurement of the velocity by the IMU

  xk_ = estimator_.getEstimatedState(k + 1);

  so::Vector3 tilt = xk_.tail(3);

  // Orientation of the imu in the world obtained from the estimated tilt and the yaw of the control robot.
  estimatedRotationIMU_ = so::kine::mergeTiltWithYaw(tilt, worldImuKine.orientation.toMatrix3());

  // Estimated orientation of the floating base in the world
  R_0_fb_ = estimatedRotationIMU_ * fbImuKine_.orientation.toMatrix3();
  updatePoseAndVel(ctl, estimator_.getVirtualLocalVelocityMeasurement(), imu.angularVelocity());

  // update the velocities as MotionVecd for the logs
  imuVelC_.linear() = anchorImuKine.linVel();
  imuVelC_.angular() = anchorImuKine.angVel();

  // update the pose as PTransformd for the logs
  X_C_IMU_.translation() = anchorImuKine.position();
  X_C_IMU_.rotation() = anchorImuKine.orientation.toMatrix3().transpose();

  /* Update of the observed robot */
  my_robots_->robot().mbc().q = ctl.realRobot().mbc().q;
  update(my_robots_->robot(), ctl);
  std::cout << std::endl << "1: " << std::endl;

  return true;
}

void TiltObserver::updatePoseAndVel(const mc_control::MCController & ctl,
                                    const so::Vector3 & localWorldImuLinVel,
                                    const so::Vector3 & localWorldImuAngVel)
{
  poseW_.rotation() = R_0_fb_.transpose();

  realWorldFbKine_.orientation = R_0_fb_; // we take into account the newly estimated orientation
  std::cout << std::endl << "2: " << std::endl;
  so::kine::Kinematics realFbAnchorKine = realWorldFbKine_.getInverse() * realWorldAnchorKine_;

  poseW_.translation() =
      R_0_fb_
      * (worldFbKine_.orientation.toMatrix3().transpose() * worldAnchorKine_.position() - realFbAnchorKine.position());
  std::cout << std::endl << "3: " << std::endl;
  if(!anchorFrameJumped_)
  {
    sva::MotionVecd errVel = sva::transformError(prevPoseW_, poseW_) / ctl.timeStep;

    // std::cout << std::endl << "correctedWorldFbKine.orientation: " << std::endl << correctedWorldFbKine << std::endl;
    // std::cout << std::endl << "fbImuKine_.orientation: " << std::endl << fbImuKine_ << std::endl;
    so::kine::Kinematics correctedWorldImuKine =
        realWorldFbKine_ * fbImuKine_; // corrected pose of the imu in the world. This step is used only to get the pose
                                       // of the IMU in the world that is required for the kinematics composition.
    correctedWorldImuKine.linVel = correctedWorldImuKine.orientation * localWorldImuLinVel;

    correctedWorldImuKine.angVel = correctedWorldImuKine.orientation * localWorldImuAngVel;
    std::cout << std::endl << "4: " << std::endl;
    // std::cout << std::endl << "correctedWorldImuKine: " << std::endl << correctedWorldImuKine << std::endl;
    realWorldFbKine_ = correctedWorldImuKine * fbImuKine_.getInverse();
    // std::cout << std::endl << "fbImuKine_.getInverse(): " << std::endl << fbImuKine_.getInverse() << std::endl;
    // std::cout << std::endl << "correctedWorldFbKine: " << std::endl << correctedWorldFbKine << std::endl;
    std::cout << std::endl << "5: " << std::endl;
    so::Vector3 vel =
        (realWorldFbKine_.angVel())
            .cross(worldFbKine_.orientation.toMatrix3().transpose() * worldAnchorKine_.position()
                   - realFbAnchorKine.position())
        + realWorldFbKine_.orientation
              * (worldFbKine_.orientation.toMatrix3().transpose()
                     * (worldAnchorKine_.linVel() - (worldFbKine_.angVel()).cross(worldAnchorKine_.position()))
                 - realFbAnchorKine.linVel());
    std::cout << std::endl << "6: " << std::endl;
    // delete from here...
    auto & rimu = const_cast<mc_rbdyn::BodySensor &>(ctl.realRobot(robot_).bodySensor(imuSensor_));

    so::kine::Kinematics realRobotParentImuPose = kinematicsTools::poseFromSva(
        rimu.X_b_s(), so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vels);
    so::kine::Kinematics realRobotWorldParentKine = kinematicsTools::poseAndVelFromSva(
        ctl.realRobot(robot_).mbc().bodyPosW[ctl.realRobot(robot_).bodyIndexByName(rimu.parentBody())],
        ctl.realRobot(robot_).mbc().bodyVelW[ctl.realRobot(robot_).bodyIndexByName(rimu.parentBody())], true);
    std::cout << std::endl << "7: " << std::endl;
    so::kine::Kinematics realRobotWorldImuKine = realRobotWorldParentKine * realRobotParentImuPose;
    estimatedWorldImuLocalLinVel_ = localWorldImuLinVel;
    virtualMeasureWorldImuLocalLinVel_ = estimator_.getVirtualLocalVelocityMeasurement();
    realRobotWorldImuLocalLinVel_ =
        realRobotWorldImuKine.orientation.toMatrix3().transpose() * realRobotWorldImuKine.linVel();

    oldRealRobotWorldAnchorKine_.update(kinematicsTools::poseFromSva(X_0_C_real_, so::kine::Kinematics::Flags::pose),
                                        ctl.timeStep, flagPoseVels_);

    // to here
    velW_.linear() = vel;
    velW_.angular() = realWorldFbKine_.angVel();
  }
  else
  {
    mc_rtc::log::warning("[{}] Skipping velocity update (anchor frame jumped)", name());
  }
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
  robot.velW(velW_);
}

void TiltObserver::addToLogger(const mc_control::MCController &, mc_rtc::Logger & logger, const std::string & category)
{
  logger.addLogEntry(category + "_controlAnchorFrame", [this]() -> const sva::PTransformd & { return X_0_C_; });

  logger.addLogEntry(category + "_IMU_world_orientation",
                     [this]() { return Eigen::Quaterniond{estimatedRotationIMU_}; });
  logger.addLogEntry(category + "_IMU_world_localLinVel", [this]() -> so::Vector3 { return xk_.head(3); });
  logger.addLogEntry(category + "_IMU_AnchorFrame_pose", [this]() -> const sva::PTransformd & { return X_C_IMU_; });
  logger.addLogEntry(category + "_IMU_AnchorFrame_linVel", [this]() -> const sva::MotionVecd & { return imuVelC_; });
  logger.addLogEntry(category + "_AnchorFrame_world_linVel_global",
                     [this]() -> so::Vector3 { return worldAnchorKine_.linVel(); });
  logger.addLogEntry(category + "_AnchorFrame_world_linVel_local",
                     [this]() -> so::Vector3
                     { return worldAnchorKine_.orientation.toMatrix3().transpose() * worldAnchorKine_.linVel(); });
  logger.addLogEntry(category + "_AnchorFrame_world_angVel",
                     [this]() -> const so::Vector3 & { return worldAnchorKine_.angVel(); });
  logger.addLogEntry(category + "_FloatingBase_world_pose", [this]() -> const sva::PTransformd & { return poseW_; });
  logger.addLogEntry(category + "_FloatingBase_world_vel", [this]() -> const sva::MotionVecd & { return velW_; });
  logger.addLogEntry(category + "_debug_realRobotWorldImuLocalLinVel_",
                     [this]() -> const so::Vector3 & { return realRobotWorldImuLocalLinVel_; });
  logger.addLogEntry(category + "_debug_x1_", [this]() -> const so::Vector3 & { return x1_; });
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
