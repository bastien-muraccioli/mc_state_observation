#include <mc_control/MCController.h>
#include <mc_observers/ObserverMacros.h>
#include <iostream>
#include <mc_state_observation/TiltObserver.h>
#include <mc_state_observation/gui_helpers.h>
#include <mc_state_observation/observersTools/kinematicsTools.h>

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

  std::string odometryType = static_cast<std::string>(config("odometryType"));
  bool odometry6d;

  if(odometryType != "None")
  {
    withOdometry_ = true;

    if(odometryType == "flatOdometry")
    {
      odometry6d = false;
    }
    else if(odometryType == "6dOdometry")
    {
      odometry6d = true;
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "Odometry type not allowed. Please pick among : [None, flatOdometry, 6dOdometry]");
    }
  }

  // if(ctl.datastore().has("KoBackupInterval")) // not working because called before MCKO
  bool asBackup = false;
  config("asBackup", asBackup);
  if(asBackup)
  {
    BOOST_ASSERT(withOdometry_ && "The odometry must be used to perform backup");
    auto & datastore = (const_cast<mc_control::MCController &>(ctl)).datastore();
    ctl.gui()->addElement({"OdometryBackup"}, mc_rtc::gui::Button("OdometryBackup", [this, &ctl]() { backupFb(ctl); }));

    datastore.make_call("runBackup", [this, &ctl]() -> const so::kine::Kinematics { return backupFb(ctl); });
  }

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

  if(withOdometry_)
  {
    std::vector<std::string> surfacesForContactDetection = config("surfacesForContactDetection");
    double contactDetectionPropThreshold = config("contactDetectionPropThreshold");
    bool withYawEstimation = config("withYawEstimation");
    std::vector<std::string> contactsSensorsDisabledInit = config("contactsSensorDisabledInit");

    std::string contactsDetection = static_cast<std::string>(config("contactsDetection"));

    if(contactsDetection != "fromSolver" && contactsDetection != "fromThreshold" && contactsDetection != "fromSurfaces")
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "Contacts detection type not allowed. Please pick among : [fromSolver, fromThreshold, fromSurfaces] or "
          "initialize a list of surfaces with the variable surfacesForContactDetection");
    }
    if(surfacesForContactDetection.size() > 0)
    {
      if(contactsDetection != "fromSurfaces")
      {
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "Another type of contacts detection is currently used, please change it to 'fromSurfaces' or empty the "
            "surfacesForContactDetection variable");
      }
    }
    else if(contactsDetection == "fromSurfaces")
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "You selected the contacts detection using surfaces but didn't add the list of surfaces, please add it usign "
          "the variable surfacesForContactDetection");
    }

    const auto & robot = ctl.robot(robot_);
    double contactDetectionThreshold = robot.mass() * so::cst::gravityConstant * contactDetectionPropThreshold_;
    std::vector<std::string> contactsSensorDisabledInit = config("contactsSensorDisabledInit");
    if(contactsDetection == "fromSurfaces")
    {
      odometryManager_.init(ctl, robot_, "TiltObserver", odometry6d, withYawEstimation, contactsDetection,
                            surfacesForContactDetection, contactsSensorDisabledInit, contactDetectionThreshold);
    }
    else
    {
      odometryManager_.init(ctl, robot_, "TiltObserver", odometry6d, withYawEstimation, contactsDetection,
                            contactsSensorDisabledInit, contactDetectionThreshold);
    }
  }
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
}

void TiltObserver::updateAnchorFrame(const mc_control::MCController & ctl)
{
  if(withOdometry_)
  {
    updateAnchorFrameOdometry(ctl);
  }
  else
  {
    updateAnchorFrameNoOdometry(ctl);
  }
}

void TiltObserver::updateAnchorFrameOdometry(const mc_control::MCController & ctl)
{
  auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();

  odometryManager_.run(ctl, logger, poseW_, velW_); // we update the pose and velocities of the floating base

  so::kine::Kinematics worldAnchorKine = odometryManager_.getAnchorFramePose(ctl);
  X_0_C_.translation() = worldAnchorKine.position();
  X_0_C_.rotation() = worldAnchorKine.orientation.toMatrix3().transpose();
  X_0_C_real_ = X_0_C_;
}

void TiltObserver::updateAnchorFrameNoOdometry(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);

  anchorFrameJumped_ = false;

  if(!ctl.datastore().has(anchorFrameFunction_))
  {
    error_ = fmt::format(
        "Observer {} requires a \"{}\" function in the datastore to provide the observer's kinematic anchor frame.\n"
        "Please refer to https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/observers.html for further details.",
        name(), anchorFrameFunction_);

    double leftFootRatio = robot.indirectSurfaceForceSensor("LeftFootCenter").force().z()
                           / (robot.indirectSurfaceForceSensor("LeftFootCenter").force().z()
                              + robot.indirectSurfaceForceSensor("RightFootCenter").force().z());

    X_0_C_ = sva::interpolate(robot.surfacePose("RightFootCenter"), robot.surfacePose("LeftFootCenter"), leftFootRatio);
    X_0_C_real_ = sva::interpolate(realRobot.surfacePose("RightFootCenter"), realRobot.surfacePose("LeftFootCenter"),
                                   leftFootRatio);
  }
  else
  {
    X_0_C_ = ctl.datastore().call<sva::PTransformd>(anchorFrameFunction_, ctl.robot(robot_));
    X_0_C_real_ = ctl.datastore().call<sva::PTransformd>(anchorFrameFunction_, ctl.realRobot(robot_));
  }

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
}

void TiltObserver::runTiltEstimator(const mc_control::MCController & ctl, const mc_rbdyn::Robot & realRobot)
{
  const auto & robot = ctl.robot(robot_);

  estimator_.setAlpha(alpha_);
  estimator_.setBeta(beta_);
  estimator_.setGamma(gamma_);

  updateAnchorFrame(ctl);

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

  // Compute velocity of the imu in the control frame
  auto v_0_imuParent = robot.mbc().bodyVelW[robot.bodyIndexByName(imu.parentBody())];

  auto real_v_0_imuParent = realRobot.mbc().bodyVelW[realRobot.bodyIndexByName(rimu.parentBody())];

  so::kine::Kinematics worldParentKine = kinematicsTools::poseAndVelFromSva(parentPoseW, v_0_imuParent, true);
  so::kine::Kinematics realWorldParentKine =
      kinematicsTools::poseAndVelFromSva(realParentPoseW, real_v_0_imuParent, true);

  worldImuKine_ = worldParentKine * parentImuKine;
  realWorldImuKine_ = realWorldParentKine * realParentImuKine;

  so::kine::Kinematics newWorldAnchorKine = kinematicsTools::poseFromSva(X_0_C_, so::kine::Kinematics::Flags::pose);

  so::kine::Kinematics newRealWorldAnchorKine =
      kinematicsTools::poseFromSva(X_0_C_real_, so::kine::Kinematics::Flags::pose);

  worldAnchorKine_.update(newWorldAnchorKine, ctl.timeStep, flagPoseVels_);

  realWorldAnchorKine_.update(newRealWorldAnchorKine, ctl.timeStep,
                              flagPoseVels_); // this is not the real robot's anchor frame

  // Pose of the imu in the control frame
  so::kine::Kinematics realAnchorImuKine = realWorldAnchorKine_.getInverse() * realWorldImuKine_;
  realImuAnchorKine_ = realAnchorImuKine.getInverse();
  realFbImuKine_ = realWorldFbKine_.getInverse() * realWorldImuKine_;

  BOOST_ASSERT((realAnchorImuKine.linVel.isSet() || realAnchorImuKine.angVel.isSet()) && "vels were not computed");

  estimator_.setSensorPositionInC(realAnchorImuKine.position());
  estimator_.setSensorOrientationInC(realAnchorImuKine.orientation.toMatrix3());
  estimator_.setSensorLinearVelocityInC(realAnchorImuKine.linVel());
  estimator_.setSensorAngularVelocityInC(realAnchorImuKine.angVel());
  estimator_.setControlOriginVelocityInW(worldAnchorKine_.orientation.toMatrix3().transpose()
                                         * worldAnchorKine_.linVel());

  auto k = estimator_.getCurrentTime();

  x1_ = realWorldImuKine_.orientation.toMatrix3().transpose() * worldAnchorKine_.linVel() - realImuAnchorKine_.linVel()
        - (imu.angularVelocity()).cross(realImuAnchorKine_.position());

  /*
  x1_ = (realWorldImuKine_.orientation.toMatrix3().transpose() * realWorldImuKine_.angVel())
            .cross(worldImuKine_.orientation.toMatrix3().transpose() * worldAnchorKine_.position()
                   + realAnchorImuKine.orientation.toMatrix3().transpose() * realAnchorImuKine.position())
        + worldImuKine_.orientation.toMatrix3().transpose() * worldAnchorKine_.linVel()
        - (realAnchorImuKine.orientation.toMatrix3().transpose() * realAnchorImuKine.angVel())
              .cross(realAnchorImuKine.orientation.toMatrix3().transpose() * realAnchorImuKine.position())
        + realAnchorImuKine.orientation.toMatrix3().transpose() * realAnchorImuKine.linVel()
        - (worldImuKine_.orientation.toMatrix3().transpose() * worldImuKine_.angVel())
              .cross(worldImuKine_.orientation.toMatrix3().transpose() * worldAnchorKine_.position());
              */

  estimator_.setMeasurement(imu.linearAcceleration(), imu.angularVelocity(), k + 1);
  // estimator_.setExplicitX1(x1_); // we directly give the virtual measurement of the velocity by the IMU

  xk_ = estimator_.getEstimatedState(k + 1);

  so::Vector3 tilt = xk_.tail(3);

  // Orientation of the imu in the world obtained from the estimated tilt and the yaw of the control robot.
  estimatedRotationIMU_ = so::kine::mergeTiltWithYaw(tilt, worldImuKine_.orientation.toMatrix3());

  // Estimated orientation of the floating base in the world
  R_0_fb_ = estimatedRotationIMU_ * realFbImuKine_.orientation.toMatrix3().transpose();
  updatePoseAndVel(ctl, xk_.head(3),
                   imu.angularVelocity()); // we should use xk_.head(3) for the local linear velocity

  // update the velocities as MotionVecd for the logs
  imuVelC_.linear() = realAnchorImuKine.linVel();
  imuVelC_.angular() = realAnchorImuKine.angVel();

  // update the pose as PTransformd for the logs
  X_C_IMU_.translation() = realAnchorImuKine.position();
  X_C_IMU_.rotation() = realAnchorImuKine.orientation.toMatrix3().transpose();

  /* Update of the observed robot */
  my_robots_->robot().mbc().q = ctl.realRobot().mbc().q;
  update(my_robots_->robot(), ctl);

  // delete from here...

  estimatedWorldImuLocalLinVel_ = estimator_.getVirtualLocalVelocityMeasurement();
  virtualMeasureWorldImuLocalLinVel_ = estimator_.getVirtualLocalVelocityMeasurement();
  realRobotWorldImuLocalLinVel_ = realWorldImuKine_.orientation.toMatrix3().transpose() * realWorldImuKine_.linVel();
  realRobotWorldImuLocalAngVel_ = realWorldImuKine_.orientation.toMatrix3().transpose() * realWorldImuKine_.angVel();
}

bool TiltObserver::run(const mc_control::MCController & ctl)
{
  if(!withOdometry_)
  {
    const auto & realRobot = ctl.realRobot(robot_);
    runTiltEstimator(ctl, realRobot);
  }
  else
  {
    // odometryManager_.updateOdometryRobot(ctl, true, false);
    runTiltEstimator(ctl, odometryManager_.odometryRobot());

    backupFbKinematics_.push_back(odometryManager_.odometryRobot().posW());
  }

  return true;
}

void TiltObserver::updatePoseAndVel(const mc_control::MCController & ctl,
                                    const so::Vector3 & localWorldImuLinVel,
                                    const so::Vector3 & localWorldImuAngVel)
{
  if(!withOdometry_) // if we use odometry, the position is updated during the anchor frame update
  {
    poseW_.rotation() = R_0_fb_.transpose();

    realWorldFbKine_.orientation = R_0_fb_; // we replace the previous orientation estimation by the newly estimated one

    realFbAnchorKine_ = realWorldFbKine_.getInverse() * realWorldAnchorKine_;

    std::cout << std::endl << realWorldFbKine_ << std::endl;

    poseW_.translation() = R_0_fb_
                           * (worldFbKine_.orientation.toMatrix3().transpose() * worldAnchorKine_.position()
                              - realFbAnchorKine_.position());
  }
  else
  {
    // we combine the yaw estimated by the odometry with the newly estimated tilt
    poseW_.rotation() = (so::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(
                             R_0_fb_, odometryManager_.odometryRobot().posW().rotation().transpose()))
                            .transpose();
  }

  if(!anchorFrameJumped_)
  {
    sva::MotionVecd errVel = sva::transformError(prevPoseW_, poseW_) / ctl.timeStep;
  }
  else
  {
    mc_rtc::log::warning("[{}] Skipping velocity update (anchor frame jumped)", name());
  }
  so::kine::Kinematics correctedWorldImuKine =
      realWorldFbKine_
      * realFbImuKine_; // corrected pose of the imu in the world. This step is used only to get the pose
                        // of the IMU in the world that is required for the kinematics composition.
  correctedWorldImuKine.linVel = correctedWorldImuKine.orientation * localWorldImuLinVel;

  correctedWorldImuKine.angVel = correctedWorldImuKine.orientation * localWorldImuAngVel;

  realWorldFbKine_ = correctedWorldImuKine * realFbImuKine_.getInverse();

  realFbAnchorKine_ = realWorldFbKine_.getInverse() * realWorldAnchorKine_;

  so::Vector3 vel = worldAnchorKine_.linVel() - realWorldFbKine_.orientation * realFbAnchorKine_.linVel()
                    - (realWorldFbKine_.angVel()).cross(realWorldFbKine_.orientation * realFbAnchorKine_.position());

  velW_.linear() = vel;
  velW_.angular() = realWorldFbKine_.angVel();
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

const so::kine::Kinematics TiltObserver::backupFb(const mc_control::MCController & ctl)
{
  // new initial pose of the floating base

  boost::circular_buffer<so::kine::Kinematics> * koBackupFbKinematics =
      ctl.datastore().get<boost::circular_buffer<so::kine::Kinematics> *>("koBackupFbKinematics");
  so::kine::Kinematics worldResetKine = *(koBackupFbKinematics->begin());

  // so::kine::Kinematics worldResetKine = so::kine::Kinematics::zeroKinematics(so::kine::Kinematics::Flags::pose);

  // original initial pose of the floating base
  so::kine::Kinematics worldFbInitBackup =
      kinematicsTools::poseFromSva(backupFbKinematics_.front(), so::kine::Kinematics::Flags::pose);

  so::kine::Kinematics fbWorldInitBackup = worldFbInitBackup.getInverse();

  // we apply the transformation from the initial pose to the intermediates pose estimated by the tilt estimator to the
  // new starting pose of the Kinetics Observer
  for(int i = 0; i < koBackupFbKinematics->size(); i++)
  {
    // Intermediary pose of the floating base estimated by the tilt estimator
    so::kine::Kinematics worldFbIntermBackup =
        kinematicsTools::poseFromSva(backupFbKinematics_.at(i), so::kine::Kinematics::Flags::pose);
    // transformation between the initial and the intermediary pose during the backup interval
    so::kine::Kinematics initInterm = fbWorldInitBackup * worldFbIntermBackup;

    koBackupFbKinematics->at(i) = worldResetKine * initInterm;
  }

  so::Vector3 tiltLocalLinVel = poseW_.rotation() * velW_.linear();
  so::Vector3 tiltLocalAngVel = poseW_.rotation() * velW_.angular();

  // koBackupFbKinematics->back() is the new last pose of the kinetics observer
  koBackupFbKinematics->back().linVel = koBackupFbKinematics->back().orientation.toMatrix3() * tiltLocalLinVel;
  koBackupFbKinematics->back().angVel = koBackupFbKinematics->back().orientation.toMatrix3() * tiltLocalAngVel;

  return koBackupFbKinematics->back();
}

void TiltObserver::addToLogger(const mc_control::MCController & ctl,
                               mc_rtc::Logger & logger,
                               const std::string & category)
{
  logger.addLogEntry(category + "_controlAnchorFrame", [this]() -> const sva::PTransformd & { return X_0_C_; });

  logger.addLogEntry(category + "_IMU_world_orientation",
                     [this]() { return Eigen::Quaterniond{estimatedRotationIMU_}; });
  logger.addLogEntry(category + "_IMU_world_localLinVel", [this]() -> so::Vector3 { return xk_.head(3); });
  logger.addLogEntry(category + "_IMU_AnchorFrame_pose", [this]() -> const sva::PTransformd & { return X_C_IMU_; });
  logger.addLogEntry(category + "_IMU_AnchorFrame_linVel", [this]() -> const sva::MotionVecd & { return imuVelC_; });
  logger.addLogEntry(category + "_AnchorFrame_world_position",
                     [this]() -> so::Vector3 { return worldAnchorKine_.position(); });
  logger.addLogEntry(category + "_AnchorFrame_world_orientation",
                     [this]() -> so::Vector3 { return worldAnchorKine_.orientation.toRotationVector(); });
  logger.addLogEntry(category + "_AnchorFrame_world_linVel_global",
                     [this]() -> so::Vector3 { return worldAnchorKine_.linVel(); });
  logger.addLogEntry(category + "_AnchorFrame_world_linVel_local",
                     [this]() -> so::Vector3
                     { return worldAnchorKine_.orientation.toMatrix3().transpose() * worldAnchorKine_.linVel(); });
  logger.addLogEntry(category + "_AnchorFrame_world_angVel",
                     [this]() -> const so::Vector3 & { return worldAnchorKine_.angVel(); });
  logger.addLogEntry(category + "_FloatingBase_world_pose", [this]() -> const sva::PTransformd & { return poseW_; });
  logger.addLogEntry(category + "_FloatingBase_world_vel", [this]() -> const sva::MotionVecd & { return velW_; });
  logger.addLogEntry(category + "_debug_realRobotWorldImuLocalLinVel",
                     [this]() -> const so::Vector3 & { return realRobotWorldImuLocalLinVel_; });
  logger.addLogEntry(category + "_debug_realRobotWorldImuLocalAngVel",
                     [this]() -> const so::Vector3 & { return realRobotWorldImuLocalAngVel_; });
  logger.addLogEntry(category + "_debug_x1", [this]() -> const so::Vector3 & { return x1_; });

  logger.addLogEntry(category + "_debug_x1_part1",
                     [this, &ctl]() -> so::Vector3
                     {
                       auto & imu = const_cast<mc_rbdyn::BodySensor &>(ctl.robot(robot_).bodySensor(imuSensor_));
                       return worldImuKine_.orientation.toMatrix3().transpose()
                              * ((so::kine::skewSymmetric(worldImuKine_.orientation * imu.angularVelocity())
                                  - so::kine::skewSymmetric(worldImuKine_.angVel()))
                                 * worldAnchorKine_.position());
                       ;
                     });
  logger.addLogEntry(category + "_debug_x1_part2",
                     [this, &ctl]() -> so::Vector3
                     {
                       auto & imu = const_cast<mc_rbdyn::BodySensor &>(ctl.robot(robot_).bodySensor(imuSensor_));
                       return -imu.angularVelocity().cross(realImuAnchorKine_.position());
                       ;
                     });
  logger.addLogEntry(category + "_debug_x1_part3",
                     [this, &ctl]() -> so::Vector3
                     {
                       auto & imu = const_cast<mc_rbdyn::BodySensor &>(ctl.robot(robot_).bodySensor(imuSensor_));
                       return worldImuKine_.orientation.toMatrix3().transpose() * worldAnchorKine_.linVel();
                       ;
                     });
  logger.addLogEntry(category + "_debug_x1_part4",
                     [this, &ctl]() -> so::Vector3
                     {
                       auto & imu = const_cast<mc_rbdyn::BodySensor &>(ctl.robot(robot_).bodySensor(imuSensor_));
                       return -realImuAnchorKine_.linVel();
                       ;
                     });

  logger.addLogEntry(category + "_debug_x1_part5",
                     [this, &ctl]() -> so::Vector3
                     {
                       auto & imu = const_cast<mc_rbdyn::BodySensor &>(ctl.robot(robot_).bodySensor(imuSensor_));
                       return worldImuKine_.orientation.toMatrix3().transpose()
                              * (so::kine::skewSymmetric(worldImuKine_.orientation * imu.angularVelocity())
                                 * worldAnchorKine_.position());
                       ;
                     });
  logger.addLogEntry(category + "_debug_x1_part6",
                     [this, &ctl]() -> so::Vector3
                     {
                       auto & imu = const_cast<mc_rbdyn::BodySensor &>(ctl.robot(robot_).bodySensor(imuSensor_));
                       return so::kine::skewSymmetric(imu.angularVelocity())
                              * (worldImuKine_.orientation.toMatrix3().transpose() * worldAnchorKine_.position());
                       ;
                     });
  logger.addLogEntry(category + "_debug_x1_part6bis",
                     [this, &ctl]() -> so::Vector3
                     {
                       auto & imu = const_cast<mc_rbdyn::BodySensor &>(ctl.robot(robot_).bodySensor(imuSensor_));
                       return worldImuKine_.orientation.toMatrix3().transpose() * worldAnchorKine_.position();
                       ;
                     });
  logger.addLogEntry(category + "_debug_x1_part6ter",
                     [this, &ctl]() -> so::Vector3
                     {
                       auto & imu = const_cast<mc_rbdyn::BodySensor &>(ctl.robot(robot_).bodySensor(imuSensor_));
                       return -worldImuKine_.orientation.toMatrix3().transpose()
                              * (worldImuKine_.angVel()).cross(worldAnchorKine_.position());
                       ;
                     });
  logger.addLogEntry(category + "_debug_x1_part6quat",
                     [this, &ctl]() -> so::Vector3
                     {
                       auto & imu = const_cast<mc_rbdyn::BodySensor &>(ctl.robot(robot_).bodySensor(imuSensor_));
                       return -worldImuKine_.orientation.toMatrix3().transpose() * (worldImuKine_.angVel());
                       ;
                     });
  logger.addLogEntry(category + "_debug_x1_part7",
                     [this, &ctl]() -> so::Vector3
                     {
                       auto & imu = const_cast<mc_rbdyn::BodySensor &>(ctl.robot(robot_).bodySensor(imuSensor_));
                       return worldImuKine_.orientation * imu.angularVelocity() - worldImuKine_.angVel();
                       ;
                     });
  logger.addLogEntry(category + "_debug_x1_part8",
                     [this, &ctl]() -> so::Vector3
                     {
                       auto & imu = const_cast<mc_rbdyn::BodySensor &>(ctl.robot(robot_).bodySensor(imuSensor_));
                       return worldImuKine_.orientation * imu.angularVelocity();
                       ;
                     });
  logger.addLogEntry(category + "_debug_x1_part9",
                     [this, &ctl]() -> so::Vector3
                     {
                       auto & imu = const_cast<mc_rbdyn::BodySensor &>(ctl.robot(robot_).bodySensor(imuSensor_));
                       return -worldImuKine_.angVel();
                       ;
                     });

  logger.addLogEntry(category + "_debug_x1_other",
                     [this, &ctl]() -> so::Vector3
                     {
                       auto & imu = const_cast<mc_rbdyn::BodySensor &>(ctl.robot(robot_).bodySensor(imuSensor_));
                       return worldImuKine_.orientation.toMatrix3().transpose()
                                  * ((so::kine::skewSymmetric(worldImuKine_.orientation * imu.angularVelocity())
                                      - so::kine::skewSymmetric(worldImuKine_.angVel()))
                                     * worldAnchorKine_.position())
                              - imu.angularVelocity().cross(realImuAnchorKine_.position())
                              + worldImuKine_.orientation.toMatrix3().transpose() * worldAnchorKine_.linVel()
                              - realImuAnchorKine_.linVel();
                       ;
                     });

  kinematicsTools::addToLogger(worldAnchorKine_, logger, category + "_debug_worldAnchorKine");
  kinematicsTools::addToLogger(realWorldImuKine_, logger, category + "_debug_realWorldImuKine");
  kinematicsTools::addToLogger(worldImuKine_, logger, category + "_debug_worldImuKine");
  kinematicsTools::addToLogger(realWorldAnchorKine_, logger, category + "_debug_realWorldAnchorKine");
  kinematicsTools::addToLogger(realImuAnchorKine_, logger, category + "_debug_realImuAnchorKine_");

  /*
  x1_ = (imu.angularVelocity())
            .cross(worldImuKine_.orientation.toMatrix3().transpose() * worldAnchorKine_.position()
                   - realImuAnchorKine.position())
        + worldImuKine_.orientation.toMatrix3().transpose()
              * (worldAnchorKine_.linVel() - (worldImuKine_.angVel()).cross(worldAnchorKine_.position()))
        - realImuAnchorKine.linVel(); */
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
