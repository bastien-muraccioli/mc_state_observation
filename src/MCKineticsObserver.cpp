/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#include <mc_control/MCController.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/io_utils.h>
#include <mc_rtc/logging.h>
#include <mc_state_observation/MCKineticsObserver.h>
#include <mc_state_observation/gui_helpers.h>

#include <RBDyn/CoM.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <typeinfo>

#include <iostream>

#include <mc_state_observation/observersTools/kinematicsTools.h>

namespace so = stateObservation;

namespace mc_state_observation
{
MCKineticsObserver::MCKineticsObserver(const std::string & type, double dt)
: mc_observers::Observer(type, dt), observer_(4, 2)
{
  observer_.setSamplingTime(dt);
}

///////////////////////////////////////////////////////////////////////
/// --------------------------Core functions---------------------------
///////////////////////////////////////////////////////////////////////

void MCKineticsObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  IMUs_ = config("imuSensor", ctl.robot().bodySensors());
  config("debug", debug_);
  config("verbose", verbose_);

  std::string odometryType = static_cast<std::string>(config("odometryType"));

  if(odometryType != "None")
  {
    if(odometryType == "flatOdometry")
    {
      withOdometry_ = true;
      withFlatOdometry_ = true;
    }
    else if(odometryType == "6dOdometry")
    {
      withOdometry_ = true;
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "Odometry type not allowed. Please pick among : [None, flatOdometry, 6dOdometry]");
    }
  }

  config("withDebugLogs", withDebugLogs_);
  config("contactDetectionPropThreshold", contactDetectionPropThreshold_);
  config("withFilteredForcesContactDetection", withFilteredForcesContactDetection_);
  if(withFilteredForcesContactDetection_)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("The forces filtering has an error, please don't use it now");
  }
  config("withUnmodeledWrench", withUnmodeledWrench_);
  config("withGyroBias", withGyroBias_);

  observer_.setWithUnmodeledWrench(withUnmodeledWrench_);
  observer_.setWithGyroBias(withGyroBias_);
  observer_.useFiniteDifferencesJacobians(config("withFiniteDifferences"));
  so::Vector dx(observer_.getStateSize());
  dx.setConstant(static_cast<double>(config("finiteDifferenceStep")));
  observer_.setFiniteDifferenceStep(dx);
  observer_.setWithAccelerationEstimation(config("withAccelerationEstimation"));

  linStiffness_ = (config("linStiffness").operator so::Vector3()).matrix().asDiagonal();
  angStiffness_ = (config("angStiffness").operator so::Vector3()).matrix().asDiagonal();
  linDamping_ = (config("linDamping").operator so::Vector3()).matrix().asDiagonal();
  angDamping_ = (config("angDamping").operator so::Vector3()).matrix().asDiagonal();

  zeroPose_.translation().setZero();
  zeroPose_.rotation().setIdentity();
  zeroMotion_.linear().setZero();
  zeroMotion_.angular().setZero();

  // Initial State
  statePositionInitCovariance_ = (config("statePositionInitVariance").operator so::Vector3()).matrix().asDiagonal();
  stateOriInitCovariance_ = (config("stateOriInitVariance").operator so::Vector3()).matrix().asDiagonal();
  stateLinVelInitCovariance_ = (config("stateLinVelInitVariance").operator so::Vector3()).matrix().asDiagonal();
  stateAngVelInitCovariance_ = (config("stateAngVelInitVariance").operator so::Vector3()).matrix().asDiagonal();
  gyroBiasInitCovariance_.setZero();
  unmodeledWrenchInitCovariance_.setZero();
  contactInitCovarianceFirstContacts_.setZero();
  contactInitCovarianceFirstContacts_.block<3, 3>(0, 0) =
      (config("contactPositionInitVarianceFirstContacts").operator so::Vector3()).matrix().asDiagonal();
  contactInitCovarianceFirstContacts_.block<3, 3>(3, 3) =
      (config("contactOriInitVarianceFirstContacts").operator so::Vector3()).matrix().asDiagonal();
  contactInitCovarianceFirstContacts_.block<3, 3>(6, 6) =
      (config("contactForceInitVarianceFirstContacts").operator so::Vector3()).matrix().asDiagonal();
  contactInitCovarianceFirstContacts_.block<3, 3>(9, 9) =
      (config("contactTorqueInitVarianceFirstContacts").operator so::Vector3()).matrix().asDiagonal();

  contactInitCovarianceNewContacts_.setZero();
  contactInitCovarianceNewContacts_.block<3, 3>(0, 0) =
      (config("contactPositionInitVarianceNewContacts").operator so::Vector3()).matrix().asDiagonal();
  contactInitCovarianceNewContacts_.block<3, 3>(3, 3) =
      (config("contactOriInitVarianceNewContacts").operator so::Vector3()).matrix().asDiagonal();
  contactInitCovarianceNewContacts_.block<3, 3>(6, 6) =
      (config("contactForceInitVarianceNewContacts").operator so::Vector3()).matrix().asDiagonal();
  contactInitCovarianceNewContacts_.block<3, 3>(9, 9) =
      (config("contactTorqueInitVarianceNewContacts").operator so::Vector3()).matrix().asDiagonal();

  // Process //
  statePositionProcessCovariance_ =
      (config("statePositionProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  stateOriProcessCovariance_ = (config("stateOriProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  stateLinVelProcessCovariance_ = (config("stateLinVelProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  stateAngVelProcessCovariance_ = (config("stateAngVelProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  gyroBiasProcessCovariance_.setZero();
  unmodeledWrenchProcessCovariance_.setZero();

  contactProcessCovariance_.setZero();
  contactProcessCovariance_.block<3, 3>(0, 0) =
      (config("contactPositionProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  contactProcessCovariance_.block<3, 3>(3, 3) =
      (config("contactOrientationProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  contactProcessCovariance_.block<3, 3>(6, 6) =
      (config("contactForceProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  contactProcessCovariance_.block<3, 3>(9, 9) =
      (config("contactTorqueProcessVariance").operator so::Vector3()).matrix().asDiagonal();

  // Unmodeled Wrench //
  if(withUnmodeledWrench_)
  {
    // initial
    unmodeledWrenchInitCovariance_.block<3, 3>(0, 0) =
        (config("unmodeledForceInitVariance").operator so::Vector3()).matrix().asDiagonal();
    unmodeledWrenchInitCovariance_.block<3, 3>(3, 3) =
        (config("unmodeledTorqueInitVariance").operator so::Vector3()).matrix().asDiagonal();

    // process
    unmodeledWrenchProcessCovariance_.block<3, 3>(0, 0) =
        (config("unmodeledForceProcessVariance").operator so::Vector3()).matrix().asDiagonal();
    unmodeledWrenchProcessCovariance_.block<3, 3>(3, 3) =
        (config("unmodeledTorqueProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  }
  // Gyrometer Bias
  if(withGyroBias_)
  {
    gyroBiasInitCovariance_ = (config("gyroBiasInitVariance").operator so::Vector3()).matrix().asDiagonal();
    gyroBiasProcessCovariance_ = (config("gyroBiasProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  }

  // Sensor //
  positionSensorCovariance_ = (config("positionSensorVariance").operator so::Vector3()).matrix().asDiagonal();
  orientationSensorCoVariance_ = (config("orientationSensorVariance").operator so::Vector3()).matrix().asDiagonal();
  acceleroSensorCovariance_ = (config("acceleroSensorVariance").operator so::Vector3()).matrix().asDiagonal();
  gyroSensorCovariance_ = (config("gyroSensorVariance").operator so::Vector3()).matrix().asDiagonal();
  contactSensorCovariance_.setZero();
  contactSensorCovariance_.block<3, 3>(0, 0) =
      (config("forceSensorVariance").operator so::Vector3()).matrix().asDiagonal();
  contactSensorCovariance_.block<3, 3>(3, 3) =
      (config("torqueSensorVariance").operator so::Vector3()).matrix().asDiagonal();

  observer_.setAllCovariances(statePositionInitCovariance_, stateOriInitCovariance_, stateLinVelInitCovariance_,
                              stateAngVelInitCovariance_, gyroBiasInitCovariance_, unmodeledWrenchInitCovariance_,
                              contactInitCovarianceFirstContacts_, statePositionProcessCovariance_,
                              stateOriProcessCovariance_, stateLinVelProcessCovariance_, stateAngVelProcessCovariance_,
                              gyroBiasProcessCovariance_, unmodeledWrenchProcessCovariance_, contactProcessCovariance_,
                              positionSensorCovariance_, orientationSensorCoVariance_, acceleroSensorCovariance_,
                              gyroSensorCovariance_, contactSensorCovariance_);

  const auto & robot = ctl.robot(robot_);
  double contactDetectionThreshold = robot.mass() * so::cst::gravityConstant * contactDetectionPropThreshold_;

  std::string contactsDetection = static_cast<std::string>(config("contactsDetection"));
  std::vector<std::string> contactsSensorDisabledInit = config("contactsSensorDisabledInit");
  if(contactsDetection == "fromThreshold")
  {
    contactsDetectionFromThreshold_ = true;
  }
  if(contactsDetection == "fromSurfaces")
  {
    std::vector<std::string> surfacesForContactDetection = config("surfacesForContactDetection");
    contactsManager_.init(ctl, robot_, "MCKineticsObserver", contactsDetection, surfacesForContactDetection,
                          contactsSensorDisabledInit, contactDetectionThreshold);
  }
  else
  {
    contactsManager_.init(ctl, robot_, "MCKineticsObserver", contactsDetection, contactsSensorDisabledInit,
                          contactDetectionThreshold);
  }

  if(config.has("KoBackupInterval"))
  {
    /*
    BOOST_ASSERT(ctl.datastore().has("runBackup")
                 && "The Tilt Observer must be used before the Kinetics Observer when used as a backup");
                 */
    int backupInterval = config("KoBackupInterval");
    auto & datastore = (const_cast<mc_control::MCController &>(ctl)).datastore();
    datastore.make<int>("KoBackupInterval", backupInterval);
    datastore.make<boost::circular_buffer<stateObservation::kine::Kinematics> *>("koBackupFbKinematics",
                                                                                 &koBackupFbKinematics_);
    backupIterInterval_ = backupInterval / ctl.timeStep;
    invincibilityFrame_ = 1.5 / ctl.timeStep;

    ctl.gui()->addElement({"MCKineticsObserver"},
                          mc_rtc::gui::Button("SimulateNanBehaviour", [this]() { observer_.nanDetected_ = true; }));
  }
}

void MCKineticsObserver::reset(const mc_control::MCController & ctl)
{
  // if true, the contact detection has started
  simStarted_ = false;
  // the Kinetics Observer completed at least one iteration
  ekfIsSet_ = false;

  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);
  const auto & realRobotModule = realRobot.module();

  rbd::MultiBodyGraph mergeMbg(realRobotModule.mbg);
  std::map<std::string, std::vector<double>> jointPosByName;
  for(int i = 0; i < realRobotModule.mb.nrJoints(); ++i)
  {
    auto jointName = realRobotModule.mb.joint(i).name();
    auto jointIndex = static_cast<unsigned long>(realRobotModule.mb.jointIndexByName(jointName));
    jointPosByName[jointName] = realRobotModule.mbc.q[jointIndex];
  }

  std::vector<std::string> rootJoints = {};
  int nbJoints = static_cast<int>(realRobot.mb().joints().size());
  for(int i = 0; i < nbJoints; ++i)
  {
    if(realRobot.mb().predecessor(i) == 0)
    {
      rootJoints.push_back(realRobot.mb().joint(i).name());
    }
  }
  for(const auto & joint : rootJoints)
  {
    if(!realRobot.hasJoint(joint))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Robot does not have a joint named {}", joint);
    }
    mergeMbg.mergeSubBodies(realRobotModule.mb.body(0).name(), joint, jointPosByName);
  }

  inertiaWaist_ = mergeMbg.nodeByName(realRobotModule.mb.body(0).name())->body.inertia();
  mass(ctl.realRobot(robot_).mass());

  for(const auto & imu : IMUs_)
  {
    mapIMUs_.insertIMU(imu.name());
  }

  if(debug_)
  {
    mc_rtc::log::info("inertiaWaist = {}", inertiaWaist_);
  }

  my_robots_ = mc_rbdyn::Robots::make();
  my_robots_->robotCopy(robot, robot.name());
  my_robots_->robotCopy(realRobot, "inputRobot");
  ctl.gui()->addElement(
      {"Robots"},
      mc_rtc::gui::Robot("MCKineticsobserver", [this]() -> const mc_rbdyn::Robot & { return my_robots_->robot(); }));
  ctl.gui()->addElement({"Robots"},
                        mc_rtc::gui::Robot("Real", [&ctl]() -> const mc_rbdyn::Robot & { return ctl.realRobot(); }));

  X_0_fb_ = robot.posW().translation();
}

bool MCKineticsObserver::run(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);
  auto & inputRobot = my_robots_->robot("inputRobot");
  auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();

  inputRobot.mbc() = realRobot.mbc();
  inputRobot.mb() = realRobot.mb();

  inputRobot.posW(zeroPose_);
  inputRobot.velW(zeroMotion_);
  inputRobot.accW(zeroMotion_);

  /** Center of mass (assumes FK, FV and FA are already done)
      Must be initialized now as used for the conversion from user to centroid frame !!! **/

  worldCoMKine_.position = inputRobot.com();
  worldCoMKine_.linVel = inputRobot.comVelocity();
  worldCoMKine_.linAcc = inputRobot.comAcceleration();

  observer_.setCenterOfMass(worldCoMKine_.position(), worldCoMKine_.linVel(), worldCoMKine_.linAcc());

  /** Contacts
   * Note that when we use force sensors directly for the contact detection, the pose of the contact is the one of the
   * force sensor and not the contact surface!
   */
  // retrieves the list of contacts and set simStarted to true once a contact is detected
  updateContacts(ctl, findNewContacts(ctl), logger);

  // starts the estimation only if contacts are detected
  if(!simStarted_)
  {
    return true;
  }
  /** Accelerometers **/
  updateIMUs(robot, inputRobot);

  /** Inertias **/
  /** TODO : Merge inertias into CoM inertia and/or get it from fd() **/

  observer_.setCoMAngularMomentum(
      rbd::computeCentroidalMomentum(inputRobot.mb(), inputRobot.mbc(), inputRobot.com()).moment());

  observer_.setCoMInertiaMatrix(so::Matrix3(
      inertiaWaist_.inertia() + observer_.getMass() * so::kine::skewSymmetric2(observer_.getCenterOfMass()())));
  /* Step once, and return result */

  // the ekf is not updated, which means that it still has the initial values
  if(!ekfIsSet_ && withDebugLogs_)
  {
    plotVariablesBeforeUpdate(ctl, logger);
  }

  res_ = observer_.update();

  if(!ekfIsSet_ && withDebugLogs_)
  {
    plotVariablesAfterUpdate(logger);
  }

  ekfIsSet_ = true;

  // Kinematics of the floating base in the real world frame (our estimation goal)
  so::kine::Kinematics mcko_K_0_fb;

  // if no anomaly is detected and if we aren't in the "invicibility frame", we update the floating base with the
  // results of the Kinetics Observer
  if(!(observer_.nanDetected_ || (invincibilityIter_ != 0 && invincibilityIter_ < invincibilityFrame_)))
  {
    /* Core */
    so::kine::Kinematics fbFb; // "Zero" Kinematics
    fbFb.setZero(so::kine::Kinematics::Flags::all);

    // Given, the Kinematics of the floating base inside its own frame (zero kinematics) which is our user
    // frame, the Kinetics Observer will return the kinematics of the floating base in the real world frame.
    mcko_K_0_fb = observer_.getGlobalKinematicsOf(fbFb);

    koBackupFbKinematics_.push_back(mcko_K_0_fb);

    X_0_fb_.rotation() = mcko_K_0_fb.orientation.toMatrix3().transpose();
    X_0_fb_.translation() = mcko_K_0_fb.position();

    /* Bring velocity of the IMU to the origin of the joint : we want the
     * velocity of joint 0, so stop one before the first joint */

    v_fb_0_.angular() = mcko_K_0_fb.angVel();
    v_fb_0_.linear() = mcko_K_0_fb.linVel();

    a_fb_0_.angular() = mcko_K_0_fb.angAcc();
    a_fb_0_.linear() = mcko_K_0_fb.linAcc();
  }
  else
  {
    auto & datastore = (const_cast<mc_control::MCController &>(ctl)).datastore();

    // We add an empty Kinematics object to the floating base pose buffer. This is because the buffer of the tilt
    // observer already contacins the last estimation of the floating base so we prevent a disalignment of the two
    // buffers. This empty Kinematics is filled and returned by the "runBackup" function.
    koBackupFbKinematics_.push_back(so::kine::Kinematics::zeroKinematics(so::kine::Kinematics::Flags::pose));

    mcko_K_0_fb = datastore.call<const so::kine::Kinematics>("runBackup");

    X_0_fb_.rotation() = mcko_K_0_fb.orientation.toMatrix3().transpose();
    X_0_fb_.translation() = mcko_K_0_fb.position();

    // the tilt observer doesn't estimate the acceleration so we get it by finite differences
    a_fb_0_.angular() = (mcko_K_0_fb.angVel() - v_fb_0_.angular()) / ctl.timeStep;
    a_fb_0_.linear() = (mcko_K_0_fb.linVel() - v_fb_0_.linear()) / ctl.timeStep;

    v_fb_0_.angular() = mcko_K_0_fb.angVel();
    v_fb_0_.linear() = mcko_K_0_fb.linVel();

    // a NaN was just detected, we reset the state vector and covariances and start the invicibility frame, during which
    // we let the Kinetics Observer converge before using it again.
    if(observer_.nanDetected_)
    {
      auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();
      if(logger.t() / ctl.timeStep < backupIterInterval_)
      {
        mc_rtc::log::warning("The backup function was called before the required time was ellapsed. The backup will be "
                             "performed using the last {} seconds",
                             logger.t());
      }

      if(logger.t() / ctl.timeStep - lastBackupIter_ < backupIterInterval_)
      {
        mc_rtc::log::warning("The backup function was called again too quickly. The backup will be "
                             "performed using the last {} seconds",
                             logger.t() - lastBackupIter_ * ctl.timeStep);
      }

      // we update update robot as it will be updated at the beginning of the next iteration anyway
      update(inputRobot);
      inputRobot.forwardKinematics();
      so::kine::Kinematics newWorldCentroidKine;
      newWorldCentroidKine.position = inputRobot.com();
      newWorldCentroidKine.linVel = inputRobot.comVelocity();
      // the orientation of the centroid frame is the one of the floating base
      newWorldCentroidKine.orientation = mcko_K_0_fb.orientation;
      newWorldCentroidKine.angVel = mcko_K_0_fb.angVel();

      observer_.setWorldCentroidStateKinematics(newWorldCentroidKine, true);
      observer_.setStateUnmodeledWrench(so::Vector6::Zero(), true);

      for(int i = 0; i < mapIMUs_.getList().size(); i++)
      {
        observer_.setGyroBias(so::Vector3::Zero(), i, true);
      }

      for(const int & contactIndex : contactsManager_.contactsFound())
      {
        KoContactWithSensor contact = contactsManager_.contactWithSensor(contactIndex);

        // Update of the force measurements (the offset due to the gravity changed)
        const mc_rbdyn::ForceSensor & forceSensor = robot.forceSensor(contact.forceSensorName());

        so::kine::Kinematics bodySensorKine =
            kinematicsTools::poseFromSva(forceSensor.X_p_f(), so::kine::Kinematics::Flags::vels);

        so::kine::Kinematics bodySurfaceKine = kinematicsTools::poseFromSva(
            robot.surface(contact.surfaceName()).X_b_s(), so::kine::Kinematics::Flags::vels);

        so::kine::Kinematics surfaceSensorKine = bodySurfaceKine.getInverse() * bodySensorKine;

        updateContactForceMeasurement(contact, surfaceSensorKine, forceSensor.wrenchWithoutGravity(inputRobot));

        so::kine::Kinematics newWorldContactKineRef;

        getOdometryWorldContactReference(ctl, contact, newWorldContactKineRef);

        observer_.setStateContact(contactIndex, newWorldContactKineRef, contact.contactWrenchVector_, true);
      }

      invincibilityIter_ = 0;
      lastBackupIter_ = logger.t() / ctl.timeStep;
    }

    observer_.nanDetected_ = false;

    invincibilityIter_++;
    // when we reach the end of the invicibility frame, we give again the pose of the floating base corrected by the
    // tilt estimator to the Kinetics Observer. Thus we don't get affected by the drift that occured during the
    // convergence. But this time we don't reset the covariances.
    if(invincibilityIter_ == invincibilityFrame_)
    {
      update(inputRobot);
      inputRobot.forwardKinematics();
      so::kine::Kinematics fbFb; // "Zero" Kinematics
      fbFb.setZero(so::kine::Kinematics::Flags::all);
      so::kine::Kinematics newWorldCentroidKine;
      newWorldCentroidKine.position = inputRobot.com();
      // the orientation of the centroid frame is the one of the floating base
      newWorldCentroidKine.orientation = mcko_K_0_fb.orientation;

      newWorldCentroidKine.linVel = inputRobot.comVelocity();
      newWorldCentroidKine.angVel = mcko_K_0_fb.angVel();

      observer_.setWorldCentroidStateKinematics(newWorldCentroidKine, false);

      for(const int & contactIndex : contactsManager_.contactsFound())
      {
        KoContactWithSensor contact = contactsManager_.contactWithSensor(contactIndex);

        // Update of the force measurements (the contribution of the gravity changed)
        const mc_rbdyn::ForceSensor & forceSensor = robot.forceSensor(contact.forceSensorName());

        // the tilt of the robot changed so the contribution of the gravity to the measurements changed too
        if(contactsDetectionFromThreshold_)
        {
          updateContactForceMeasurement(contact, forceSensor.wrenchWithoutGravity(inputRobot));
        }
        else // the kinematics of the contact are the ones of the associated surface
        {
          updateContactForceMeasurement(contact, contact.surfaceSensorKine_,
                                        forceSensor.wrenchWithoutGravity(inputRobot));
        }

        so::kine::Kinematics newWorldContactKineRef;

        getOdometryWorldContactReference(ctl, contact, newWorldContactKineRef);

        observer_.setStateContact(contactIndex, newWorldContactKineRef, contact.contactWrenchVector_, false);
      }
    }
  }

  if(withDebugLogs_)
  {
    /* Update of the logged variables */
    correctedMeasurements_ = observer_.getEKF().getSimulatedMeasurement(observer_.getEKF().getCurrentTime());
    globalCentroidKinematics_ = observer_.getGlobalCentroidKinematics();
  }

  /* Update of the visual representation (only a visual feature) of the observed robot */
  my_robots_->robot().mbc().q = ctl.realRobot().mbc().q;

  /* Update of the observed robot */
  update(my_robots_->robot());

  return true;
}

so::kine::Kinematics MCKineticsObserver::updateContactsPoseFromFb(
    const stateObservation::kine::Kinematics & oldFbWorld,
    const stateObservation::kine::Kinematics & oldWorldContactRef,
    const stateObservation::kine::Kinematics & newWorldFbKine)
{
  so::kine::Kinematics newWorldContactKine;

  so::kine::Kinematics fbContactRefKine = oldFbWorld * oldWorldContactRef;

  if(contactsDetectionFromThreshold_)
  {
    newWorldContactKine = newWorldFbKine * fbContactRefKine;
  }
  else // the kinematics of the contact are the ones of the associated surface
  {
    newWorldContactKine = newWorldFbKine * fbContactRefKine;
  }

  return newWorldContactKine;
}

///////////////////////////////////////////////////////////////////////
/// -------------------------Called functions--------------------------
///////////////////////////////////////////////////////////////////////

void MCKineticsObserver::initObserverStateVector(const mc_rbdyn::Robot & robot)
{
  so::kine::Orientation initOrientation;
  initOrientation.setZeroRotation<so::Quaternion>();
  Eigen::VectorXd initStateVector;
  initStateVector = Eigen::VectorXd::Zero(observer_.getStateSize());

  initStateVector.segment<int(observer_.sizePos)>(observer_.posIndex()) = robot.com();
  initStateVector.segment<int(observer_.sizeOri)>(observer_.oriIndex()) = initOrientation.toVector4();
  initStateVector.segment<int(observer_.sizeLinVel)>(observer_.linVelIndex()) = robot.comVelocity();

  observer_.setInitWorldCentroidStateVector(initStateVector);
}

void MCKineticsObserver::update(mc_control::MCController & ctl) // this function is called by the pipeline if the
                                                                // update is set to true in the configuration file
{
  auto & realRobot = ctl.realRobot(robot_);
  update(realRobot);
  realRobot.forwardKinematics();
}

// used only to update the visual representation of the estimated robot
void MCKineticsObserver::update(mc_rbdyn::Robot & robot)
{
  robot.posW(X_0_fb_);
  robot.velW(v_fb_0_.vector());
}

void MCKineticsObserver::inputAdditionalWrench(const mc_rbdyn::Robot & inputRobot, const mc_rbdyn::Robot & measRobot)
{
  additionalUserResultingForce_.setZero();
  additionalUserResultingMoment_.setZero();

  for(auto & contactWithSensor : contactsManager_.contactsWithSensors())
  {

    KoContactWithSensor & contact = contactWithSensor.second;
    const std::string & fsName = contact.forceSensorName();

    if(!contact.isSet_
       && contact.sensorEnabled_) // if the contact is not set but we use the force sensor measurements,
                                  // then we give the measured force as an input to the Kinetics Observer
    {
      sva::ForceVecd measuredWrench = measRobot.forceSensor(fsName).worldWrenchWithoutGravity(inputRobot);
      additionalUserResultingForce_ += measuredWrench.force();
      additionalUserResultingMoment_ += measuredWrench.moment();
    }

    /*
    if(contact.isExternalWrench == true) // if a force sensor is not associated to a currently set contact,
    its measurement is given as an input external wrench
    {
      sva::ForceVecd measuredWrench = measRobot.forceSensor(fsName).worldWrenchWithoutGravity(inputRobot);
      additionalUserResultingForce_ += measuredWrench.force();
      additionalUserResultingMoment_ += measuredWrench.moment();
    }
    */
  }
  observer_.setAdditionalWrench(additionalUserResultingForce_, additionalUserResultingMoment_);

  if(withDebugLogs_)
  {

    for(auto & contactWithSensor :
        contactsManager_.contactsWithSensors()) // if a force sensor is not associated to a contact, its
                                                // measurement is given as an input external wrench
    {
      KoContactWithSensor & contact = contactWithSensor.second;
      const std::string & fsName = contact.forceSensorName();
      so::Vector3 forceCentroid = so::Vector3::Zero();
      so::Vector3 torqueCentroid = so::Vector3::Zero();
      observer_.convertWrenchFromUserToCentroid(
          measRobot.forceSensor(fsName).worldWrenchWithoutGravity(inputRobot).force(),
          measRobot.forceSensor(fsName).worldWrenchWithoutGravity(inputRobot).moment(), forceCentroid, torqueCentroid);

      contact.wrenchInCentroid_.segment<3>(0) = forceCentroid;
      contact.wrenchInCentroid_.segment<3>(3) = torqueCentroid;
    }
  }
}

void MCKineticsObserver::updateIMUs(const mc_rbdyn::Robot & measRobot, const mc_rbdyn::Robot & inputRobot)
{
  unsigned i = 0;
  for(const auto & imu : IMUs_)
  {
    /** Position of accelerometer **/

    const sva::PTransformd & bodyImuPose = inputRobot.bodySensor(imu.name()).X_b_s();
    so::kine::Kinematics bodyImuKine = kinematicsTools::poseFromSva(
        bodyImuPose, so::kine::Kinematics::Flags::vels | so::kine::Kinematics::Flags::accs);

    so::kine::Kinematics worldBodyKine = kinematicsTools::kinematicsFromSva(
        inputRobot.mbc().bodyPosW[inputRobot.bodyIndexByName(imu.parentBody())],
        inputRobot.mbc().bodyVelW[inputRobot.bodyIndexByName(imu.parentBody())],
        inputRobot.mbc().bodyAccB[inputRobot.bodyIndexByName(imu.parentBody())], true, false);

    so::kine::Kinematics worldImuKine = worldBodyKine * bodyImuKine;
    const so::kine::Kinematics fbImuKine = worldImuKine;

    observer_.setIMU(measRobot.bodySensor().linearAcceleration(), measRobot.bodySensor().angularVelocity(),
                     acceleroSensorCovariance_, gyroSensorCovariance_, fbImuKine, mapIMUs_.getNumFromName(imu.name()));

    ++i;
  }
}

const measurements::ContactsManager<KoContactWithSensor, measurements::ContactWithoutSensor>::ContactsSet &
    MCKineticsObserver::findNewContacts(const mc_control::MCController & ctl)
{
  const auto & measRobot = ctl.robot(robot_);
  auto & inputRobot = my_robots_->robot("inputRobot");

  contactsManager_.findContacts(ctl, robot_);

  // force measurements from sensor that are not associated to a currently set contact are given to the Kinetics
  // Observer as inputs.
  inputAdditionalWrench(inputRobot, measRobot);
  // we start the observation once a contact has been detected. The estimation works only if the contact detection works.
  if(!contactsManager_.contactsFound().empty() && !simStarted_)
  {
    simStarted_ = true;
    initObserverStateVector(measRobot);
  }

  return contactsManager_.contactsFound(); // list of currently set contacts
}

const so::kine::Kinematics MCKineticsObserver::getContactWorldKinematics(KoContactWithSensor & contact,
                                                                         const mc_rbdyn::Robot & robot,
                                                                         const mc_rbdyn::ForceSensor & fs,
                                                                         const sva::ForceVecd & measuredWrench)
{
  /*
  Can be used with inputRobot, a virtual robot corresponding to the real robot whose floating base's frame is
  superimposed with the world frame. Getting kinematics associated to the inputRobot inside the world frame is the same
  as getting the same kinematics of the real robot inside the frame of its floating base, which is needed for the inputs
  of the Kinetics Observer. This allows to use the basic mc_rtc functions directly giving kinematics in the world frame
  and not do the conversion: initial frame -> world + world -> floating base as the latter is zero.
  */

  so::kine::Kinematics worldContactKine;

  const sva::PTransformd & bodyContactSensorPose = fs.X_p_f();
  so::kine::Kinematics bodyContactSensorKine =
      kinematicsTools::poseFromSva(bodyContactSensorPose, so::kine::Kinematics::Flags::vels);

  // kinematics of the sensor's parent body in the world
  so::kine::Kinematics worldBodyKine =
      kinematicsTools::poseAndVelFromSva(robot.mbc().bodyPosW[robot.bodyIndexByName(fs.parentBody())],
                                         robot.mbc().bodyVelW[robot.bodyIndexByName(fs.parentBody())], true);

  so::kine::Kinematics worldSensorKine = worldBodyKine * bodyContactSensorKine;

  if(contactsDetectionFromThreshold_)
  {
    // If the contact is detecting using thresholds, we will then consider the sensor frame as
    // the contact surface frame directly.
    worldContactKine = worldSensorKine;
    updateContactForceMeasurement(contact, measuredWrench);
  }
  else // the kinematics of the contacts are the ones of the surface, but we must transport the measured wrench
  {
    // pose of the surface in the world / floating base's frame
    sva::PTransformd worldSurfacePose = robot.surfacePose(contact.surfaceName());
    // Kinematics of the surface in the world / floating base's frame
    worldContactKine = kinematicsTools::poseFromSva(worldSurfacePose, so::kine::Kinematics::Flags::vels);

    contact.surfaceSensorKine_ = worldContactKine.getInverse() * worldSensorKine;
    // expressing the force measurement in the frame of the surface
    updateContactForceMeasurement(contact, contact.surfaceSensorKine_, measuredWrench);
  }

  return worldContactKine;
}

const so::kine::Kinematics MCKineticsObserver::getContactWorldKinematics(KoContactWithSensor & contact,
                                                                         const mc_rbdyn::Robot & robot,
                                                                         const mc_rbdyn::ForceSensor & fs)
{
  /*
  Can be used with inputRobot, a virtual robot corresponding to the real robot whose floating base's frame is
  superimposed with the world frame. Getting kinematics associated to the inputRobot inside the world frame is the same
  as getting the same kinematics of the real robot inside the frame of its floating base, which is needed for the inputs
  of the Kinetics Observer. This allows to use the basic mc_rtc functions directly giving kinematics in the world frame
  and not do the conversion: initial frame -> world + world -> floating base as the latter is zero.
  */

  so::kine::Kinematics worldContactKine;

  const sva::PTransformd & bodyContactSensorPose = fs.X_p_f();
  so::kine::Kinematics bodyContactSensorKine =
      kinematicsTools::poseFromSva(bodyContactSensorPose, so::kine::Kinematics::Flags::vels);

  // kinematics of the sensor's parent body in the world
  so::kine::Kinematics worldBodyKine =
      kinematicsTools::poseAndVelFromSva(robot.mbc().bodyPosW[robot.bodyIndexByName(fs.parentBody())],
                                         robot.mbc().bodyVelW[robot.bodyIndexByName(fs.parentBody())], true);

  so::kine::Kinematics worldSensorKine = worldBodyKine * bodyContactSensorKine;

  if(contactsDetectionFromThreshold_)
  {
    // If the contact is detecting using thresholds, we will then consider the sensor frame as
    // the contact surface frame directly.
    worldContactKine = worldSensorKine;
  }
  else // the kinematics of the contacts are the ones of the surface.
  {
    // pose of the surface in the world / floating base's frame
    sva::PTransformd worldSurfacePose = robot.surfacePose(contact.surfaceName());
    // Kinematics of the surface in the world / floating base's frame
    worldContactKine = kinematicsTools::poseFromSva(worldSurfacePose, so::kine::Kinematics::Flags::vels);
  }

  return worldContactKine;
}

void MCKineticsObserver::updateContactForceMeasurement(KoContactWithSensor & contact,
                                                       so::kine::Kinematics surfaceSensorKine,
                                                       const sva::ForceVecd & measuredWrench)
{
  // expressing the force measurement in the frame of the surface
  contact.contactWrenchVector_.segment<3>(0) = surfaceSensorKine.orientation * measuredWrench.force();

  // expressing the torque measurement in the frame of the surface
  contact.contactWrenchVector_.segment<3>(3) =
      surfaceSensorKine.orientation * measuredWrench.moment()
      + surfaceSensorKine.position().cross(contact.contactWrenchVector_.segment<3>(0));
}

void MCKineticsObserver::updateContactForceMeasurement(KoContactWithSensor & contact,
                                                       const sva::ForceVecd & measuredWrench)
{
  // If the contact is detecting using thresholds, we will then consider the sensor frame as
  // the contact surface frame directly.
  contact.contactWrenchVector_.segment<3>(0) = measuredWrench.force(); // retrieving the force measurement
  contact.contactWrenchVector_.segment<3>(3) = measuredWrench.moment(); // retrieving the torque measurement
}

void MCKineticsObserver::getOdometryWorldContactReference(const mc_control::MCController & ctl,
                                                          KoContactWithSensor & contact,
                                                          so::kine::Kinematics & worldContactKineRef)
{
  const auto & robot = ctl.robot(robot_);
  if(!contact.sensorEnabled_)
  {
    mc_rtc::log::info("The sensor is disabled but is required for the odometry. It will be used for the odometry "
                      "but not in the correction made by the Kinetics Observer.");
  }
  const so::Vector3 & contactForceMeas = contact.contactWrenchVector_.segment<3>(0); // retrieving the force measurement
  const so::Vector3 & contactTorqueMeas =
      contact.contactWrenchVector_.segment<3>(3); // retrieving the torque measurement
  // we get the kinematics of the contact in the real world from the ones of the centroid estimated by the Kinetics
  // Observer. These kinematics are not the reference kinematics of the contact as they take into account the
  // visco-elastic model of the contacts.

  const so::kine::Kinematics worldContactKine = observer_.getGlobalKinematicsOf(contact.fbContactKine_);

  // we get the reference position of the contact by removing the contribution of the visco-elastic model
  worldContactKineRef.position =
      worldContactKine.orientation.toMatrix3() * linStiffness_.inverse()
          * (contactForceMeas
             + worldContactKine.orientation.toMatrix3().transpose() * linDamping_ * worldContactKine.linVel())
      + worldContactKine.position();

  /* We get the reference orientation of the contact by removing the contribution of the visco-elastic model */
  // difference between the reference orientation and the real one, obtained from the visco-elastic model
  so::Vector3 flexRotDiff =
      -2 * worldContactKine.orientation.toMatrix3() * angStiffness_.inverse()
      * (contactTorqueMeas
         + worldContactKine.orientation.toMatrix3().transpose() * angDamping_ * worldContactKine.angVel());

  so::Vector3 flexRotAxis = flexRotDiff / flexRotDiff.norm();

  double diffNorm = flexRotDiff.norm() / 2;

  if(diffNorm > 1.0)
  {
    diffNorm = 1.0;
  }
  else if(diffNorm < -1.0)
  {
    diffNorm = -1.0;
  }

  double flexRotAngle = std::asin(diffNorm);

  // angle axis representation of the rotation due to the visco-elastic model
  Eigen::AngleAxisd flexRotAngleAxis(flexRotAngle, flexRotAxis);
  // matrix representation of the rotation due to the visco-elastic model
  so::Matrix3 flexRotMatrix = so::kine::Orientation(flexRotAngleAxis).toMatrix3();
  worldContactKineRef.orientation = so::Matrix3(flexRotMatrix.transpose() * worldContactKine.orientation.toMatrix3());

  if(withFlatOdometry_) // if true, the position odometry is made only along the x and y axis, the position along z
                        // is assumed to be the one of the control robot
  {
    // kinematics of the contact of the control robot in the world frame
    so::kine::Kinematics worldContactKineControl =
        getContactWorldKinematics(contact, robot, robot.forceSensor(contact.forceSensorName()));

    // the reference altitude of the contact is the one in the control robot
    worldContactKineRef.position()(2) = worldContactKineControl.position()(2);
  }
}

void MCKineticsObserver::updateContact(const mc_control::MCController & ctl,
                                       const int & contactIndex,
                                       mc_rtc::Logger & logger)
{
  /*
  Uses the inputRobot, a virtual robot corresponding to the real robot whose floating base's frame is superimposed with
  the world frame. Getting kinematics associated to the inputRobot inside the world frame is the same as getting the
  same kinematics of the real robot inside the frame of its floating base, which is needed for the inputs of the
  Kinetics Observer. This allows to use the basic mc_rtc functions directly giving kinematics in the world frame and not
  do the conversion: initial frame -> world + world -> floating base as the latter is zero.
  */
  auto & inputRobot = my_robots_->robot("inputRobot");

  const auto & robot = ctl.robot(robot_);
  KoContactWithSensor & contact = contactsManager_.contactWithSensor(contactIndex);
  const mc_rbdyn::ForceSensor & forceSensor = robot.forceSensor(contact.forceSensorName());

  sva::ForceVecd measuredWrench = forceSensor.wrenchWithoutGravity(inputRobot);

  // when used on input robot, returns the kinematics of the contact in the frame of the floating base
  contact.fbContactKine_ = getContactWorldKinematics(contact, inputRobot, forceSensor, measuredWrench);

  contact.fbContactKine_ = contact.fbContactKine_;

  if(contact.wasAlreadySet_) // checks if the contact already exists, if yes, it is updated
  {
    if(contact.sensorEnabled_) // the force sensor attached to the contact is used in the correction by the
                               // Kinetics Observer.
    {
      observer_.updateContactWithWrenchSensor(contact.contactWrenchVector_, contactSensorCovariance_,
                                              contact.fbContactKine_, contactIndex);
    }
    else
    {
      observer_.updateContactWithNoSensor(contact.fbContactKine_, contactIndex);
    }

    if(withDebugLogs_)
    {
      if(contact.sensorEnabled_ && !contact.sensorWasEnabled_)
      {
        addContactMeasurementsLogEntries(logger, contactIndex);
        contact.sensorWasEnabled_ = true;
      }
      if(!contact.sensorEnabled_ && contact.sensorWasEnabled_)
      {
        removeContactMeasurementsLogEntries(logger, contactIndex);
        contact.sensorWasEnabled_ = false;
      }
    }
  }
  else // the contact still doesn't exist, it is added to the observer
  {
    // reference of the contact in the world / floating base of the input robot
    so::kine::Kinematics worldContactKineRef;

    if(withOdometry_) // the Kinetics Observer performs odometry. The estimated state is used to provide
                      // the new contacts references.
    {
      getOdometryWorldContactReference(ctl, contact, worldContactKineRef);
    }
    else // we don't perform odometry, the reference pose of the contact is its pose in the control robot
    {
      worldContactKineRef = getContactWorldKinematics(contact, robot, forceSensor);
    }

    if(observer_.getNumberOfSetContacts() > 0) // The initial covariance on the pose of the contact depending on whether
                                               // another contact is already set or not
    {
      observer_.addContact(worldContactKineRef, contactInitCovarianceNewContacts_, contactProcessCovariance_,
                           contactIndex, linStiffness_, linDamping_, angStiffness_, angDamping_);
    }
    else
    {
      observer_.addContact(worldContactKineRef, contactInitCovarianceFirstContacts_, contactProcessCovariance_,
                           contactIndex, linStiffness_, linDamping_, angStiffness_, angDamping_);
    }
    if(contact.sensorEnabled_) // checks if the sensor is used in the correction of the Kinetics Observer
                               // or not
    {
      // we update the measurements of the sensor and the input kinematics of the contact in the user /
      // floating base's frame
      observer_.updateContactWithWrenchSensor(contact.contactWrenchVector_, contactSensorCovariance_,
                                              contact.fbContactKine_, contactIndex);
    }
    else
    {
      // we update the input kinematics of the contact in the user / floating base's frame
      observer_.updateContactWithNoSensor(contact.fbContactKine_, contactIndex);
    }

    if(withDebugLogs_)
    {
      addContactLogEntries(logger, contactIndex);
    }
  }
}

void MCKineticsObserver::updateContacts(
    const mc_control::MCController & ctl,
    const measurements::ContactsManager<KoContactWithSensor, measurements::ContactWithoutSensor>::ContactsSet &
        updatedContactsIndexes,
    mc_rtc::Logger & logger)
{
  for(const auto & updatedContactIndex : updatedContactsIndexes)
  {
    updateContact(ctl, updatedContactIndex, logger);
  }
  // List of the contact that were set on last iteration but are not set anymore on the current one
  for(const int & removedContactIndex : contactsManager_.removedContacts())
  {
    observer_.removeContact(removedContactIndex);

    if(withDebugLogs_)
    {
      removeContactLogEntries(logger, removedContactIndex);
      removeContactMeasurementsLogEntries(logger, removedContactIndex);
    }
  }

  unsigned nbContacts = static_cast<unsigned>(updatedContactsIndexes.size());
  if(debug_)
  {
    mc_rtc::log::info("nbContacts = {}", nbContacts);
  }
}

void MCKineticsObserver::mass(double mass)
{
  mass_ = mass;
  observer_.setMass(mass);
}

///////////////////////////////////////////////////////////////////////
/// -------------------------------Logs--------------------------------
///////////////////////////////////////////////////////////////////////

void MCKineticsObserver::addToLogger(const mc_control::MCController &,
                                     mc_rtc::Logger & logger,
                                     const std::string & category)
{
  logger.addLogEntry(category + "_mcko_fb_posW", [this]() -> sva::PTransformd & { return X_0_fb_; });
  logger.addLogEntry(category + "_mcko_fb_velW", [this]() -> sva::MotionVecd & { return v_fb_0_; });
  logger.addLogEntry(category + "_mcko_fb_accW", [this]() -> sva::MotionVecd & { return a_fb_0_; });

  logger.addLogEntry(category + "_mcko_fb_yaw",
                     [this]() -> double { return -so::kine::rotationMatrixToYawAxisAgnostic(X_0_fb_.rotation()); });

  logger.addLogEntry(category + "_constants_mass", [this]() -> double { return observer_.getMass(); });

  logger.addLogEntry(category + "_constants_forceThreshold",
                     [this]() -> double { return mass_ * so::cst::gravityConstant * contactDetectionPropThreshold_; });
  logger.addLogEntry(category + "_debug_invincibilityFrame",
                     [this]() -> int { return invincibilityIter_ != 0 && invincibilityIter_ < invincibilityFrame_; });
}

void MCKineticsObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_posW");
  logger.removeLogEntry(category + "_velW");
  logger.removeLogEntry(category + "_mass");
  logger.removeLogEntry(category + "_flexStiffness");
  logger.removeLogEntry(category + "_flexDamping");
}

void MCKineticsObserver::addToGUI(const mc_control::MCController &,
                                  mc_rtc::gui::StateBuilder & gui,
                                  const std::vector<std::string> & category)
{
  using namespace mc_rtc::gui;
  // clang-format off
  gui.addElement(category,
    mc_state_observation::gui::make_input_element("Accel Covariance", acceleroSensorCovariance_(0,0)),
    mc_state_observation::gui::make_input_element("Force Covariance", contactSensorCovariance_(0,0)),
    mc_state_observation::gui::make_input_element("Gyro Covariance", gyroSensorCovariance_(0,0)));
  // clang-format on
}

void MCKineticsObserver::plotVariablesBeforeUpdate(const mc_control::MCController & ctl, mc_rtc::Logger & logger)
{
  /* Plots of the updated state */
  kinematicsTools::addToLogger(globalCentroidKinematics_, logger, category_ + "_globalWorldCentroidState");
  logger.addLogEntry(category_ + "_globalWorldCentroidState_positionW_",
                     [this]() -> Eigen::Vector3d { return globalCentroidKinematics_.position(); });
  logger.addLogEntry(category_ + "_globalWorldCentroidState_linVelW",
                     [this]() -> Eigen::Vector3d { return globalCentroidKinematics_.linVel(); });
  logger.addLogEntry(category_ + "_globalWorldCentroidState_linAccW",
                     [this]() -> Eigen::Vector3d { return globalCentroidKinematics_.linAcc(); });
  logger.addLogEntry(category_ + "_globalWorldCentroidState_oriW",
                     [this]() -> Eigen::Quaternion<double>
                     { return globalCentroidKinematics_.orientation.inverse().toQuaternion(); });
  logger.addLogEntry(category_ + "_globalWorldCentroidState_angVelW",
                     [this]() -> Eigen::Vector3d { return globalCentroidKinematics_.angVel(); });
  logger.addLogEntry(category_ + "_globalWorldCentroidState_angAccW",
                     [this]() -> Eigen::Vector3d { return globalCentroidKinematics_.angAcc(); });
  for(const auto & imu : IMUs_)
  {
    logger.addLogEntry(category_ + "_globalWorldCentroidState_gyroBias_" + imu.name(),
                       [this, imu]() -> Eigen::Vector3d
                       {
                         return observer_.getCurrentStateVector().segment<int(observer_.sizeGyroBias)>(
                             observer_.gyroBiasIndex(mapIMUs_.getNumFromName(imu.name())));
                       });
  }
  logger.addLogEntry(
      category_ + "_globalWorldCentroidState_extForceCentr",
      [this]() -> Eigen::Vector3d
      { return observer_.getCurrentStateVector().segment<int(observer_.sizeForce)>(observer_.unmodeledForceIndex()); });

  logger.addLogEntry(category_ + "_globalWorldCentroidState_extTorqueCentr",
                     [this]() -> Eigen::Vector3d {
                       return observer_.getCurrentStateVector().segment<int(observer_.sizeTorque)>(
                           observer_.unmodeledTorqueIndex());
                     });

  /* Inputs */
  logger.addLogEntry(category_ + "_inputs_additionalWrench_Force",
                     [this]() -> Eigen::Vector3d
                     { return observer_.getAdditionalWrench().segment<int(observer_.sizeForce)>(0); });
  logger.addLogEntry(category_ + "_inputs_additionalWrench_Torque",
                     [this]() -> Eigen::Vector3d {
                       return observer_.getAdditionalWrench().segment<int(observer_.sizeTorque)>(observer_.sizeForce);
                     });

  /* State covariances */
  logger.addLogEntry(category_ + "_stateCovariances_positionW_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block<int(observer_.sizePosTangent), int(observer_.sizePosTangent)>(
                               observer_.posIndexTangent(), observer_.posIndexTangent())
                           .diagonal();
                     });
  logger.addLogEntry(category_ + "_stateCovariances_orientationW_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block<int(observer_.sizeOriTangent), int(observer_.sizeOriTangent)>(
                               observer_.oriIndexTangent(), observer_.oriIndexTangent())
                           .diagonal();
                     });
  logger.addLogEntry(category_ + "_stateCovariances_linVelW_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block<int(observer_.sizeLinVelTangent), int(observer_.sizeLinVelTangent)>(
                               observer_.linVelIndexTangent(), observer_.linVelIndexTangent())
                           .diagonal();
                     });
  logger.addLogEntry(category_ + "_stateCovariances_angVelW_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block<int(observer_.sizeAngVelTangent), int(observer_.sizeAngVelTangent)>(
                               observer_.angVelIndexTangent(), observer_.angVelIndexTangent())
                           .diagonal();
                     });

  for(const auto & imu : IMUs_)
  {
    logger.addLogEntry(category_ + "_stateCovariances_gyroBias_" + imu.name(),
                       [this, imu]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF()
                             .getStateCovariance()
                             .block<int(observer_.sizeGyroBiasTangent), int(observer_.sizeGyroBiasTangent)>(
                                 observer_.gyroBiasIndexTangent(mapIMUs_.getNumFromName(imu.name())),
                                 observer_.gyroBiasIndexTangent(mapIMUs_.getNumFromName(imu.name())))
                             .diagonal();
                       });
  }

  logger.addLogEntry(category_ + "_stateCovariances_extForce_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block<int(observer_.sizeForceTangent), int(observer_.sizeForceTangent)>(
                               observer_.unmodeledForceIndexTangent(), observer_.unmodeledForceIndexTangent())
                           .diagonal();
                     });
  logger.addLogEntry(category_ + "_stateCovariances_extTorque_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block<int(observer_.sizeTorqueTangent), int(observer_.sizeTorqueTangent)>(
                               observer_.unmodeledTorqueIndexTangent(), observer_.unmodeledTorqueIndexTangent())
                           .diagonal();
                     });

  if(ctl.realRobot().hasBody("LeftFoot"))
  {
    logger.addLogEntry(category_ + "_realRobot_LeftFoot",
                       [&ctl]() { return ctl.realRobot().frame("LeftFoot").position(); });
  }

  if(ctl.realRobot().hasBody("RightFoot"))
  {
    logger.addLogEntry(category_ + "_realRobot_RightFoot",
                       [&ctl]() { return ctl.realRobot().frame("RightFoot").position(); });
  }

  if(ctl.realRobot().hasBody("LeftHand"))
  {
    logger.addLogEntry(category_ + "_realRobot_LeftHand",
                       [&ctl]() { return ctl.realRobot().frame("LeftHand").position(); });
  }
  if(ctl.realRobot().hasBody("RightHand"))
  {
    logger.addLogEntry(category_ + "_realRobot_RightHand",
                       [&ctl]() { return ctl.realRobot().frame("RightHand").position(); });
  }
  if(ctl.robot().hasBody("LeftFoot"))
  {
    logger.addLogEntry(category_ + "_ctlRobot_LeftFoot", [&ctl]() { return ctl.robot().frame("LeftFoot").position(); });
  }
  if(ctl.robot().hasBody("RightFoot"))
  {
    logger.addLogEntry(category_ + "_ctlRobot_RightFoot",
                       [&ctl]() { return ctl.robot().frame("RightFoot").position(); });
  }

  if(ctl.robot().hasBody("LeftHand"))
  {
    logger.addLogEntry(category_ + "_ctlRobot_LeftHand", [&ctl]() { return ctl.robot().frame("LeftHand").position(); });
  }

  if(ctl.robot().hasBody("RightHand"))
  {
    logger.addLogEntry(category_ + "_ctlRobot_RightHand",
                       [&ctl]() { return ctl.robot().frame("RightHand").position(); });
  }
}

void MCKineticsObserver::plotVariablesAfterUpdate(mc_rtc::Logger & logger)
{
  /* Plots of the inputs */

  logger.addLogEntry(category_ + "_inputs_angularMomentum",
                     [this]() -> Eigen::Vector3d { return observer_.getAngularMomentum()(); });
  logger.addLogEntry(category_ + "_inputs_angularMomentumDot",
                     [this]() -> Eigen::Vector3d { return observer_.getAngularMomentumDot()(); });
  logger.addLogEntry(category_ + "_inputs_com", [this]() -> Eigen::Vector3d { return observer_.getCenterOfMass()(); });
  logger.addLogEntry(category_ + "_inputs_comDot",
                     [this]() -> Eigen::Vector3d { return observer_.getCenterOfMassDot()(); });
  logger.addLogEntry(category_ + "_inputs_comDotDot",
                     [this]() -> Eigen::Vector3d { return observer_.getCenterOfMassDotDot()(); });
  logger.addLogEntry(category_ + "_inputs_inertiaMatrix",
                     [this]() -> Eigen::Vector6d
                     {
                       so::Vector6 inertia;
                       inertia.segment<3>(0) = observer_.getInertiaMatrix()().diagonal();
                       inertia.segment<2>(3) = observer_.getInertiaMatrix()().block<1, 2>(0, 1);
                       inertia(5) = observer_.getInertiaMatrix()()(1, 2);
                       return inertia;
                     });

  logger.addLogEntry(category_ + "_inputs_inertiaMatrixDot",
                     [this]() -> Eigen::Vector6d
                     {
                       so::Vector6 inertiaDot;
                       inertiaDot.segment<3>(0) = observer_.getInertiaMatrixDot()().diagonal();
                       inertiaDot.segment<2>(3) = observer_.getInertiaMatrixDot()().block<1, 2>(0, 1);
                       inertiaDot(5) = observer_.getInertiaMatrixDot()()(1, 2);
                       return inertiaDot;
                     });

  /* Plots of the measurements */
  {
    for(const auto & imu : IMUs_)
    {
      logger.addLogEntry(category_ + "_measurements_gyro_" + imu.name() + "_measured",
                         [this, imu]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getLastMeasurement().segment<int(observer_.sizeGyroBias)>(
                               observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name()))
                               + observer_.sizeAcceleroSignal);
                         });
      logger.addLogEntry(category_ + "_measurements_gyro_" + imu.name() + "_predicted",
                         [this, imu]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getLastPredictedMeasurement().segment<int(observer_.sizeGyroBias)>(
                               observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name()))
                               + observer_.sizeAcceleroSignal);
                         });
      logger.addLogEntry(category_ + "_measurements_gyro_" + imu.name() + "_corrected",
                         [this, imu]() -> Eigen::Vector3d
                         {
                           return correctedMeasurements_.segment<int(observer_.sizeGyroBias)>(
                               observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name()))
                               + observer_.sizeAcceleroSignal);
                         });

      logger.addLogEntry(category_ + "_measurements_accelerometer_" + imu.name() + "_measured",
                         [this, imu]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getLastMeasurement().segment<int(observer_.sizeAcceleroSignal)>(
                               observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name())));
                         });
      logger.addLogEntry(
          category_ + "_measurements_accelerometer_" + imu.name() + "_predicted",
          [this, imu]() -> Eigen::Vector3d
          {
            return observer_.getEKF().getLastPredictedMeasurement().segment<int(observer_.sizeAcceleroSignal)>(
                observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name())));
          });
      logger.addLogEntry(category_ + "_measurements_accelerometer_" + imu.name() + "_corrected",
                         [this, imu]() -> Eigen::Vector3d
                         {
                           return correctedMeasurements_.segment<int(observer_.sizeAcceleroSignal)>(
                               observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name())));
                         });
    }
  }

  /* Plots of the innovation */
  logger.addLogEntry(category_ + "_innovation_positionW_",
                     [this]() -> Eigen::Vector3d {
                       return observer_.getEKF().getInnovation().segment<int(observer_.sizePosTangent)>(
                           observer_.posIndexTangent());
                     });
  logger.addLogEntry(category_ + "_innovation_linVelW_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getInnovation().segment<int(observer_.sizeLinVelTangent)>(
                           observer_.linVelIndexTangent());
                     });
  logger.addLogEntry(category_ + "_innovation_oriW_",
                     [this]() -> Eigen::Vector3d {
                       return observer_.getEKF().getInnovation().segment<int(observer_.sizeOriTangent)>(
                           observer_.oriIndexTangent());
                     });
  logger.addLogEntry(category_ + "_innovation_angVelW_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getInnovation().segment<int(observer_.sizeAngVelTangent)>(
                           observer_.angVelIndexTangent());
                     });
  for(const auto & imu : IMUs_)
  {
    logger.addLogEntry(category_ + "_innovation_gyroBias_" + imu.name(),
                       [this, imu]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getInnovation().segment<int(observer_.sizeGyroBias)>(
                             observer_.gyroBiasIndexTangent(mapIMUs_.getNumFromName(imu.name())));
                       });
  }
  logger.addLogEntry(category_ + "_innovation_unmodeledForce_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getInnovation().segment<int(observer_.sizeForceTangent)>(
                           observer_.unmodeledForceIndexTangent());
                     });
  logger.addLogEntry(category_ + "_innovation_unmodeledTorque_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getInnovation().segment<int(observer_.sizeTorqueTangent)>(
                           observer_.unmodeledTorqueIndexTangent());
                     });

  logger.addLogEntry(category_ + "_debug_worldInputRobotKine_position",
                     [this]() -> Eigen::Vector3d { return my_robots_->robot("inputRobot").posW().translation(); });
  logger.addLogEntry(category_ + "_debug_worldInputRobotKine_orientation",
                     [this]() -> Eigen::Quaternion<double>
                     {
                       return so::kine::Orientation(so::Matrix3(my_robots_->robot("inputRobot").posW().rotation()))
                           .inverse()
                           .toQuaternion();
                     });
  logger.addLogEntry(category_ + "_debug_worldInputRobotKine_linVel",
                     [this]() -> Eigen::Vector3d { return my_robots_->robot("inputRobot").velW().linear(); });
  logger.addLogEntry(category_ + "_debug_worldInputRobotKine_angVel",
                     [this]() -> Eigen::Vector3d { return my_robots_->robot("inputRobot").velW().angular(); });
  logger.addLogEntry(category_ + "_debug_worldInputRobotKine_linAcc",
                     [this]() -> Eigen::Vector3d { return my_robots_->robot("inputRobot").accW().linear(); });
  logger.addLogEntry(category_ + "_debug_worldInputRobotKine_angAcc",
                     [this]() -> Eigen::Vector3d { return my_robots_->robot("inputRobot").accW().angular(); });

  for(auto & contactWithSensor : contactsManager_.contactsWithSensors())
  {
    const measurements::ContactWithSensor & contact = contactWithSensor.second;
    logger.addLogEntry(category_ + "_debug_wrenchesInCentroid_" + contact.getName() + "_force",
                       [this, contact]() -> Eigen::Vector3d { return contact.wrenchInCentroid_.segment<3>(0); });
    logger.addLogEntry(category_ + "_debug_wrenchesInCentroid_" + contact.getName() + "_torque",
                       [this, contact]() -> Eigen::Vector3d { return contact.wrenchInCentroid_.segment<3>(3); });
    logger.addLogEntry(category_ + "_debug_wrenchesInCentroid_" + contact.getName() + "_forceWithUnmodeled",
                       [this, contact]() -> Eigen::Vector3d
                       {
                         return observer_.getCurrentStateVector().segment<int(observer_.sizeForce)>(
                                    observer_.unmodeledForceIndex())
                                + contact.wrenchInCentroid_.segment<3>(0);
                       });
    logger.addLogEntry(category_ + "_debug_wrenchesInCentroid_" + contact.getName() + "_torqueWithUnmodeled",
                       [this, contact]() -> Eigen::Vector3d
                       {
                         return observer_.getCurrentStateVector().segment<int(observer_.sizeTorque)>(
                                    observer_.unmodeledTorqueIndex())
                                + contact.wrenchInCentroid_.segment<3>(3);
                       });
  }

  for(const auto & imu : IMUs_)
  {
    logger.addLogEntry(category_ + "_debug_gyroBias_" + imu.name(),
                       [this, imu]() -> Eigen::Vector3d { return mapIMUs_(imu.name()).gyroBias; });
  }
}

void MCKineticsObserver::addContactLogEntries(mc_rtc::Logger & logger, const int & contactIndex)
{
  const std::string & contactName = contactsManager_.mapContacts_.getNameFromNum(contactIndex);
  if(observer_.getContactIsSetByNum(contactIndex))
  {
    logger.addLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_position",
                       [this, contactIndex]() -> Eigen::Vector3d {
                         return observer_.getCurrentStateVector().segment<int(observer_.sizePos)>(
                             observer_.contactPosIndex(contactIndex));
                       });
    logger.addLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_orientation",
                       [this, contactIndex]() -> Eigen::Quaternion<double>
                       {
                         so::kine::Orientation ori;
                         return ori
                             .fromVector4(observer_.getCurrentStateVector().segment<int(observer_.sizeOri)>(
                                 observer_.contactOriIndex(contactIndex)))
                             .inverse()
                             .toQuaternion();
                       });
    logger.addLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_orientation_RollPitchYaw",
                       [this, contactIndex]() -> so::Vector3
                       {
                         so::kine::Orientation ori;
                         return so::kine::rotationMatrixToRollPitchYaw(
                             ori.fromVector4(observer_.getCurrentStateVector().segment<int(observer_.sizeOri)>(
                                                 observer_.contactOriIndex(contactIndex)))
                                 .toMatrix3());
                       });
    logger.addLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_forces",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getCurrentStateVector().segment<int(observer_.sizeForce)>(
                             observer_.contactForceIndex(contactIndex));
                       });
    logger.addLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_torques",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return globalCentroidKinematics_.orientation.toMatrix3()
                                * observer_.getCurrentStateVector().segment<int(observer_.sizeTorque)>(
                                    observer_.contactTorqueIndex(contactIndex));
                       });
    logger.addLogEntry(category_ + "_stateCovariances_contact_" + contactName + "_position_",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF()
                             .getStateCovariance()
                             .block<int(observer_.sizePosTangent), int(observer_.sizePosTangent)>(
                                 observer_.contactPosIndexTangent(contactIndex),
                                 observer_.contactPosIndexTangent(contactIndex))
                             .diagonal();
                       });
    logger.addLogEntry(category_ + "_stateCovariances_contact_" + contactName + "_orientation_",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF()
                             .getStateCovariance()
                             .block<int(observer_.sizeOriTangent), int(observer_.sizeOriTangent)>(
                                 observer_.contactOriIndexTangent(contactIndex),
                                 observer_.contactOriIndexTangent(contactIndex))
                             .diagonal();
                       });
    logger.addLogEntry(category_ + "_stateCovariances_contact_" + contactName + "_Force_",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF()
                             .getStateCovariance()
                             .block<int(observer_.sizeForceTangent), int(observer_.sizeForceTangent)>(
                                 observer_.contactForceIndexTangent(contactIndex),
                                 observer_.contactForceIndexTangent(contactIndex))
                             .diagonal();
                       });
    logger.addLogEntry(category_ + "_stateCovariances_contact_" + contactName + "_Torque_",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF()
                             .getStateCovariance()
                             .block<int(observer_.sizeTorqueTangent), int(observer_.sizeTorqueTangent)>(
                                 observer_.contactTorqueIndexTangent(contactIndex),
                                 observer_.contactTorqueIndexTangent(contactIndex))
                             .diagonal();
                       });

    logger.addLogEntry(category_ + "_innovation_contact_" + contactName + "_position",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getInnovation().segment<int(observer_.sizePos)>(
                             observer_.contactPosIndexTangent(contactIndex));
                       });
    logger.addLogEntry(category_ + "_innovation_contact_" + contactName + "_orientation",
                       [this, contactIndex]() -> Eigen::Quaternion<double>
                       {
                         so::kine::Orientation ori;
                         return ori
                             .fromVector4(observer_.getEKF().getInnovation().segment<int(observer_.sizeOri)>(
                                 observer_.contactOriIndexTangent(contactIndex)))
                             .inverse()
                             .toQuaternion();
                       });
    logger.addLogEntry(category_ + "_innovation_contact_" + contactName + "_forces",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getInnovation().segment<int(observer_.sizeForce)>(
                             observer_.contactForceIndexTangent(contactIndex));
                       });
    logger.addLogEntry(category_ + "_innovation_contact_" + contactName + "_torques",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getInnovation().segment<int(observer_.sizeTorque)>(
                             observer_.contactTorqueIndexTangent(contactIndex));
                       });

    logger.addLogEntry(category_ + "_debug_contactWrench_Centroid_" + contactName + "_force",
                       [this, contactIndex]() -> Eigen::Vector3d {
                         return observer_.getCentroidContactWrench(contactIndex).segment<int(observer_.sizeForce)>(0);
                       });

    logger.addLogEntry(category_ + "_debug_contactWrench_Centroid_" + contactName + "_torque",
                       [this, contactIndex]() -> Eigen::Vector3d {
                         return observer_.getCentroidContactWrench(contactIndex).segment<int(observer_.sizeTorque)>(3);
                       });

    logger.addLogEntry(category_ + "_debug_contactPose_" + contactName + "_inputCentroidContactKine_position",
                       [this, contactIndex]() -> Eigen::Vector3d
                       { return observer_.getCentroidContactInputPose(contactIndex).position(); });

    logger.addLogEntry(
        category_ + "_debug_contactPose_" + contactName + "_inputCentroidContactKine_orientation",
        [this, contactIndex]() -> Eigen::Quaternion<double>
        { return observer_.getCentroidContactInputPose(contactIndex).orientation.inverse().toQuaternion(); });

    logger.addLogEntry(category_ + "_debug_contactPose_" + contactName + "_worldContactPoseFromCentroid_position",
                       [this, contactIndex]() -> Eigen::Vector3d
                       { return observer_.getWorldContactPoseFromCentroid(contactIndex).position(); });

    logger.addLogEntry(
        category_ + "_debug_contactPose_" + contactName + "_worldContactPoseFromCentroid_orientation",
        [this, contactIndex]() -> Eigen::Quaternion<double>
        { return observer_.getWorldContactPoseFromCentroid(contactIndex).orientation.inverse().toQuaternion(); });

    logger.addLogEntry(category_ + "_debug_contactPose_" + contactName + "_inputUserContactKine_position",
                       [this, contactIndex]() -> Eigen::Vector3d
                       { return observer_.getUserContactInputPose(contactIndex).position(); });

    logger.addLogEntry(category_ + "_debug_contactPose_" + contactName + "_inputUserContactKine_orientation",
                       [this, contactIndex]() -> Eigen::Quaternion<double> {
                         return observer_.getUserContactInputPose(contactIndex).orientation.inverse().toQuaternion();
                       });
    logger.addLogEntry(category_ + "_debug_contactState_isSet_" + contactName,
                       [this, contactIndex]() -> int
                       { return int(contactsManager_.contactWithSensor(contactIndex).isSet_); });
  }
}

void MCKineticsObserver::addContactMeasurementsLogEntries(mc_rtc::Logger & logger, const int & contactIndex)
{
  const std::string & contactName = contactsManager_.mapContacts_.getNameFromNum(contactIndex);
  if(observer_.getContactIsSetByNum(contactIndex))
  {
    logger.addLogEntry(category_ + "_measurements_contacts_force_" + contactName + "_measured",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getLastMeasurement().segment<int(observer_.sizeForce)>(
                             observer_.getContactMeasIndexByNum(contactIndex));
                       });
    logger.addLogEntry(category_ + "_measurements_contacts_force_" + contactName + "_predicted",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getLastPredictedMeasurement().segment<int(observer_.sizeForce)>(
                             observer_.getContactMeasIndexByNum(contactIndex));
                       });
    logger.addLogEntry(category_ + "_measurements_contacts_force_" + contactName + "_corrected",
                       [this, contactIndex]() -> Eigen::Vector3d {
                         return correctedMeasurements_.segment<int(observer_.sizeForce)>(
                             observer_.getContactMeasIndexByNum(contactIndex));
                       });

    logger.addLogEntry(category_ + "_measurements_contacts_torque_" + contactName + "_measured",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getLastMeasurement().segment<int(observer_.sizeTorque)>(
                             observer_.getContactMeasIndexByNum(contactIndex) + observer_.sizeForce);
                       });
    logger.addLogEntry(category_ + "_measurements_contacts_torque_" + contactName + "_predicted",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getLastPredictedMeasurement().segment<int(observer_.sizeTorque)>(
                             observer_.getContactMeasIndexByNum(contactIndex) + observer_.sizeForce);
                       });
    logger.addLogEntry(category_ + "_measurements_contacts_torque_" + contactName + "_corrected",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return correctedMeasurements_.segment<int(observer_.sizeTorque)>(
                             observer_.getContactMeasIndexByNum(contactIndex) + observer_.sizeForce);
                       });
  }
}

void MCKineticsObserver::removeContactLogEntries(mc_rtc::Logger & logger, const int & contactIndex)
{
  const std::string & contactName = contactsManager_.mapContacts_.getNameFromNum(contactIndex);
  logger.removeLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_position");
  logger.removeLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_position");
  logger.removeLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_orientation");
  logger.removeLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_orientation_RollPitchYaw");
  logger.removeLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_forces");
  logger.removeLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_torques");
  logger.removeLogEntry(category_ + "_stateCovariances_contact_" + contactName + "_position_");
  logger.removeLogEntry(category_ + "_stateCovariances_contact_" + contactName + "_orientation_");
  logger.removeLogEntry(category_ + "_stateCovariances_contact_" + contactName + "_Force_");
  logger.removeLogEntry(category_ + "_stateCovariances_contact_" + contactName + "_Torque_");

  logger.removeLogEntry(category_ + "_predictedGlobalCentroidKinematics_contact_" + contactName + "_position");
  logger.removeLogEntry(category_ + "_predictedGlobalCentroidKinematics_contact_" + contactName + "_orientation");
  logger.removeLogEntry(category_ + "_predictedGlobalCentroidKinematics_contact_" + contactName + "_forces");
  logger.removeLogEntry(category_ + "_predictedGlobalCentroidKinematics_contact_" + contactName + "_torques");

  logger.removeLogEntry(category_ + "_innovation_contact_" + contactName + "_position");
  logger.removeLogEntry(category_ + "_innovation_contact_" + contactName + "_orientation");
  logger.removeLogEntry(category_ + "_innovation_contact_" + contactName + "_forces");
  logger.removeLogEntry(category_ + "_innovation_contact_" + contactName + "_torques");

  logger.removeLogEntry(category_ + "_debug_contactWrench_World_" + contactName + "_force");

  logger.removeLogEntry(category_ + "_debug_contactWrench_World_" + contactName + "_torque");

  logger.removeLogEntry(category_ + "_debug_contactWrench_Centroid_" + contactName + "_force");

  logger.removeLogEntry(category_ + "_debug_contactWrench_Centroid_" + contactName + "_torque");

  logger.removeLogEntry(category_ + "_debug_contactPose_" + contactName + "_inputWorldRef_position");

  logger.removeLogEntry(category_ + "_debug_contactPose_" + contactName + "_inputWorldRef_orientation");

  logger.removeLogEntry(category_ + "_debug_contactPose_" + contactName + "_inputCentroidContactKine_position");

  logger.removeLogEntry(category_ + "_debug_contactPose_" + contactName + "_inputCentroidContactKine_orientation");

  logger.removeLogEntry(category_ + "_debug_contactPose_" + contactName + "_worldContactPoseFromCentroid_position");

  logger.removeLogEntry(category_ + "_debug_contactPose_" + contactName + "_worldContactPoseFromCentroid_orientation");

  logger.removeLogEntry(category_ + "_debug_contactPose_" + contactName + "_inputUserContactKine_position");

  logger.removeLogEntry(category_ + "_debug_contactPose_" + contactName + "_inputUserContactKine_orientation");
  logger.removeLogEntry(category_ + "_debug_contactState_isSet_" + contactName);
  // logger.removeLogEntry(category_ + "_debug_zmp_" + contactName);
}

void MCKineticsObserver::removeContactMeasurementsLogEntries(mc_rtc::Logger & logger, const int & contactIndex)
{
  const std::string & contactName = contactsManager_.mapContacts_.getNameFromNum(contactIndex);
  logger.removeLogEntry(category_ + "_measurements_contacts_force_" + contactName + "_measured");
  logger.removeLogEntry(category_ + "_measurements_contacts_force_" + contactName + "_predicted");
  logger.removeLogEntry(category_ + "_measurements_contacts_force_" + contactName + "_corrected");

  logger.removeLogEntry(category_ + "_measurements_contacts_torque_" + contactName + "_measured");
  logger.removeLogEntry(category_ + "_measurements_contacts_torque_" + contactName + "_predicted");
  logger.removeLogEntry(category_ + "_measurements_contacts_torque_" + contactName + "_corrected");
}

} // namespace mc_state_observation

EXPORT_OBSERVER_MODULE("MCKineticsObserver", mc_state_observation::MCKineticsObserver)
