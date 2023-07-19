#include <mc_state_observation/observersTools/kinematicsTools.h>

#include <mc_state_observation/observersTools/leggedOdometryTools.h>

namespace so = stateObservation;

namespace mc_state_observation
{
namespace leggedOdometry
{

///////////////////////////////////////////////////////////////////////
/// -------------------------Legged Odometry---------------------------
///////////////////////////////////////////////////////////////////////

void LeggedOdometryManager::init(const mc_control::MCController & ctl,
                                 const std::string robotName,
                                 const std::string & odometryName,
                                 const bool odometry6d,
                                 const bool withYawEstimation,
                                 const std::string contactsDetection,
                                 std::vector<std::string> surfacesForContactDetection,
                                 std::vector<std::string> contactsSensorDisabledInit,
                                 const double contactDetectionThreshold)
{
  robotName_ = robotName;
  odometry6d_ = odometry6d;
  withYawEstimation_ = withYawEstimation;
  odometryName_ = odometryName;
  const auto & realRobot = ctl.realRobot(robotName);
  odometryRobot_ = mc_rbdyn::Robots::make();
  odometryRobot_->robotCopy(realRobot, "odometryRobot");

  worldAnchorFramePose_ = kinematicsTools::poseFromSva(
      ctl.datastore().call<sva::PTransformd>("KinematicAnchorFrame::" + ctl.robot(robotName).name(),
                                             ctl.realRobot(robotName)),
      so::kine::Kinematics::Flags::pose);

  fbPose_.translation() = realRobot.posW().translation();
  fbPose_.rotation() = realRobot.posW().rotation();
  contactsManager_.init(ctl, robotName, odometryName_, contactsDetection, surfacesForContactDetection,
                        contactsSensorDisabledInit, contactDetectionThreshold);
}

void LeggedOdometryManager::init(const mc_control::MCController & ctl,
                                 const std::string robotName,
                                 const std::string & odometryName,
                                 const bool odometry6d,
                                 const bool withYawEstimation,
                                 const std::string contactsDetection,
                                 std::vector<std::string> contactsSensorDisabledInit,
                                 const double contactDetectionThreshold)
{
  robotName_ = robotName;
  odometry6d_ = odometry6d;
  withYawEstimation_ = withYawEstimation;
  odometryName_ = odometryName;
  const auto & realRobot = ctl.realRobot(robotName);
  odometryRobot_ = mc_rbdyn::Robots::make();
  odometryRobot_->robotCopy(realRobot, "odometryRobot");
  fbPose_.translation() = realRobot.posW().translation();
  fbPose_.rotation() = realRobot.posW().rotation();
  odometryRobot().posW(fbPose_);

  worldAnchorFramePose_ = kinematicsTools::poseFromSva(
      ctl.datastore().call<sva::PTransformd>("KinematicAnchorFrame::" + ctl.robot(robotName).name(),
                                             ctl.realRobot(robotName)),
      so::kine::Kinematics::Flags::pose);
  if(contactsDetection == "fromThreshold")
  {
    detectionFromThreshold_ = true;
  }
  contactsManager_.init(ctl, robotName, odometryName_, contactsDetection, contactsSensorDisabledInit,
                        contactDetectionThreshold);
}

void LeggedOdometryManager::run(const mc_control::MCController & ctl,
                                mc_rtc::Logger & logger,
                                sva::PTransformd & pose,
                                sva::MotionVecd & vels,
                                sva::MotionVecd & accs)
{
  const auto & realRobot = ctl.realRobot(robotName_);

  // copies the updated joints configuration from the real robot.
  odometryRobot().q() = realRobot.q();

  odometryRobot().posW(fbPose_);

  // we set the velocities and accelerations to zero as they will be compensated anyway as we compute the
  // successive poses in the local frame
  sva::MotionVecd zeroMotion;
  zeroMotion.linear() = so::Vector3::Zero();
  zeroMotion.angular() = so::Vector3::Zero();
  odometryRobot().velW(zeroMotion);
  odometryRobot().accW(zeroMotion);

  odometryRobot().forwardKinematics();

  // detects the contacts currently set with the environment
  contactsManager().findContacts(ctl, robotName_);
  // updates the contacts and the resulting floating base kinematics
  updateContacts(ctl, logger);
  // updates the floating base kinematics in the observer
  updateFbKinematics(ctl, pose, vels, accs);
}

void LeggedOdometryManager::run(const mc_control::MCController & ctl,
                                mc_rtc::Logger & logger,
                                sva::PTransformd & pose,
                                sva::MotionVecd & vels)
{
  const auto & realRobot = ctl.realRobot(robotName_);

  // copies the updated joints configuration from the real robot.
  odometryRobot().q() = realRobot.q();

  odometryRobot().posW(fbPose_);

  // we set the velocities and accelerations to zero as they will be compensated anyway as we compute the
  // successive poses in the local frame
  sva::MotionVecd zeroMotion;
  zeroMotion.linear() = so::Vector3::Zero();
  zeroMotion.angular() = so::Vector3::Zero();
  odometryRobot().velW(zeroMotion);
  odometryRobot().accW(zeroMotion);

  odometryRobot().forwardKinematics();

  // detects the contacts currently set with the environment
  contactsManager().findContacts(ctl, robotName_);
  // updates the contacts and the resulting floating base kinematics
  updateContacts(ctl, logger);
  // updates the floating base kinematics in the observer
  updateFbKinematics(ctl, pose, vels);
}

void LeggedOdometryManager::run(const mc_control::MCController & ctl, mc_rtc::Logger & logger, sva::PTransformd & pose)
{
  const auto & realRobot = ctl.realRobot(robotName_);

  // copies the updated joints configuration from the real robot.
  odometryRobot().q() = realRobot.q();

  odometryRobot().posW(fbPose_);
  odometryRobot().forwardKinematics();

  // we set the velocities and accelerations to zero as they will be compensated anyway as we compute the
  // successive poses in the local frame
  sva::MotionVecd zeroMotion;
  zeroMotion.linear() = so::Vector3::Zero();
  zeroMotion.angular() = so::Vector3::Zero();
  odometryRobot().velW(zeroMotion);
  odometryRobot().accW(zeroMotion);

  odometryRobot().forwardKinematics();

  // detects the contacts currently set with the environment
  contactsManager().findContacts(ctl, robotName_);
  // updates the contacts and the resulting floating base kinematics
  updateContacts(ctl, logger);
  // updates the floating base kinematics in the observer
  updateFbKinematics(ctl, pose);
}

void LeggedOdometryManager::updateContacts(const mc_control::MCController & ctl, mc_rtc::Logger & logger)
{
  const auto & robot = ctl.robot(robotName_);
  const auto & realRobot = ctl.realRobot(robotName_);

  double sumForces_position = 0.0;
  double sumForces_orientation = 0.0;

  // indicates if the position can be updated from the current contacts or not
  bool positionUpdatable = false;
  // indicates if the orientation can be updated from the current contacts or not
  bool orientationUpdatable = false;

  // selects the contacts to use for the yaw odometry
  selectForOrientationOdometry();

  // force weighted sum of the estimated floating base positions
  so::Vector3 totalFbPosition = so::Vector3::Zero();

  // checks that the position and orientation can be updated from the currently set contacts and computes the pose of
  // the floating base obtained from each contact
  for(const int & setContactIndex : contactsManager().contactsFound())
  {
    if(contactsManager()
           .contactWithSensor(setContactIndex)
           .wasAlreadySet_) // the contact already exists so we will use it to estimate the floating base pose
    {
      /* Computation of the world floating base position obtained from each currently set contact */
      positionUpdatable = true;

      LoContactWithSensor & setContact = contactsManager_.contactWithSensor(setContactIndex);
      const so::kine::Kinematics & worldContactKineOdometryRobot =
          getContactKinematics(setContact, robot.forceSensor(setContact.getName()));

      const so::Vector3 & worldFbPositionOdometryRobot = odometryRobot().posW().translation();

      so::Vector3 worldFbPositionOdometry = setContact.worldRefKine_.position()
                                            + (worldFbPositionOdometryRobot - worldContactKineOdometryRobot.position());

      sumForces_position += setContact.forceNorm_;
      // force weighted sum of the estimated floating base positions
      totalFbPosition += worldFbPositionOdometry * setContact.forceNorm_;

      if(withYawEstimation_ && setContact.useForOrientation_) // the orientation can be estimated
      {
        /* Computation of the world floating base orientation obtained from each currently set contact */
        orientationUpdatable = true;

        setContact.currentWorldFbOrientation_ =
            so::Matrix3(setContact.worldRefKine_.orientation.toMatrix3()
                        * worldContactKineOdometryRobot.orientation.toMatrix3().transpose()
                        * odometryRobot().posW().rotation().transpose());

        sumForces_orientation += setContact.forceNorm_;
      }
    }
  }

  // if the position and orientations can be updated through contacts (that were already set on the previous iteration),
  // they are updated, else we keep the previous estimation
  if(positionUpdatable)
  {
    // position of the floating base in the world, obtained by a weighted average of the estimations for each contact
    fbPose_.translation() = totalFbPosition / sumForces_position;

    if(orientationUpdatable) // the orientation can be updated
    {
      if(contactsManager_.oriOdometryContacts_.size() == 1) // the orientation can be updated using 1 contact
      {
        const so::Matrix3 & realRobotOri = realRobot.posW().rotation().transpose();
        // We merge the obtained yaw with the tilt estimated by the previous observers
        fbPose_.rotation() =
            so::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(
                realRobotOri, contactsManager_.oriOdometryContacts_.begin()->get().currentWorldFbOrientation_)
                .transpose();
      }
      if(contactsManager_.oriOdometryContacts_.size() == 2) // the orientation can be updated using 2 contacts
      {
        const auto & contact1 = *contactsManager_.oriOdometryContacts_.begin();
        const auto & contact2 = *std::next(contactsManager_.oriOdometryContacts_.begin(), 1);

        const auto & R1 = contact1.get().currentWorldFbOrientation_.toMatrix3();
        const auto & R2 = contact2.get().currentWorldFbOrientation_.toMatrix3();

        double u = contact1.get().forceNorm_ / sumForces_orientation;
        so::Matrix3 diffRot = R1.transpose() * R2;

        so::Vector3 diffRotVector = (1.0 - u)
                                    * so::kine::skewSymmetricToRotationVector(
                                        diffRot); // we perform the multiplication by the weighting coefficient now so a
                                                  // zero coefficient gives a unit rotation matrix and not a zero matrix
        so::AngleAxis diffRotAngleAxis = so::kine::rotationVectorToAngleAxis(diffRotVector);

        so::Matrix3 diffRotMatrix =
            so::kine::Orientation(diffRotAngleAxis).toMatrix3(); // exp( (1 - u) * log(R1^T R2) )

        so::Matrix3 meanOri = R1 * diffRotMatrix;

        const so::Matrix3 & realRobotOri =
            realRobot.posW().rotation().transpose(); // the odometryRobot()'s orientation is overwritten by the
                                                     // realRobot's one on every iteration
        fbPose_.rotation() = so::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(realRobotOri, meanOri).transpose();
      }
    }
  }
  // update of the pose of the floating base of the odometry robot in the world frame
  updateOdometryRobot(ctl, false, false);

  // computation of the reference kinematics of the newly set contacts in the world.
  for(const int & foundContactIndex : contactsManager().contactsFound())
  {
    if(!contactsManager().contactWithSensor(foundContactIndex).wasAlreadySet_) // the contact was not set so we will
                                                                               // compute its kinematics
    {
      LoContactWithSensor & foundContact = contactsManager_.contactWithSensor(foundContactIndex);

      setNewContact(foundContact, robot);
      addContactLogEntries(logger, foundContact);
    }
  }

  for(auto & removedContactIndex : contactsManager().removedContacts())
  {
    LoContactWithSensor & removedContact = contactsManager_.contactWithSensor(removedContactIndex);

    removeContactLogEntries(logger, removedContact);
  }
}

void LeggedOdometryManager::updateOdometryRobot(const mc_control::MCController & ctl,
                                                const bool updateVels,
                                                const bool updateAccs)
{
  const auto & realRobot = ctl.realRobot(robotName_);

  odometryRobot().posW(fbPose_);
  if(updateVels)
  {
    // realRobot.posW().rotation() is the transpose of R
    so::Vector3 realLocalLinVel = realRobot.posW().rotation() * realRobot.velW().linear();
    so::Vector3 realLocalAngVel = realRobot.posW().rotation() * realRobot.velW().angular();
    sva::MotionVecd vels;

    vels.linear() = odometryRobot().posW().rotation().transpose() * realLocalLinVel;
    vels.angular() = odometryRobot().posW().rotation().transpose() * realLocalAngVel;
    odometryRobot().velW(vels);
  }

  if(updateAccs)
  {
    // realRobot.posW().rotation() is the transpose of R
    so::Vector3 realLocalLinAcc = realRobot.posW().rotation() * realRobot.accW().linear();
    so::Vector3 realLocalAngAcc = realRobot.posW().rotation() * realRobot.accW().angular();
    sva::MotionVecd accs;

    accs.linear() = odometryRobot().posW().rotation().transpose() * realLocalLinAcc;
    accs.angular() = odometryRobot().posW().rotation().transpose() * realLocalAngAcc;

    odometryRobot().accW(accs);
  }

  odometryRobot().forwardKinematics();
}

void LeggedOdometryManager::updateFbKinematics(const mc_control::MCController & ctl,
                                               sva::PTransformd & pose,
                                               sva::MotionVecd & vels,
                                               sva::MotionVecd & accs)
{
  updateOdometryRobot(ctl, true, true);

  pose.rotation() = odometryRobot().posW().rotation();
  pose.translation() = odometryRobot().posW().translation();

  // we express the velocities and accelerations computed by the previous obervers in our newly estimated frame

  vels.linear() = odometryRobot().velW().linear();
  vels.angular() = odometryRobot().velW().angular();

  accs.linear() = odometryRobot().accW().linear();
  accs.angular() = odometryRobot().accW().angular();
}

void LeggedOdometryManager::updateFbKinematics(const mc_control::MCController & ctl,
                                               sva::PTransformd & pose,
                                               sva::MotionVecd & vels)
{
  updateOdometryRobot(ctl, true, false);

  pose.rotation() = odometryRobot().posW().rotation();
  pose.translation() = odometryRobot().posW().translation();

  // we express the velocities and accelerations computed by the previous obervers in our newly estimated frame

  vels.linear() = odometryRobot().velW().linear();
  vels.angular() = odometryRobot().velW().angular();
}

void LeggedOdometryManager::updateFbKinematics(const mc_control::MCController & ctl, sva::PTransformd & pose)
{
  updateOdometryRobot(ctl, false, false);

  pose.rotation() = odometryRobot().posW().rotation();
  pose.translation() = odometryRobot().posW().translation();
}

void LeggedOdometryManager::setNewContact(LoContactWithSensor & contact, const mc_rbdyn::Robot & measurementsRobot)
{
  const mc_rbdyn::ForceSensor & forceSensor = measurementsRobot.forceSensor(contact.forceSensorName());
  // If the contact is not detected using surfaces, we must consider that the frame of the sensor is the one of the
  // surface).
  if(detectionFromThreshold_)
  {
    so::kine::Kinematics worldNewContactKineOdometryRobot;
    so::kine::Kinematics worldContactKineRef;
    worldContactKineRef.setZero(so::kine::Kinematics::Flags::position);

    // getting the position in the world of the new contact
    const sva::PTransformd & bodyNewContactPoseRobot = forceSensor.X_p_f();
    so::kine::Kinematics bodyNewContactKine;
    bodyNewContactKine.setZero(so::kine::Kinematics::Flags::pose);
    bodyNewContactKine.position = bodyNewContactPoseRobot.translation();
    bodyNewContactKine.orientation = so::Matrix3(bodyNewContactPoseRobot.rotation().transpose());

    so::kine::Kinematics worldBodyKineOdometryRobot;

    worldBodyKineOdometryRobot.position =
        odometryRobot().mbc().bodyPosW[odometryRobot().bodyIndexByName(forceSensor.parentBody())].translation();
    worldBodyKineOdometryRobot.orientation =
        so::Matrix3(odometryRobot()
                        .mbc()
                        .bodyPosW[odometryRobot().bodyIndexByName(forceSensor.parentBody())]
                        .rotation()
                        .transpose());

    worldNewContactKineOdometryRobot = worldBodyKineOdometryRobot * bodyNewContactKine;

    contact.worldRefKine_.position = worldNewContactKineOdometryRobot.position();
    contact.worldRefKine_.orientation = worldNewContactKineOdometryRobot.orientation;
  }
  else // the kinematics of the contact are directly the ones of the surface
  {
    sva::PTransformd worldSurfacePoseOdometryRobot = odometryRobot().surfacePose(contact.surfaceName());

    contact.worldRefKine_.position = worldSurfacePoseOdometryRobot.translation();
    contact.worldRefKine_.orientation = so::Matrix3(worldSurfacePoseOdometryRobot.rotation().transpose());
  }

  if(!odometry6d_)
  {
    contact.worldRefKine_.position()(2) = 0.0;
  }
}

const so::kine::Kinematics & LeggedOdometryManager::getContactKinematics(LoContactWithSensor & contact,
                                                                         const mc_rbdyn::ForceSensor & fs)
{
  // robot is necessary because odometry robot doesn't have the copy of the force measurements
  const sva::PTransformd & bodyContactSensorPose = fs.X_p_f();
  so::kine::Kinematics bodyContactSensorKine =
      kinematicsTools::poseFromSva(bodyContactSensorPose, so::kine::Kinematics::Flags::vels);

  // kinematics of the sensor's parent body in the world
  so::kine::Kinematics worldBodyKineOdometryRobot =
      kinematicsTools::poseFromSva(odometryRobot().mbc().bodyPosW[odometryRobot().bodyIndexByName(fs.parentBody())],
                                   so::kine::Kinematics::Flags::pose);

  so::kine::Kinematics worldSensorKineOdometryRobot = worldBodyKineOdometryRobot * bodyContactSensorKine;

  if(detectionFromThreshold_)
  {
    // If the contact is detecting using thresholds, we will then consider the sensor frame as
    // the contact surface frame directly.
    contact.currentWorldKine_ = worldSensorKineOdometryRobot;
  }
  else // the kinematics of the contact are the ones of the associated surface
  {
    // the kinematics of the contacts are the ones of the surface, but we must transport the measured wrench
    sva::PTransformd worldSurfacePoseOdometryRobot = odometryRobot().surfacePose(contact.surfaceName());
    contact.currentWorldKine_ =
        kinematicsTools::poseFromSva(worldSurfacePoseOdometryRobot, so::kine::Kinematics::Flags::pose);

    so::kine::Kinematics contactSensorKine = contact.currentWorldKine_.getInverse() * worldSensorKineOdometryRobot;
    // expressing the force measurement in the frame of the surface
    contact.forceNorm_ = (contactSensorKine.orientation * fs.wrenchWithoutGravity(odometryRobot()).force()).norm();
  }

  return contact.currentWorldKine_;
}

void LeggedOdometryManager::selectForOrientationOdometry()
{
  contactsManager_.oriOdometryContacts_.clear();
  for(auto it = contactsManager_.contactsFound().begin(); it != contactsManager_.contactsFound().end(); it++)
  {
    LoContactWithSensor & contact = contactsManager_.contactWithSensor(*it);
    if(contact.getName().find("Hand") == std::string::npos
       && contact.wasAlreadySet_) // we don't use hands for the orientation odometry
    {
      contact.useForOrientation_ = true;
      contactsManager_.oriOdometryContacts_.insert(contact);
    }
  }
  // contacts are sorted from the lowest force to the highest force
  while(contactsManager_.oriOdometryContacts_.size() > 2)
  {
    (*contactsManager_.oriOdometryContacts_.begin()).get().useForOrientation_ = false;
    contactsManager_.oriOdometryContacts_.erase(contactsManager_.oriOdometryContacts_.begin());
  }
}

void LeggedOdometryManager::addContactLogEntries(mc_rtc::Logger & logger, const LoContactWithSensor & contact)
{
  const std::string & contactName = contact.getName();
  logger.addLogEntry(odometryName_ + "_" + contactName + "_ref_position",
                     [this, contact]() -> Eigen::Vector3d { return contact.worldRefKine_.position(); });
  logger.addLogEntry(odometryName_ + "_" + contactName + "_ref_orientation",
                     [this, contact]() -> so::Quaternion
                     { return contact.worldRefKine_.orientation.toQuaternion().inverse(); });
  logger.addLogEntry(odometryName_ + "_" + contactName + "_ref_orientation_RollPitchYaw",
                     [this, contact]() -> so::Vector3
                     {
                       so::kine::Orientation ori;
                       return so::kine::rotationMatrixToRollPitchYaw(
                           contact.worldRefKine_.orientation.toMatrix3().transpose());
                     });
}

void LeggedOdometryManager::removeContactLogEntries(mc_rtc::Logger & logger, const LoContactWithSensor & contact)
{
  const std::string & contactName = contact.getName();
  logger.removeLogEntry(odometryName_ + "_" + contactName + "_ref_position");
  logger.removeLogEntry(odometryName_ + "_" + contactName + "_ref_orientation");
  logger.removeLogEntry(odometryName_ + "_" + contactName + "_ref_orientation_RollPitchYaw");
}

so::kine::Kinematics & LeggedOdometryManager::getAnchorFramePose(const mc_control::MCController & ctl)
{

  const auto & robot = ctl.robot(robotName_);

  double sumForces_position = 0.0;
  double sumForces_orientation = 0.0;

  bool anchorUpdatable = false;

  // force weighted sum of the estimated floating base positions
  so::Vector3 totalAnchorPosition = so::Vector3::Zero();

  // checks that the position and orientation can be updated from the currently set contacts and computes the pose of
  // the floating base obtained from each contact
  for(const int & setContactIndex : contactsManager().contactsFound())
  {
    if(contactsManager()
           .contactWithSensor(setContactIndex)
           .wasAlreadySet_) // the contact already exists so we will use it to estimate the floating base pose
    {
      anchorUpdatable = true;

      LoContactWithSensor & setContact = contactsManager_.contactWithSensor(setContactIndex);
      const so::kine::Kinematics & worldContactKineOdometryRobot =
          getContactKinematics(setContact, robot.forceSensor(setContact.getName()));

      sumForces_position += setContact.forceNorm_;
      // force weighted sum of the estimated floating base positions
      totalAnchorPosition += setContact.currentWorldKine_.position() * setContact.forceNorm_;

      if(setContact.getName().find("Hand")
         == std::string::npos) // we take less the hands into account for the orientation odometry
      {
        sumForces_orientation += setContact.forceNorm_;
      }
      else
      {
        sumForces_orientation += 0.3 * setContact.forceNorm_;
      }
    }
  }

  if(!anchorUpdatable)
  {
    return worldAnchorFramePose_;
  }

  worldAnchorFramePose_.position = totalAnchorPosition / sumForces_position;

  if(contactsManager().contactsFound().size() > 2)
  {
    if(contactsManager_.oriOdometryContacts_.size() == 1) // the orientation can be updated using 1 contact
    {
      worldAnchorFramePose_.orientation =
          contactsManager_.oriOdometryContacts_.begin()->get().currentWorldKine_.orientation;
    }
    if(contactsManager_.oriOdometryContacts_.size() == 2) // the orientation can be updated using 2 contacts
    {
      const auto & contact1 = *contactsManager_.oriOdometryContacts_.begin();
      const auto & contact2 = *std::next(contactsManager_.oriOdometryContacts_.begin(), 1);

      const auto & R1 = contact1.get().currentWorldKine_.orientation.toMatrix3();
      const auto & R2 = contact2.get().currentWorldKine_.orientation.toMatrix3();

      double u;

      // we take less the hands into account for the orientation odometry. The hands
      // orientation is generally less trustable, notably because of the lower force implying
      // more potential slippage.
      if(contact1.get().getName().find("Hand") == std::string::npos)
      {
        u = 0.3 * contact1.get().forceNorm_ / sumForces_orientation;
      }
      else
      {
        u = contact1.get().forceNorm_ / sumForces_orientation;
      }

      so::Matrix3 diffRot = R1.transpose() * R2;

      so::Vector3 diffRotVector = (1.0 - u)
                                  * so::kine::skewSymmetricToRotationVector(
                                      diffRot); // we perform the multiplication by the weighting coefficient now so a
                                                // zero coefficient gives a unit rotation matrix and not a zero matrix
      so::AngleAxis diffRotAngleAxis = so::kine::rotationVectorToAngleAxis(diffRotVector);

      so::Matrix3 diffRotMatrix = so::kine::Orientation(diffRotAngleAxis).toMatrix3(); // exp( (1 - u) * log(R1^T R2) )

      so::Matrix3 meanOri = R1 * diffRotMatrix;

      worldAnchorFramePose_.orientation = meanOri;
    }
  }
  else
  {
    if(contactsManager().contactsFound().size() == 1) // the orientation can be updated using 1 contact
    {
      worldAnchorFramePose_.orientation =
          contactsManager().contactWithSensor(*contactsManager().contactsFound().begin()).currentWorldKine_.orientation;
    }
    if(contactsManager().contactsFound().size() == 2) // the orientation can be updated using 2 contacts
    {
      const auto & contact1 = contactsManager().contactWithSensor(*contactsManager().contactsFound().begin());
      const auto & contact2 =
          contactsManager().contactWithSensor(*std::next(contactsManager().contactsFound().begin(), 1));

      const auto & R1 = contact1.currentWorldKine_.orientation.toMatrix3();
      const auto & R2 = contact2.currentWorldKine_.orientation.toMatrix3();

      double u;

      if(contact1.getName().find("Hand")
         == std::string::npos) // we take less the hands into account for the orientation odometry
      {
        u = 0.3 * contact1.forceNorm_ / sumForces_orientation;
      }
      else
      {
        u = contact1.forceNorm_ / sumForces_orientation;
      }

      so::Matrix3 diffRot = R1.transpose() * R2;

      so::Vector3 diffRotVector = (1.0 - u)
                                  * so::kine::skewSymmetricToRotationVector(
                                      diffRot); // we perform the multiplication by the weighting coefficient now so a
                                                // zero coefficient gives a unit rotation matrix and not a zero matrix
      so::AngleAxis diffRotAngleAxis = so::kine::rotationVectorToAngleAxis(diffRotVector);

      so::Matrix3 diffRotMatrix = so::kine::Orientation(diffRotAngleAxis).toMatrix3(); // exp( (1 - u) * log(R1^T R2) )

      so::Matrix3 meanOri = R1 * diffRotMatrix;

      worldAnchorFramePose_.orientation = meanOri;
    }
  }
  return worldAnchorFramePose_;
}
} // namespace leggedOdometry
} // namespace mc_state_observation
