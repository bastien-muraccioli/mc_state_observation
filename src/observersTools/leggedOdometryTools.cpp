#include <mc_state_observation/observersTools/kinematicsTools.h>

#include <mc_state_observation/observersTools/leggedOdometryTools.h>

namespace so = stateObservation;

namespace mc_state_observation
{
namespace leggedOdometry
{

///////////////////////////////////////////////////////////////////////
/// ------------------------------Contacts-----------------------------
///////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
/// -------------------------Legged Odometry---------------------------
///////////////////////////////////////////////////////////////////////

void LeggedOdometryManager::setNewContact(const mc_rbdyn::Robot & odometryRobot,
                                          const mc_rbdyn::ForceSensor forceSensor)
{

  if(contactsManager().contactWithSensor(forceSensor.name()).sensorAttachedToSurface)
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
        odometryRobot.mbc().bodyPosW[odometryRobot.bodyIndexByName(forceSensor.parentBody())].translation();
    worldBodyKineOdometryRobot.orientation = so::Matrix3(
        odometryRobot.mbc().bodyPosW[odometryRobot.bodyIndexByName(forceSensor.parentBody())].rotation().transpose());
    /*
worldBodyKineOdometryRobot.linVel =
    odometryRobot.mbc().bodyVelW[odometryRobot.bodyIndexByName(forceSensor.parentBody())].linear();
worldBodyKineOdometryRobot.angVel =
    odometryRobot.mbc().bodyVelW[odometryRobot.bodyIndexByName(forceSensor.parentBody())].angular();
worldBodyKineOdometryRobot.linAcc =
    worldBodyKineOdometryRobot.orientation.toMatrix3()
    * odometryRobot.mbc().bodyAccB[odometryRobot.bodyIndexByName(forceSensor.parentBody())].linear();
worldBodyKineOdometryRobot.angAcc =
    worldBodyKineOdometryRobot.orientation.toMatrix3()
    * odometryRobot.mbc().bodyAccB[odometryRobot.bodyIndexByName(forceSensor.parentBody())].angular();
    */

    worldNewContactKineOdometryRobot = worldBodyKineOdometryRobot * bodyNewContactKine;

    contactsManager().contactWithSensor(forceSensor.name()).worldRefKine_.position =
        worldNewContactKineOdometryRobot.position();
    contactsManager().contactWithSensor(forceSensor.name()).worldRefKine_.orientation =
        worldNewContactKineOdometryRobot.orientation;
  }
  else
  {
    sva::PTransformd worldSurfacePoseOdometryRobot =
        odometryRobot.surfacePose(contactsManager().contactWithSensor(forceSensor.name()).surface);

    contactsManager().contactWithSensor(forceSensor.name()).worldRefKine_.position =
        worldSurfacePoseOdometryRobot.translation();
    contactsManager().contactWithSensor(forceSensor.name()).worldRefKine_.orientation =
        so::Matrix3(worldSurfacePoseOdometryRobot.rotation().transpose());
  }

  if(!odometry6d_)
  {
    contactsManager().contactWithSensor(forceSensor.name()).worldRefKine_.position()(2) = 0.0;
  }
}

void LeggedOdometryManager::updateContacts(const mc_control::MCController & ctl,
                                           mc_rbdyn::Robot & odometryRobot,
                                           mc_rtc::Logger & logger)
{
  const auto & robot = ctl.robot(robotName_);
  const auto & realRobot = ctl.realRobot(robotName_);

  double sumForces_position = 0.0;
  double sumForces_orientation = 0.0;

  bool positionUpdatable = false;
  bool orientationUpdatable = false;

  selectForOrientationOdometry();

  so::Vector3 totalFbPosition = so::Vector3::Zero();
  for(const int & setContactIndex : contactsManager().contactsFound())
  {
    if(contactsManager()
           .contactWithSensor(setContactIndex)
           .wasAlreadySet_) // the contact already exists so we will use it to estimate the floating base pose
    {
      positionUpdatable = true;

      LoContactWithSensor & setContact = contactsManager_.contactWithSensor(setContactIndex);
      so::kine::Kinematics worldContactKineOdometryRobot = getContactKinematics(setContact, robot, odometryRobot);

      const so::Vector3 & worldFbPositionOdometryRobot = odometryRobot.posW().translation();

      so::Vector3 worldFbPositionOdometry = setContact.worldRefKine_.position()
                                            + (worldFbPositionOdometryRobot - worldContactKineOdometryRobot.position());

      sumForces_position += setContact.forceNorm_;
      totalFbPosition += worldFbPositionOdometry * setContact.forceNorm_;

      if(withNaiveYawEstimation_ && setContact.useForOrientation_)
      {
        orientationUpdatable = true;

        setContact.currentWorldOrientation_ =
            so::Matrix3(setContact.worldRefKine_.orientation.toMatrix3()
                        * worldContactKineOdometryRobot.orientation.toMatrix3().transpose()
                        * odometryRobot.posW().rotation().transpose());

        sumForces_orientation += setContact.forceNorm_;
      }
    }
  }

  // if the position and orientations can be updated through contacts (that were already set on the previous iteration),
  // they are updated, else we keep the previous estimation
  if(positionUpdatable)
  {
    X_0_fb_.translation() = totalFbPosition / sumForces_position;
    if(withNaiveYawEstimation_)
    {
      if(orientationUpdatable)
      {
        if(contactsManager_.oriOdometryContacts_.size() == 1) // the orientation can be updated using 1 contact
        {
          const so::Matrix3 & realRobotOri = realRobot.posW().rotation().transpose();
          // We merge the obtained yaw with the tilt estimated by the previous observers
          X_0_fb_.rotation() =
              so::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(
                  realRobotOri, contactsManager_.oriOdometryContacts_.begin()->get().currentWorldOrientation_)
                  .transpose();
        }
        if(contactsManager_.oriOdometryContacts_.size() == 2) // the orientation can be updated using 2 contacts
        {
          const auto & contact1 = *contactsManager_.oriOdometryContacts_.begin();
          const auto & contact2 = *std::next(contactsManager_.oriOdometryContacts_.begin(), 1);

          const auto & R1 = contact1.get().currentWorldOrientation_.toMatrix3();
          const auto & R2 = contact2.get().currentWorldOrientation_.toMatrix3();

          double u = contact1.get().forceNorm_ / sumForces_orientation;
          so::Matrix3 diffRot = R1.transpose() * R2;

          so::Vector3 diffRotVector =
              (1.0 - u)
              * so::kine::skewSymmetricToRotationVector(
                  diffRot); // we perform the multiplication by the weighting coefficient now so a
                            // zero coefficient gives a unit rotation matrix and not a zero matrix
          so::AngleAxis diffRotAngleAxis = so::kine::rotationVectorToAngleAxis(diffRotVector);

          so::Matrix3 diffRotMatrix =
              so::kine::Orientation(diffRotAngleAxis).toMatrix3(); // exp( (1 - u) * log(R1^T R2) )

          so::Matrix3 meanOri = R1 * diffRotMatrix;

          const so::Matrix3 & realRobotOri =
              realRobot.posW().rotation().transpose(); // the odometryRobot's orientation is overwritten by the
                                                       // realRobot's one on every iteration
          X_0_fb_.rotation() = so::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(realRobotOri, meanOri).transpose();
        }
      }
    }
  }

  odometryRobot.posW(X_0_fb_);
  odometryRobot.forwardKinematics();
  for(const int & foundContactIndex : contactsManager().contactsFound())
  {
    if(!contactsManager().contactWithSensor(foundContactIndex).wasAlreadySet_) // the contact was not set so we will
                                                                               // compute its kinematics
    {
      LoContactWithSensor & foundContact = contactsManager_.contactWithSensor(foundContactIndex);
      const mc_rbdyn::ForceSensor & fs = robot.forceSensor(foundContact.getName());
      foundContact.isSet_ = true;

      setNewContact(odometryRobot, fs);
      addContactLogEntries(logger, fs.name());
    }
  }

  for(auto & removedContactIndex : contactsManager().removedContacts())
  {
    LoContactWithSensor & removedContact = contactsManager_.contactWithSensor(removedContactIndex);
    removedContact.isSet_ = false;
    removeContactLogEntries(logger, removedContact.getName());
  }
}

so::kine::Kinematics LeggedOdometryManager::getContactKinematics(LoContactWithSensor & contact,
                                                                 const mc_rbdyn::Robot & robot,
                                                                 const mc_rbdyn::Robot & odometryRobot)
{
  // robot is necessary because odometry robot doesn't have the copy of the force measurements
  const mc_rbdyn::ForceSensor & fs = robot.forceSensor(contact.getName());
  const sva::PTransformd & bodyContactSensorPose = fs.X_p_f();
  so::kine::Kinematics bodyContactSensorKine =
      kinematicsTools::poseFromSva(bodyContactSensorPose, so::kine::Kinematics::Flags::vels);

  // kinematics of the sensor's parent body in the world
  so::kine::Kinematics worldBodyKineOdometryRobot = kinematicsTools::kinematicsFromSva(
      odometryRobot.mbc().bodyPosW[odometryRobot.bodyIndexByName(fs.parentBody())],
      odometryRobot.mbc().bodyVelW[odometryRobot.bodyIndexByName(fs.parentBody())],
      odometryRobot.mbc().bodyAccB[odometryRobot.bodyIndexByName(fs.parentBody())], true, false);

  so::kine::Kinematics worldSensorKineOdometryRobot = worldBodyKineOdometryRobot * bodyContactSensorKine;
  if(contact.sensorAttachedToSurface)
  {
    // The kinematics of the surface corresponds to the kinematics of the sensor.
    // This also applies if the contact is detecting using thresholds, as we will then consider the sensor frame as
    // the contact surface frame directly.
    return worldSensorKineOdometryRobot;
  }
  else
  {
    // the kinematics of the contacts are the ones of the surface, but we must transport the measured wrench
    sva::PTransformd worldSurfacePoseOdometryRobot = odometryRobot.surfacePose(contact.surface);
    so::kine::Kinematics worldContactKineOdometryRobot =
        kinematicsTools::poseFromSva(worldSurfacePoseOdometryRobot, so::kine::Kinematics::Flags::pose);

    so::kine::Kinematics contactSensorKine = worldContactKineOdometryRobot.getInverse() * worldSensorKineOdometryRobot;
    // expressing the force measurement in the frame of the surface
    contact.forceNorm_ = (contactSensorKine.orientation * fs.wrenchWithoutGravity(odometryRobot).force()).norm();

    return worldContactKineOdometryRobot;
  }
}

void LeggedOdometryManager::selectForOrientationOdometry()
{
  contactsManager_.oriOdometryContacts_.clear();
  for(auto it = contactsManager_.contactsFound().begin(); it != contactsManager_.contactsFound().end(); it++)
  {
    LoContactWithSensor & contact = contactsManager_.contactWithSensor(*it);
    if(contact.getName().find("Hand") == std::string::npos) // we don't use hands for the orientation odometry
    {
      contact.useForOrientation_ = true;
      contactsManager_.oriOdometryContacts_.insert(contact);
    }
  }
  // contacts are sorted from the lowest force to the highest force
  while(contactsManager_.oriOdometryContacts_.size() > 2)
  {
    contactsManager_.oriOdometryContacts_.erase(contactsManager_.oriOdometryContacts_.begin());
  }
}

void LeggedOdometryManager::addContactLogEntries(mc_rtc::Logger & logger, const std::string & contactName)
{
  logger.addLogEntry(odometryName_ + "_" + contactName + "_ref_position",
                     [this, contactName]() -> Eigen::Vector3d
                     { return contactsManager_.contactWithSensor(contactName).worldRefKine_.position(); });
  logger.addLogEntry(
      odometryName_ + "_" + contactName + "_ref_orientation",
      [this, contactName]() -> so::Quaternion
      { return contactsManager_.contactWithSensor(contactName).worldRefKine_.orientation.toQuaternion().inverse(); });
  logger.addLogEntry(
      odometryName_ + "_" + contactName + "_ref_orientation_RollPitchYaw",
      [this, contactName]() -> so::Vector3
      {
        so::kine::Orientation ori;
        return so::kine::rotationMatrixToRollPitchYaw(
            contactsManager_.contactWithSensor(contactName).worldRefKine_.orientation.toMatrix3().transpose());
      });
}

void LeggedOdometryManager::removeContactLogEntries(mc_rtc::Logger & logger, const std::string & contactName)
{
  logger.removeLogEntry(odometryName_ + "_" + contactName + "_ref_position");
  logger.removeLogEntry(odometryName_ + "_" + contactName + "_ref_orientation");
  logger.removeLogEntry(odometryName_ + "_" + contactName + "_ref_orientation_RollPitchYaw");
}

} // namespace leggedOdometry
} // namespace mc_state_observation
