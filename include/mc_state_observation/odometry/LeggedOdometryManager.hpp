#pragma once
#include <mc_state_observation/conversions/kinematics.h>
#include <mc_state_observation/odometry/LeggedOdometryManager.h>

namespace mc_state_observation::odometry
{
template<typename OnNewContactObserver,
         typename OnMaintainedContactObserver,
         typename OnRemovedContactObserver,
         typename OnAddedContactObserver>
void LeggedOdometryManager::run(const mc_control::MCController & ctl,
                                mc_rtc::Logger & logger,
                                sva::PTransformd & pose,
                                OnNewContactObserver onNewContactObserver,
                                OnMaintainedContactObserver onMaintainedContactObserver,
                                OnRemovedContactObserver onRemovedContactObserver,
                                OnAddedContactObserver onAddedContactObserver)
{
  // the tilt must come from another estimator so we will use the real robot for the orientation
  runPvt(ctl, logger, pose, onNewContactObserver, onMaintainedContactObserver, onRemovedContactObserver,
         onAddedContactObserver);
}

template<typename OnNewContactObserver,
         typename OnMaintainedContactObserver,
         typename OnRemovedContactObserver,
         typename OnAddedContactObserver>
void LeggedOdometryManager::run(const mc_control::MCController & ctl,
                                mc_rtc::Logger & logger,
                                sva::PTransformd & pose,
                                sva::MotionVecd & vel,
                                OnNewContactObserver onNewContactObserver,
                                OnMaintainedContactObserver onMaintainedContactObserver,
                                OnRemovedContactObserver onRemovedContactObserver,
                                OnAddedContactObserver onAddedContactObserver)
{
  // the tilt must come from another estimator so we will use the real robot for the orientation
  runPvt(ctl, logger, pose, nullptr, &vel, onNewContactObserver, onMaintainedContactObserver, onRemovedContactObserver,
         onAddedContactObserver);
}

template<typename OnNewContactObserver,
         typename OnMaintainedContactObserver,
         typename OnRemovedContactObserver,
         typename OnAddedContactObserver>
void LeggedOdometryManager::run(const mc_control::MCController & ctl,
                                mc_rtc::Logger & logger,
                                sva::PTransformd & pose,
                                sva::MotionVecd & vel,
                                sva::MotionVecd & acc,
                                OnNewContactObserver onNewContactObserver,
                                OnMaintainedContactObserver onMaintainedContactObserver,
                                OnRemovedContactObserver onRemovedContactObserver,
                                OnAddedContactObserver onAddedContactObserver)
{
  // the tilt must come from another estimator so we will use the real robot for the orientation
  runPvt(ctl, logger, pose, nullptr, &vel, &acc, onNewContactObserver, onMaintainedContactObserver,
         onRemovedContactObserver, onAddedContactObserver);
}

template<typename OnNewContactObserver,
         typename OnMaintainedContactObserver,
         typename OnRemovedContactObserver,
         typename OnAddedContactObserver>
void LeggedOdometryManager::run(const mc_control::MCController & ctl,
                                mc_rtc::Logger & logger,
                                sva::PTransformd & pose,
                                const stateObservation::Matrix3 & tilt,
                                OnNewContactObserver onNewContactObserver,
                                OnMaintainedContactObserver onMaintainedContactObserver,
                                OnRemovedContactObserver onRemovedContactObserver,
                                OnAddedContactObserver onAddedContactObserver)
{
  runPvt(ctl, logger, pose, &tilt, onNewContactObserver, onMaintainedContactObserver, onRemovedContactObserver,
         onAddedContactObserver);
}

template<typename OnNewContactObserver,
         typename OnMaintainedContactObserver,
         typename OnRemovedContactObserver,
         typename OnAddedContactObserver>
void LeggedOdometryManager::run(const mc_control::MCController & ctl,
                                mc_rtc::Logger & logger,
                                sva::PTransformd & pose,
                                const stateObservation::Matrix3 & tilt,
                                sva::MotionVecd & vel,
                                OnNewContactObserver onNewContactObserver,
                                OnMaintainedContactObserver onMaintainedContactObserver,
                                OnRemovedContactObserver onRemovedContactObserver,
                                OnAddedContactObserver onAddedContactObserver)
{
  runPvt(ctl, logger, pose, &tilt, &vel, onNewContactObserver, onMaintainedContactObserver, onRemovedContactObserver,
         onAddedContactObserver);
}

template<typename OnNewContactObserver,
         typename OnMaintainedContactObserver,
         typename OnRemovedContactObserver,
         typename OnAddedContactObserver>
void LeggedOdometryManager::run(const mc_control::MCController & ctl,
                                mc_rtc::Logger & logger,
                                sva::PTransformd & pose,
                                const stateObservation::Matrix3 & tilt,
                                sva::MotionVecd & vel,
                                sva::MotionVecd & acc,
                                OnNewContactObserver onNewContactObserver,
                                OnMaintainedContactObserver onMaintainedContactObserver,
                                OnRemovedContactObserver onRemovedContactObserver,
                                OnAddedContactObserver onAddedContactObserver)
{
  runPvt(ctl, logger, pose, &tilt, &vel, &acc, onNewContactObserver, onMaintainedContactObserver,
         onRemovedContactObserver, onAddedContactObserver);
}

template<typename OnNewContactObserver,
         typename OnMaintainedContactObserver,
         typename OnRemovedContactObserver,
         typename OnAddedContactObserver>
void LeggedOdometryManager::runWithFullAttitude(const mc_control::MCController & ctl,
                                                mc_rtc::Logger & logger,
                                                sva::PTransformd & pose,
                                                const stateObservation::Matrix3 & attitude,
                                                sva::MotionVecd * vel,
                                                sva::MotionVecd * acc,
                                                OnNewContactObserver onNewContactObserver,
                                                OnMaintainedContactObserver onMaintainedContactObserver,
                                                OnRemovedContactObserver onRemovedContactObserver,
                                                OnAddedContactObserver onAddedContactObserver)
{
  updateJointsConfiguration(ctl);
  odometryRobot().posW(fbPose_);

  // we set the velocity and acceleration to zero as they will be compensated anyway as we compute the
  // successive poses in the local frame
  sva::MotionVecd zeroMotion;
  zeroMotion.linear() = stateObservation::Vector3::Zero();
  zeroMotion.angular() = stateObservation::Vector3::Zero();
  odometryRobot().velW(zeroMotion);
  odometryRobot().accW(zeroMotion);

  odometryRobot().forwardKinematics();
  odometryRobot().forwardVelocity();
  odometryRobot().forwardAcceleration();

  updateFbAndContactsWithFullAttitude(ctl, logger, attitude, vel, acc, onNewContactObserver,
                                      onMaintainedContactObserver, onRemovedContactObserver, onAddedContactObserver);

  // updates the floating base kinematics in the observer
  updateFbKinematicsPvt(pose, vel, acc);
}

template<typename OnNewContactObserver,
         typename OnMaintainedContactObserver,
         typename OnRemovedContactObserver,
         typename OnAddedContactObserver>
void LeggedOdometryManager::runPvt(const mc_control::MCController & ctl,
                                   mc_rtc::Logger & logger,
                                   sva::PTransformd & pose,
                                   const stateObservation::Matrix3 * tilt,
                                   sva::MotionVecd * vel,
                                   sva::MotionVecd * acc,
                                   OnNewContactObserver onNewContactObserver,
                                   OnMaintainedContactObserver onMaintainedContactObserver,
                                   OnRemovedContactObserver onRemovedContactObserver,
                                   OnAddedContactObserver onAddedContactObserver)
{
  updateJointsConfiguration(ctl);
  odometryRobot().posW(fbPose_);

  // we set the velocity and acceleration to zero as they will be compensated anyway as we compute the
  // successive poses in the local frame
  sva::MotionVecd zeroMotion;
  zeroMotion.linear() = stateObservation::Vector3::Zero();
  zeroMotion.angular() = stateObservation::Vector3::Zero();
  odometryRobot().velW(zeroMotion);
  odometryRobot().accW(zeroMotion);

  odometryRobot().forwardKinematics();
  odometryRobot().forwardVelocity();
  odometryRobot().forwardAcceleration();

  // updates the contacts and the resulting floating base kinematics
  if(tilt != nullptr)
  {
    updateFbAndContacts(ctl, logger, *tilt, vel, acc, onNewContactObserver, onMaintainedContactObserver,
                        onRemovedContactObserver, onAddedContactObserver);
  }
  else
  {
    // the tilt must come from another estimator so we will use the real robot for the orientation
    const auto & realRobot = ctl.realRobot(robotName_);
    stateObservation::Matrix3 realRobotOri = realRobot.posW().rotation().transpose();

    updateFbAndContacts(ctl, logger, realRobotOri, vel, acc, onNewContactObserver, onMaintainedContactObserver,
                        onRemovedContactObserver, onAddedContactObserver);
  }

  // updates the floating base kinematics in the observer
  updateFbKinematicsPvt(pose, vel, acc);
}

template<typename OnNewContactObserver,
         typename OnMaintainedContactObserver,
         typename OnRemovedContactObserver,
         typename OnAddedContactObserver>
void LeggedOdometryManager::updateFbAndContacts(const mc_control::MCController & ctl,
                                                mc_rtc::Logger & logger,
                                                const stateObservation::Matrix3 & tilt,
                                                sva::MotionVecd * vel,
                                                sva::MotionVecd * acc,
                                                OnNewContactObserver onNewContactObserver,
                                                OnMaintainedContactObserver onMaintainedContactObserver,
                                                OnRemovedContactObserver onRemovedContactObserver,
                                                OnAddedContactObserver onAddedContactObserver)
{
  // If the position and orientation of the floating base can be updated using contacts (that were already set on the
  // previous iteration), they are updated, else we keep the previous estimation. Then we estimate the pose of new
  // contacts using the obtained pose of the floating base.

  const auto & robot = ctl.robot(robotName_);

  double sumForces_position = 0.0;
  double sumForces_orientation = 0.0;

  // indicates if the position can be updated from the current contacts or not
  bool posUpdatable = false;
  // indicates if the orientation can be updated from the current contacts or not
  bool oriUpdatable = false;

  // force weighted sum of the estimated floating base positions
  stateObservation::Vector3 totalFbPosition = stateObservation::Vector3::Zero();

  // Needed later on
  std::vector<LoContactWithSensor *> newContacts;

  // current estimate of the pose of the robot in the world
  const stateObservation::kine::Kinematics worldFbPose =
      conversions::kinematics::fromSva(odometryRobot().posW(), stateObservation::kine::Kinematics::Flags::pose);

  auto onNewContact = [this, &logger, &newContacts, &onNewContactObserver](LoContactWithSensor & newContact)
  {
    if constexpr(!std::is_same_v<OnNewContactObserver, std::nullptr_t>) { onNewContactObserver(newContact); }
    addContactLogEntries(logger, newContact);
    newContacts.push_back(&newContact);
  };
  auto onMaintainedContact =
      [this, &robot, &posUpdatable, &onMaintainedContactObserver](LoContactWithSensor & maintainedContact)
  {
    if constexpr(!std::is_same_v<OnMaintainedContactObserver, std::nullptr_t>)
    {
      onMaintainedContactObserver(maintainedContact);
    }
    // indicates that we can compute the position of the floating base using the contacts
    posUpdatable = true;

    // we update the kinematics of the contact in the world obtained from the floating base and the sensor reading
    getCurrentContactKinematics(maintainedContact, robot.forceSensor(maintainedContact.name()));
  };

  auto onRemovedContact = [this, &logger, &onRemovedContactObserver](LoContactWithSensor & removedContact)
  {
    if constexpr(!std::is_same_v<OnRemovedContactObserver, std::nullptr_t>)
    {
      onRemovedContactObserver(removedContact);
    }
    removeContactLogEntries(logger, removedContact);
  };

  // detects the contacts currently set with the environment
  contactsManager().updateContacts(ctl, robotName_, onNewContact, onMaintainedContact, onRemovedContact,
                                   onAddedContactObserver);

  // selects the contacts to use for the yaw odometry. We cannot call it in the onMaintainedContact function as it is
  // looping over all the maintained contact and not used on each contact separately
  selectForOrientationOdometry(oriUpdatable, sumForces_orientation, worldFbPose);

  // we update the orientation of the floating base first
  if(oriUpdatable)
  {
    // the orientation can be updated using contacts, it will use at most the two most suitable contacts.
    // We merge the obtained yaw with the tilt estimated by the previous observers
    if(contactsManager_.oriOdometryContacts_.size() == 1)
    {
      // the orientation can be updated using 1 contact
      fbPose_.rotation() =
          stateObservation::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(
              tilt, contactsManager_.oriOdometryContacts_.begin()->get().currentWorldFbPose_.orientation)
              .transpose();
    }
    if(contactsManager_.oriOdometryContacts_.size() == 2) // the orientation can be updated using 2 contacts
    {
      const auto & contact1 = (*contactsManager_.oriOdometryContacts_.begin()).get();
      const auto & contact2 = (*std::next(contactsManager_.oriOdometryContacts_.begin(), 1)).get();

      const auto & R1 = contact1.currentWorldFbPose_.orientation.toMatrix3();
      const auto & R2 = contact2.currentWorldFbPose_.orientation.toMatrix3();

      double u = contact1.forceNorm() / sumForces_orientation;

      stateObservation::Matrix3 diffRot = R1.transpose() * R2;

      stateObservation::Vector3 diffRotVector =
          (1.0 - u)
          * stateObservation::kine::skewSymmetricToRotationVector(
              diffRot); // we perform the multiplication by the weighting coefficient now so a
                        // zero coefficient gives a unit rotation matrix and not a zero matrix

      stateObservation::AngleAxis diffRotAngleAxis = stateObservation::kine::rotationVectorToAngleAxis(diffRotVector);

      stateObservation::Matrix3 diffRotMatrix =
          stateObservation::kine::Orientation(diffRotAngleAxis).toMatrix3(); // exp( (1 - u) * log(R1^T R2) )

      stateObservation::Matrix3 meanOri = R1 * diffRotMatrix;
      fbPose_.rotation() = stateObservation::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(tilt, meanOri).transpose();
    }
  }
  else
  {
    // If no contact is detected, the yaw will not be updated but the tilt will.
    fbPose_.rotation() =
        stateObservation::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(tilt, fbPose_.rotation().transpose()).transpose();
  }

  // we update the orientation of the floating base
  odometryRobot().posW(fbPose_);
  odometryRobot().forwardKinematics();

  correctContactsOri(robot);

  // if we can update the position, we compute the weighted average of the position obtained from the contacts
  if(posUpdatable)
  {
    selectForPositionOdometry(sumForces_position, totalFbPosition, worldFbPose, robot);
    fbPose_.translation() = totalFbPosition / sumForces_position;
  }

  // update of the pose of the floating base of the odometry robot in the world frame before creating the new contacts
  updateOdometryRobot(ctl, vel, acc);

  // we correct the reference position of the contacts in the world
  if(posUpdatable) { correctContactsPosition(robot); }

  // computation of the reference kinematics of the newly set contacts in the world. We cannot use the onNewContacts
  // function as it is used at the beginning of the iteration and we need to compute this at the end
  for(auto * nContact : newContacts) { setNewContact(*nContact, robot); }
}

template<typename OnNewContactObserver,
         typename OnMaintainedContactObserver,
         typename OnRemovedContactObserver,
         typename OnAddedContactObserver>
void LeggedOdometryManager::updateFbAndContactsWithFullAttitude(const mc_control::MCController & ctl,
                                                                mc_rtc::Logger & logger,
                                                                const stateObservation::Matrix3 & attitude,
                                                                sva::MotionVecd * vel,
                                                                sva::MotionVecd * acc,
                                                                OnNewContactObserver onNewContactObserver,
                                                                OnMaintainedContactObserver onMaintainedContactObserver,
                                                                OnRemovedContactObserver onRemovedContactObserver,
                                                                OnAddedContactObserver onAddedContactObserver)
{
  // If the position and orientation of the floating base can be updated using contacts (that were already set on the
  // previous iteration), they are updated, else we keep the previous estimation. Then we estimate the pose of new
  // contacts using the obtained pose of the floating base.

  const auto & robot = ctl.robot(robotName_);

  double sumForces_position = 0.0;

  // indicates if the position can be updated from the current contacts or not
  bool posUpdatable = false;

  // force weighted sum of the estimated floating base positions
  stateObservation::Vector3 totalFbPosition = stateObservation::Vector3::Zero();

  // Needed later on
  std::vector<LoContactWithSensor *> newContacts;

  // current estimate of the pose of the robot in the world
  const stateObservation::kine::Kinematics worldFbPose =
      conversions::kinematics::fromSva(odometryRobot().posW(), stateObservation::kine::Kinematics::Flags::pose);

  auto onNewContact = [this, &logger, &newContacts, &onNewContactObserver](LoContactWithSensor & newContact)
  {
    if constexpr(!std::is_same_v<OnNewContactObserver, std::nullptr_t>) { onNewContactObserver(newContact); }

    addContactLogEntries(logger, newContact);
    newContacts.push_back(&newContact);
  };
  auto onMaintainedContact =
      [this, &robot, &posUpdatable, &onMaintainedContactObserver](LoContactWithSensor & maintainedContact)
  {
    if constexpr(!std::is_same_v<OnMaintainedContactObserver, std::nullptr_t>)
    {
      onMaintainedContactObserver(maintainedContact);
    }

    // indicates that we can compute the position of the floating base using the contacts
    posUpdatable = true;

    // we update the kinematics of the contact in the world obtained from the floating base and the sensor reading
    getCurrentContactKinematics(maintainedContact, robot.forceSensor(maintainedContact.name()));
  };

  auto onRemovedContact = [this, &logger, &onRemovedContactObserver](LoContactWithSensor & removedContact)
  {
    if constexpr(!std::is_same_v<OnRemovedContactObserver, std::nullptr_t>)
    {
      onRemovedContactObserver(removedContact);
    }
    removeContactLogEntries(logger, removedContact);
  };

  // detects the contacts currently set with the environment
  contactsManager().updateContacts(ctl, robotName_, onNewContact, onMaintainedContact, onRemovedContact,
                                   onAddedContactObserver);

  // we update the orientation of the floating base
  fbPose_.rotation() = attitude.transpose();
  odometryRobot().posW(fbPose_);
  odometryRobot().forwardKinematics();

  // we correct the orientation of contacts
  correctContactsOri(robot);

  // if we can update the position, we compute the weighted average of the position obtained from the contacts
  if(posUpdatable)
  {
    selectForPositionOdometry(sumForces_position, totalFbPosition, worldFbPose, robot);
    fbPose_.translation() = totalFbPosition / sumForces_position;
  }

  // update of the pose of the floating base of the odometry robot in the world frame before creating the new contacts
  updateOdometryRobot(ctl, vel, acc);

  // we correct the reference position of the contacts in the world
  if(posUpdatable) { correctContactsPosition(robot); }

  // computation of the reference kinematics of the newly set contacts in the world. We cannot use the onNewContacts
  // function as it is used at the beginning of the iteration and we need to compute this at the end
  for(auto * nContact : newContacts) { setNewContact(*nContact, robot); }
}

} // namespace mc_state_observation::odometry
