#pragma once

#include <mc_state_observation/observersTools/measurementsTools.h>
#include <state-observation/dynamics-estimators/kinetics-observer.hpp>

namespace mc_state_observation
{
namespace leggedOdometry
{

/**
 * Interface for the implementation of legged odometry. This odometry is based on the tracking of successive contacts
 * for the estimation of the pose of the floating base of the robot.
 * The tilt cannot be estimated from this method (but the yaw can), it has to be estimated beforehand by another
 * observer. One can decide to perform flat or 6D odometry. The flat odometry considers that the robot walks on a flat
 * ground and corrects the estimated height accordingly, it is preferable in this use case.
 * The odometry manager must be initialized once all the configuration parameters are retrieved using the init function,
 * and called on every iteration with \ref LeggedOdometryManager::run(const mc_control::MCController & ctl,
 * mc_rtc::Logger & logger, sva::PTransformd & pose, sva::MotionVecd & vels, sva::MotionVecd & accs).
 **/

///////////////////////////////////////////////////////////////////////
/// ------------------------------Contacts-----------------------------
///////////////////////////////////////////////////////////////////////
// Enhancement of the class ContactWithSensor with the reference of the contact in the world and the force measured by
// the associated sensor
class LoContactWithSensor : public measurements::ContactWithSensor
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  LoContactWithSensor() {}

public:
  LoContactWithSensor(int id, std::string forceSensorName)
  {
    id_ = id;
    name_ = forceSensorName;
    forceSensorName_ = forceSensorName;

    resetContact();
  }

  LoContactWithSensor(int id,
                      const std::string & forceSensorName,
                      const std::string & surfaceName,
                      bool sensorAttachedToSurface)
  {
    id_ = id;
    name_ = forceSensorName;
    resetContact();

    surface_ = surfaceName;
    forceSensorName_ = forceSensorName;
    sensorAttachedToSurface_ = sensorAttachedToSurface;
  }

public:
  // reference of the contact in the world
  stateObservation::kine::Kinematics worldRefKine_;
  // indicates whether the contact can be used for the orientation odometry or not
  bool useForOrientation_ = false;
  // norm of the force measured by the sensor
  double forceNorm_ = 0.0;
  // current estimation of the kinematics of the floating base in the world, obtained from the reference pose of the
  // contact in the world
  stateObservation::kine::Kinematics currentWorldFbPose_;
  // current estimation of the kinematics of the contact in the world
  stateObservation::kine::Kinematics currentWorldKine_;
};

class LoContactWithoutSensor : public measurements::ContactWithoutSensor
{
  // the legged odometry requires the use of contacts associated to force sensors, this class must therefore not be
  // implemented
public:
  LoContactWithoutSensor(int id, std::string name)
  {
    BOOST_ASSERT(false && "The legged odometry requires to use only contacts with sensors.");
    id_ = id;
    name_ = name;
  }

protected:
  LoContactWithoutSensor()
  {
    BOOST_ASSERT(false && "The legged odometry requires to use only contacts with sensors.");
  }
};

struct LeggedOdometryManager
{
public:
  LeggedOdometryManager() {}

protected:
  ///////////////////////////////////////////////////////////////////////
  /// ------------------------Contacts Manager---------------------------
  ///////////////////////////////////////////////////////////////////////

  typedef measurements::ContactsManager<LoContactWithSensor, LoContactWithoutSensor> ContactsManager;

  struct LeggedOdometryContactsManager : public ContactsManager
  {
  protected:
    struct sortByForce
    {
      inline bool operator()(const LoContactWithSensor & contact1, const LoContactWithSensor & contact2) const
      {
        return (contact1.forceNorm_ < contact2.forceNorm_);
      }
    };

  public:
    std::set<std::reference_wrapper<LoContactWithSensor>, sortByForce> oriOdometryContacts_;
  };

public:
  /// @brief Initializer for the odometry manager.
  /// @details Version for the contact detection using surfaces.
  /// @param ctl Controller
  /// @param robotName Name of the robot
  /// @param odometryName Name of the odometry, used in logs and in the gui.
  /// @param odometry6d Indicates if the desired odometry must be a flat or a 6D odometry.
  /// @param withYawEstimation Indicates if the orientation must be estimated by this odometry.
  /// @param contactsDetection Desired contacts detection method.
  /// @param surfacesForContactDetection Admissible surfaces for the contacts detection.
  /// @param contactsSensorDisabledInit Contacts that must not be enabled since the beginning (faulty one for example).
  /// @param contactDetectionThreshold Threshold used for the contact detection
  /// @param velUpdatedUpstream Informs whether the velocities were updated by upstream observers
  /// @param accUpdatedUpstream Informs whether the accelerations were updated by upstream observers
  void init(const mc_control::MCController & ctl,
            const std::string robotName,
            const std::string & odometryName,
            const bool odometry6d,
            const bool withYawEstimation,
            const std::string contactsDetection,
            std::vector<std::string> surfacesForContactDetection,
            std::vector<std::string> contactsSensorDisabledInit,
            const double contactDetectionThreshold,
            const bool velUpdatedUpstream,
            const bool accUpdatedUpstream);

  /// @brief Initializer for the odometry manager.
  /// @details Version for the contact detection using a thresholding on the contact force sensors measurements or by
  /// direct input from the solver.
  /// @param ctl Controller
  /// @param robotName Name of the robot
  /// @param odometryName Name of the odometry, used in logs and in the gui.
  /// @param odometry6d Indicates if the desired odometry must be a flat or a 6D odometry.
  /// @param withYawEstimation Indicates if the orientation must be estimated by this odometry.
  /// @param contactsDetection Desired contacts detection method.
  /// @param contactsSensorDisabledInit Contacts that must not be enabled since the beginning (faulty one for example).
  /// @param contactDetectionThreshold Threshold used for the contact detection
  /// @param velUpdatedUpstream Informs whether the velocities were updated by upstream observers
  /// @param accUpdatedUpstream Informs whether the accelerations were updated by upstream observers
  void init(const mc_control::MCController & ctl,
            const std::string robotName,
            const std::string & odometryName,
            const bool odometry6d,
            const bool withYawEstimation,
            const std::string contactsDetection,
            std::vector<std::string> contactsSensorDisabledInit,
            const double contactDetectionThreshold,
            const bool velUpdatedUpstream,
            const bool accUpdatedUpstream);

  /// @brief @copybrief run(const mc_control::MCController &, mc_rtc::Logger &, sva::PTransformd &, sva::MotionVecd &,
  /// sva::MotionVecd &, stateObservation::Matrix3). This version uses the tilt estimated by the upstream observers.
  /// @copydetails run(const mc_control::MCController &, mc_rtc::Logger &, sva::PTransformd &, sva::MotionVecd &,
  /// sva::MotionVecd &, stateObservation::Matrix3)
  /// @param ctl Controller
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param vels The velocities of the floating base in the world that we want to update
  /// @param accs The accelerations of the floating base in the world that we want to update
  void run(const mc_control::MCController & ctl,
           mc_rtc::Logger & logger,
           sva::PTransformd & pose,
           sva::MotionVecd & vels,
           sva::MotionVecd & accs);

  /// @brief @copybrief run(const mc_control::MCController &, mc_rtc::Logger &, sva::PTransformd &, sva::MotionVecd &,
  /// stateObservation::Matrix3). This version uses the tilt estimated by the upstream observers.
  /// @copydetails run(const mc_control::MCController &, mc_rtc::Logger &, sva::PTransformd &, sva::MotionVecd &,
  /// stateObservation::Matrix3)
  void run(const mc_control::MCController & ctl,
           mc_rtc::Logger & logger,
           sva::PTransformd & pose,
           sva::MotionVecd & vels);

  /// @brief @copybrief run(const mc_control::MCController &, mc_rtc::Logger &, sva::PTransformd &,
  /// stateObservation::Matrix3). This version uses the tilt estimated by the upstream observers.
  /// @copydetails run(const mc_control::MCController &, mc_rtc::Logger &, sva::PTransformd &, stateObservation::Matrix3)
  void run(const mc_control::MCController & ctl, mc_rtc::Logger & logger, sva::PTransformd & pose);

  /// @brief Core function runing the odometry.
  /// @param ctl Controller
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param vels The velocities of the floating base in the world that we want to update
  /// @param accs The accelerations of the floating base in the world that we want to update
  /// @param tilt The floating base's tilt (only the yaw is estimated).
  void run(const mc_control::MCController & ctl,
           mc_rtc::Logger & logger,
           sva::PTransformd & pose,
           sva::MotionVecd & vels,
           sva::MotionVecd & accs,
           const stateObservation::Matrix3 & tilt);

  /// @brief Core function runing the odometry.
  /// @param ctl Controller
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param vels The velocities of the floating base in the world that we want to update
  /// @param tilt The floating base's tilt (only the yaw is estimated).
  void run(const mc_control::MCController & ctl,
           mc_rtc::Logger & logger,
           sva::PTransformd & pose,
           sva::MotionVecd & vels,
           const stateObservation::Matrix3 & tilt);

  /// @brief Core function runing the odometry.
  /// @param ctl Controller
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param tilt The floating base's tilt (only the yaw is estimated).
  void run(const mc_control::MCController & ctl,
           mc_rtc::Logger & logger,
           sva::PTransformd & pose,
           const stateObservation::Matrix3 & tilt);

  /// @brief Updates the joints configuration of the odometry robot. Has to be called at the beginning of each
  /// iteration.
  /// @param ctl Controller
  void updateJointsConfiguration(const mc_control::MCController & ctl);

  /// @brief Updates the pose of the contacts and estimates the floating base from them.
  /// @param ctl Controller.
  /// @param logger Logger.
  /// @param updateVels Indicates if the velocities of the floating base must be updated
  /// @param updateAccs Indicates if the accelerations of the floating base must be updated
  /// @param tilt The floating base's tilt (only the yaw is estimated).
  void updateFbAndContacts(const mc_control::MCController & ctl,
                           mc_rtc::Logger & logger,
                           const bool updateVels,
                           const bool updateAccs,
                           const stateObservation::Matrix3 & tilt);

  /// @brief If the contacts respect the conditions, computes the pose of the floating base for each set contact.
  /// @details Combines the reference pose of the contact in the world and the transformation from the contact to the
  /// frame.
  /// @param ctl Controller.
  /// @param posUpdatable Indicates if the position can be updated using contacts
  /// @param oriUpdatable Indicates if the orientation can be updated using contacts
  /// @param sumForces_position Sum of the measured force of all the contacts that will be used for the position
  /// estimation
  /// @param sumForces_orientation Sum of the measured force of all the contacts that will be used for the orientation
  /// estimation
  void getFbFromContacts(const mc_control::MCController & ctl,
                         bool & posUpdatable,
                         bool & oriUpdatable,
                         double & sumForces_position,
                         double & sumForces_orientation);

  /// @brief Updates the floating base kinematics given as argument by the observer.
  /// @details Must be called after \ref updateFbAndContacts(const mc_control::MCController & ctl, mc_rtc::Logger &
  /// logger, const bool updateVels, const bool updateAccs).Beware, only the pose is updated by the odometry, the
  /// velocities (except if not updated by an upstream observer) and accelerations update only performs a
  /// transformation from the real robot to our newly estimated robot. If you want to update the accelerations of
  /// the floating base, you need to add an observer computing them beforehand.
  /// @param ctl Controller.
  /// @param updateVels If true, the velocity of the floating base of the odometry robot is updated from the one of
  /// the real robot.
  /// @param updateAccs If true, the acceleration of the floating base of the odometry robot is updated from the one
  /// of the real robot. This acceleration must be computed by an upstream observer..
  void updateOdometryRobot(const mc_control::MCController & ctl, const bool updateVels, const bool updateAccs);

  /// @brief Updates the floating base kinematics given as argument by the observer.
  /// @details Beware, only the pose is updated by the odometry, the velocities (except if not updated by an upstream
  /// observer) and accelerations update only performs a transformation from the real robot to our newly estimated
  /// robot. If you want to update the accelerations of the floating base, you need to add an observer computing them
  /// beforehand.
  /// @param ctl Controller
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param vels The velocities of the floating base in the world that we want to update.
  /// @param accs The accelerations of the floating base in the world that we want to update. This acceleration must
  /// come from an upstream observer.
  /// @param logger logger
  void updateFbKinematics(sva::PTransformd & pose, sva::MotionVecd & vels, sva::MotionVecd & accs);

  /// @brief Updates the floating base kinematics given as argument by the observer.
  /// @details Beware, only the pose is updated by the odometry, the velocities update only performs a transformation
  /// from the real robot to our newly estimated robot.
  /// @param ctl Controller
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param vels The velocities of the floating base in the world that we want to update.
  /// @param logger logger
  void updateFbKinematics(sva::PTransformd & pose, sva::MotionVecd & vels);

  /// @brief Updates the floating base kinematics given as argument by the observer.
  /// @param ctl Controller
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param logger logger
  void updateFbKinematics(sva::PTransformd & pose);

  /// @brief Computes the reference kinematics of the newly set contact in the world.
  /// @param contact The new contact
  /// @param measurementsRobot The robot containing the contact's force sensor
  void setNewContact(LoContactWithSensor & contact, const mc_rbdyn::Robot & measurementsRobot);

  /// @brief Computes the kinematics of the contact attached to the odometry robot in the world frame.
  /// @param contact Contact of which we want to compute the kinematics
  /// @param fs The force sensor associated to the contact
  /// @return stateObservation::kine::Kinematics &
  const stateObservation::kine::Kinematics & getCurrentContactKinematics(LoContactWithSensor & contact,
                                                                         const mc_rbdyn::ForceSensor & fs);

  /// @brief Select which contacts to use for the orientation odometry
  /// @details The two contacts with the highest measured force are selected. The contacts at hands are ignored because
  /// their orientation is less trustable.
  void selectForOrientationOdometry();

  /// @brief Returns the pose of the odometry robot's anchor frame based on the current floating base and encoders.
  /// @details The anchor frame can be obtained using 2 ways:
  /// - 1: contacts are detected and can be used to compute the anchor frame.
  /// - 2: no contact is detected, the robot is hanging. If we still need an anchor frame for the tilt estimation we
  /// arbitrarily use the frame of the bodySensor used by the estimator.
  /// @param ctl controller
  /// @param bodySensorName name of the body sensor.
  stateObservation::kine::Kinematics & getAnchorFramePose(const mc_control::MCController & ctl,
                                                          const std::string & bodySensorName);

  /// @brief Returns the pose of the odometry robot's anchor frame. If no contact is detected, this version does not
  /// update the anchor frame.
  /// @param ctl controller
  stateObservation::kine::Kinematics & getAnchorFramePose(const mc_control::MCController & ctl);
  /// @brief Add the log entries corresponding to the contact.
  /// @param logger
  /// @param contactName
  void addContactLogEntries(mc_rtc::Logger & logger, const LoContactWithSensor & contact);

  /// @brief Remove the log entries corresponding to the contact.
  /// @param logger
  /// @param contactName
  void removeContactLogEntries(mc_rtc::Logger & logger, const LoContactWithSensor & contact);

  /// @brief Getter for the odometry robot used for the estimation.
  mc_rbdyn::Robot & odometryRobot()
  {
    return odometryRobot_->robot("odometryRobot");
  }

  /// @brief Getter for the contacts manager.
  LeggedOdometryContactsManager & contactsManager()
  {
    return contactsManager_;
  }

public:
  // Indicates if the mode of computation of the anchor frame changed. Might me needed by the estimator (ex;
  // TiltObserver)
  bool prevAnchorFromContacts_ = true;

protected:
  // Name of the odometry, used in logs and in the gui.
  std::string odometryName_;
  // Name of the robot
  std::string robotName_;
  // Indicates if the desired odometry must be a flat or a 6D odometry.
  bool odometry6d_;
  // Indicates if the orientation must be estimated by this odometry.

  bool withYawEstimation_;
  // tracked pose of the floating base
  sva::PTransformd fbPose_ = sva::PTransformd::Identity();

protected:
  LeggedOdometryContactsManager contactsManager_;
  std::shared_ptr<mc_rbdyn::Robots> odometryRobot_;
  bool detectionFromThreshold_ = false;
  stateObservation::kine::Kinematics worldAnchorPose_;

  bool velUpdatedUpstream_ = false;
  bool accUpdatedUpstream_ = false;
};

} // namespace leggedOdometry

} // namespace mc_state_observation
