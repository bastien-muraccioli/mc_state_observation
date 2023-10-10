/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#pragma once

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Robot.h>
#include <boost/circular_buffer.hpp>
#include <mc_state_observation/observersTools/measurementsTools.h>
#include <state-observation/dynamics-estimators/kinetics-observer.hpp>

#include <mc_observers/Observer.h>

namespace mc_state_observation
{
/** Interface for the use of the Kinetics Observer within mc_rtc: \n
 * The Kinetics Observer requires inputs expressed in the frame of the floating base. It then performs a conversion to
 *the centroid frame, a frame located at the center of mass of the robot and with the orientation of the floating
 *base of the real robot.
 *The inputs are obtained from a robot called the inputRobot. Its configuration is the one of real robot, but
 *its floating base's frame is superimposed with the world frame. This allows to ease computations performed in the
 *local frame of the robot.
 *The Kinetics Observer is associated to the Tilt Observer as a backup. If an anomaly is detected, the Kinetics Observer
 *will recover the last ellapsed second (or less) using the displacement made by the Tilt Observer.
 **/

struct KoContactWithSensor : public measurements::ContactWithSensor
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  KoContactWithSensor() {}

public:
  KoContactWithSensor(int id, std::string forceSensorName)
  {
    id_ = id;
    name_ = forceSensorName;
    forceSensorName_ = forceSensorName;

    resetContact();
  }

  KoContactWithSensor(int id,
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
  stateObservation::kine::Kinematics fbContactKine_;
  stateObservation::kine::Kinematics surfaceSensorKine_;
};

struct MCKineticsObserver : public mc_observers::Observer
{

  MCKineticsObserver(const std::string & type, double dt);

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

  void update(mc_control::MCController & ctl) override;

protected:
  /// @brief Update the pose and velocities of the robot in the world frame. Used only to update the ones of the robot
  /// used for the visualization of the estimation made by the Kinetics Observer.
  /// @param robot The robot to update.
  void update(mc_rbdyn::Robot & robot);

  /// @brief Initializer for the Kinetics Observer's state vector
  /// @param robot The control robot
  void initObserverStateVector(const mc_rbdyn::Robot & robot);

  /// @brief Sums up the wrenches measured by the unused force sensors expressed in the centroid frame to give them as
  /// an input to the Kinetics Observer
  /// @param inputRobot A robot whose configuration is the one of real robot, but whose pose, velocities and
  /// accelerations are set to zero in the control frame. Allows to ease computations performed in the local frame of
  /// the robot.
  /// @param measRobot The control robot
  void inputAdditionalWrench(const mc_rbdyn::Robot & inputRobot, const mc_rbdyn::Robot & measRobot);

  /// @brief Update the IMUs, including the measurements, measurement covariances and kinematics in the floating
  /// base's frame (user frame)
  /// @param measRobot The control robot
  /// @param inputRobot A robot whose configuration is the one of real robot, but whose pose, velocities and
  /// accelerations are set to zero in the control frame. Allows to ease computations performed in the local frame of
  /// the robot.
  void updateIMUs(const mc_rbdyn::Robot & measRobot, const mc_rbdyn::Robot & inputRobot);

  /*! \brief Add observer from logger
   *
   * @param category Category in which to log this observer
   */

  void plotVariablesBeforeUpdate(const mc_control::MCController & ctl, mc_rtc::Logger & logger);

  void plotVariablesAfterUpdate(mc_rtc::Logger & logger);

  /// @brief Add the logs of the desired contact.
  /// @param contactIndex The index of the contact.
  /// @param logger
  void addContactLogEntries(mc_rtc::Logger & logger, const int & contactIndex);
  /// @brief Remove the logs of the desired contact.
  /// @param contactIndex The index of the contact.
  /// @param logger
  void removeContactLogEntries(mc_rtc::Logger & logger, const int & contactIndex);

  /// @brief Add the measurements logs of the desired contact.
  /// @param contactIndex The index of the contact.
  /// @param logger
  void addContactMeasurementsLogEntries(mc_rtc::Logger & logger, const int & contactIndex);
  /// @brief Remove the measurements logs of the desired contact.
  /// @param contactIndex The index of the contact.
  /// @param logger
  void removeContactMeasurementsLogEntries(mc_rtc::Logger & logger, const int & contactIndex);

  void addToLogger(const mc_control::MCController &, mc_rtc::Logger &, const std::string & category) override;

  /*! \brief Remove observer from logger
   *
   * @param category Category in which this observer entries are logged
   */
  void removeFromLogger(mc_rtc::Logger &, const std::string & category) override;

  /*! \brief Add observer information the GUI.
   *
   * @param category Category in which to add this observer
   */
  void addToGUI(const mc_control::MCController &,
                mc_rtc::gui::StateBuilder &,
                const std::vector<std::string> & /* category */) override;

protected:
  /// @brief Updates the list of currently set contacts.
  /// @return measurements::ContactsManager<measurements::ContactWithSensor,
  /// measurements::ContactWithoutSensor>::ContactsSet &
  const measurements::ContactsManager<KoContactWithSensor, measurements::ContactWithoutSensor>::ContactsSet &
      findNewContacts(const mc_control::MCController & ctl);

  /// @brief Update the currently set contacts.
  /// @details The list of contacts is returned by \ref findNewContacts(const mc_control::MCController & ctl). Calls
  /// \ref updateContact(const mc_control::MCController & ctl, const std::string & name, mc_rtc::Logger & logger).
  /// @param contacts The list of contacts returned by \ref findNewContacts(const mc_control::MCController & ctl).
  void updateContacts(const mc_control::MCController & ctl,
                      const measurements::ContactsManager<KoContactWithSensor,
                                                          measurements::ContactWithoutSensor>::ContactsSet & contacts,
                      mc_rtc::Logger & logger);

  /// @brief Computes the kinematics of the contact attached to the robot in the world frame. Also expresses the wrench
  /// measured at the sensor in the frame of the contact.
  /// @param contact Contact of which we want to compute the kinematics
  /// @param robot robot the contacts belong to
  /// @param fs force sensor
  /// @param measuredWrench wrench measured at the sensor
  /// @return stateObservation::kine::Kinematics &
  const stateObservation::kine::Kinematics getContactWorldKinematics(KoContactWithSensor & contact,
                                                                     const mc_rbdyn::Robot & robot,
                                                                     const mc_rbdyn::ForceSensor & fs,
                                                                     const sva::ForceVecd & measuredWrench);

  /// @brief Computes the kinematics of the contact attached to the robot in the world frame.
  /// @param contact Contact of which we want to compute the kinematics
  /// @param robot robot the contacts belong to
  /// @param fs force sensor
  /// @return stateObservation::kine::Kinematics &
  const stateObservation::kine::Kinematics getContactWorldKinematics(KoContactWithSensor & contact,
                                                                     const mc_rbdyn::Robot & robot,
                                                                     const mc_rbdyn::ForceSensor & fs);

  void updateContactForceMeasurement(KoContactWithSensor & contact,
                                     stateObservation::kine::Kinematics surfaceSensorKine,
                                     const sva::ForceVecd & measuredWrench);

  void updateContactForceMeasurement(KoContactWithSensor & contact, const sva::ForceVecd & measuredWrench);

  void getOdometryWorldContactReference(const mc_control::MCController & ctl,
                                        KoContactWithSensor & contact,
                                        stateObservation::kine::Kinematics & worldContactKineRef);

  /// @brief Update the contact or create it if it still does not exist.
  /// @details Called by \ref updateContacts(const mc_control::MCController & ctl, std::set<std::string> contacts,
  /// mc_rtc::Logger & logger).
  /// @param name The name of the contact to update.
  void updateContact(const mc_control::MCController & ctl, const int & contactIndex, mc_rtc::Logger & logger);

  stateObservation::kine::Kinematics updateContactsPoseFromFb(
      const stateObservation::kine::Kinematics & currentFbWorld,
      const stateObservation::kine::Kinematics & currentWorldContactRef,
      const stateObservation::kine::Kinematics & newWorldFbKine);

protected:
  std::string robot_ = "";
  // std::string imuSensor_ = "";
  mc_rbdyn::BodySensorVector IMUs_; ///< list of IMUs

public:
  /** Get robot mass.
   *
   */
  inline double mass() const
  {
    return mass_;
  }

  /** Set robot mass.
   *
   * \param mass Robot mass.
   *
   */
  void mass(double mass);

  /** Set stiffness of the robot-environment flexibility.
   *
   * \param stiffness Flexibility stiffness.
   *
   */
  void flexStiffness(const sva::MotionVecd & stiffness);

  /** Set damping of the robot-environment flexibility.
   *
   * \param damping Flexibility damping.
   *
   */
  void flexDamping(const sva::MotionVecd & damping);

  /** Update measurement-noise covariance matrix.
   *
   */
  void updateNoiseCovariance();

  /** Get accelerometer measurement noise covariance.
   *
   */
  inline double accelNoiseCovariance() const
  {
    return acceleroSensorCovariance_(0, 0);
  }

  /** Change accelerometer measurement noise covariance.
   *
   * \param covariance New covariance.
   *
   */
  inline void accelNoiseCovariance(double covariance)
  {
    acceleroSensorCovariance_ = stateObservation::Matrix3::Identity() * covariance;
    updateNoiseCovariance();
  }

  /** Set debug flag.
   *
   * \param flag New debug flag.
   *
   */
  inline void debug(bool flag)
  {
    debug_ = flag;
  }

  /** Get force-sensor measurement noise covariance.
   *
   */
  inline double forceSensorNoiseCovariance() const
  {
    return contactSensorCovariance_(0, 0);
  }

  /** Change force-sensor measurement noise covariance.
   *
   * \param covariance New covariance.
   *
   */
  inline void forceSensorNoiseCovariance(double covariance)
  {
    contactSensorCovariance_.block<3, 3>(0, 0) = stateObservation::Matrix3::Identity() * covariance;
    updateNoiseCovariance();
  }

  /** Get gyrometer measurement noise covariance.
   *
   */
  inline double gyroNoiseCovariance() const
  {
    return gyroSensorCovariance_(0, 0);
  }

  /** Change gyrometer measurement noise covariance.
   *
   * \param covariance New covariance.
   *
   */
  inline void gyroNoiseCovariance(double covariance)
  {
    gyroSensorCovariance_ = stateObservation::Matrix3::Identity() * covariance;
    updateNoiseCovariance();
  }

  /*
  // Get last input vector sent to observer.
  inline const Eigen::VectorXd & inputs() const
  {
    return inputs_;
  }
  */

  /** Get last measurement vector sent to observer.
   *
   */
  inline const Eigen::VectorXd measurements() const
  {
    return observer_.getEKF().getLastMeasurement();
  }

  /** Floating-base transform estimate.
   *
   */
  inline const sva::PTransformd & posW() const
  {
    return X_0_fb_;
  }

  /** Floating-base velocity estimate.
   *
   */
  inline const sva::MotionVecd & velW() const
  {
    return v_fb_0_;
  }

private:
  // zero frame transformation

  sva::PTransformd zeroPose_;
  // zero velocity or acceleration
  sva::MotionVecd zeroMotion_;

  // kinematics of the CoM within the world frame of the control robot
  stateObservation::kine::Kinematics worldCoMKine_;

  std::string category_ = "MCKineticsObserver";
  /* custom list of robots to display */
  std::shared_ptr<mc_rbdyn::Robots> my_robots_;

  // the Kinetics Observer completed the loop at least once
  bool ekfIsSet_ = false;

  // state vector resulting from the Kinetics Observer esimation
  Eigen::VectorXd res_;

  // rate to apply on the measured force to obtain the threshold for contact detection.
  double contactDetectionPropThreshold_ = 0.0;
  // threshold on the measured force for contact detection.
  double contactDetectionThreshold_ = 0.0;

  // For logs only. Prediction of the measurements from the newly corrected state
  stateObservation::Vector correctedMeasurements_;
  // For logs only. Kinematics of the centroid frame within the world frame
  stateObservation::kine::Kinematics globalCentroidKinematics_;

  bool debug_ = false;
  bool verbose_ = true;

  double mass_ = 42; // [kg]

  // instance of the Kinetics Observer
  stateObservation::KineticsObserver observer_;

  // velocity of the floating base within the world frame (real one, not the one of the control robot)
  sva::MotionVecd v_fb_0_ = sva::MotionVecd::Zero();
  // pose of the floating base within the world frame (real one, not the one of the control robot)
  sva::PTransformd X_0_fb_ = sva::PTransformd::Identity();
  // acceleration of the floating base within the world frame (real one, not the one of the control robot)
  sva::MotionVecd a_fb_0_ = sva::MotionVecd::Zero();
  /**< grouped inertia */
  sva::RBInertiad inertiaWaist_;

  // total force measured by the sensors that are not associated to a currently set contact and expressed in the
  // floating base's frame. Used as an input for the Kinetics Observer.
  stateObservation::Vector3 additionalUserResultingForce_ = stateObservation::Vector3::Zero();
  // total torque measured by the sensors that are not associated to a currently set contact and expressed in the
  // floating base's frame. Used as an input for the Kinetics Observer.
  stateObservation::Vector3 additionalUserResultingMoment_ = stateObservation::Vector3::Zero();

  // this variable is set to true when the robot touches the ground at the beginning of the simulation. Checks that
  // contacts are detected before running the estimator.
  bool simStarted_ = false;

  /* Config variables */

  // linear stiffness of contacts
  stateObservation::Matrix3 linStiffness_;
  // linear damping of contacts
  stateObservation::Matrix3 linDamping_;
  // angular stiffness of contacts
  stateObservation::Matrix3 angStiffness_;
  // linear damping of contacts
  stateObservation::Matrix3 angDamping_;

  // indicates if the debug logs have to be added.
  bool withDebugLogs_ = true;
  // indicates if we want to perform odometry along the stabilization or not.
  bool withOdometry_ = false;
  // associated to @withOdometry_. If true, the odometry on the position will be only along the x and y axes. If
  // false, the default 6D odometry is used.
  bool withFlatOdometry_ = false;

  // indicates if the forces measurement have to be filtered with a low-pass filter.
  bool withFilteredForcesContactDetection_ = false;

  // indicates if we want to estimate the unmodeled wrench within the Kinetics Observer.
  bool withUnmodeledWrench_ = true;
  // indicates if we want to estimate the bias on the gyrometer measurement within the Kinetics Observer.
  bool withGyroBias_ = true;

  // maximum amount of contacts that we want to use with the Kinetics Observer.
  int maxContacts_ = 4;
  // maximum amount of IMUs that we want to use with the Kinetics Observer.
  int maxIMUs_ = 2;

  stateObservation::Matrix3 statePositionInitCovariance_;
  stateObservation::Matrix3 stateOriInitCovariance_;
  stateObservation::Matrix3 stateLinVelInitCovariance_;
  stateObservation::Matrix3 stateAngVelInitCovariance_;
  stateObservation::Matrix3 gyroBiasInitCovariance_;
  stateObservation::Matrix6 unmodeledWrenchInitCovariance_;
  stateObservation::Matrix12 contactInitCovarianceFirstContacts_;
  stateObservation::Matrix12 contactInitCovarianceNewContacts_;

  stateObservation::Matrix3 statePositionProcessCovariance_;
  stateObservation::Matrix3 stateOriProcessCovariance_;
  stateObservation::Matrix3 stateLinVelProcessCovariance_;
  stateObservation::Matrix3 stateAngVelProcessCovariance_;
  stateObservation::Matrix3 gyroBiasProcessCovariance_;
  stateObservation::Matrix6 unmodeledWrenchProcessCovariance_;
  stateObservation::Matrix12 contactProcessCovariance_;

  stateObservation::Matrix3 positionSensorCovariance_;
  stateObservation::Matrix3 orientationSensorCoVariance_;
  stateObservation::Matrix3 acceleroSensorCovariance_;
  stateObservation::Matrix3 gyroSensorCovariance_;
  stateObservation::Matrix6 contactSensorCovariance_;

  measurements::ContactsManager<KoContactWithSensor, measurements::ContactWithoutSensor> contactsManager_;
  measurements::MapIMUs mapIMUs_;

  int lastBackupIter_ = 0;
  int backupIterInterval_ = 0;

  // time during which the Kinetics Observer is still getting updated by the Tilt Observer after the need of a backup,
  // so the Kalman Filter has time to converge again
  int invincibilityFrame_ = 0;
  int invincibilityIter_ = 0;

  boost::circular_buffer<stateObservation::kine::Kinematics> koBackupFbKinematics_ =
      boost::circular_buffer<stateObservation::kine::Kinematics>(100);
  // std::queue<stateObservation::kine::Kinematics> backupFbKinematics_;

  bool contactsDetectionFromThreshold_ = false;

  /* Config variables */
};

} // namespace mc_state_observation
