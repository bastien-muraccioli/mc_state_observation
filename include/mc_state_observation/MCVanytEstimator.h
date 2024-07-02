#pragma once

#include <boost/circular_buffer.hpp>

#include <forward_list>
#include <mc_state_observation/odometry/LeggedOdometryManager.h>
#include <state-observation/observer/vanyt-estimator.hpp>

namespace mc_state_observation
{
/// @brief Buffer containing all the variables necessary to use a-posteriori a delayed orientation measurement
/// @details Given k the time of beginning of acquisition of the measurement (ex: the acquisition of the image used to
/// get the orientation), we need to store the state at time k-1, the measurements at time k, and the state at time k
/// that was obtained without the orientation measurement.
struct DelayedOriMeasBufferedIter
{
  /// @brief Buffer containing the measurements coming from the contact position.
  struct ContactPosMeasurement
  {
    ContactPosMeasurement(stateObservation::Vector3 worldContactRefPos,
                          stateObservation::Vector3 imuContactPos,
                          double lambda,
                          double gamma)
    : worldContactRefPos_(worldContactRefPos), imuContactPos_(imuContactPos), lambda_(lambda), gamma_(gamma)
    {
    }

  public:
    stateObservation::Vector3 worldContactRefPos_;
    stateObservation::Vector3 imuContactPos_;
    double lambda_;
    double gamma_;
  };

  /// @brief Buffer containing direct (non-delayed) orientation measurements.
  struct OriDirectMeasurement
  {
    OriDirectMeasurement(stateObservation::kine::Orientation measuredOri, double gain)
    : measuredOri_(measuredOri), gain_(gain)
    {
    }

  public:
    stateObservation::kine::Orientation measuredOri_;
    double gain_;
  };

public:
  // alpha: gain for the convergence of x1 at the initial iteration.
  // beta: gain for the convergence of x2_prime at the initial iteration.
  // rho: gain related to the convergence of x2 with respect of the orthogonality at the initial iteration.
  std::array<double, 3> gains;
  // state at time k-1
  stateObservation::Vector initState_;
  // measurements at time k
  stateObservation::Vector initMeas_;
  // list containing all the measurements coming from the contact positions.
  std::forward_list<ContactPosMeasurement> contactPosMeasurements_;
  // list containing all the direct (non-delayed) orientation measurements.
  std::forward_list<OriDirectMeasurement> oriDirectMeasurements_;

  // state at time k without the orientation measurement
  stateObservation::Vector estWithoutOri_;
};

struct MCVanytEstimator : public mc_observers::Observer
{
  // we define MCKineticsObserver as a friend as it can instantiate this observer as a backup
  friend struct MCKineticsObserver;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  /// @brief Constructor for the MCVanytEstimator.
  /// @details The parameters asBackup and observerName are given only if the Tilt Observer is used as a backup by the
  /// Kinetics Observer
  MCVanytEstimator(const std::string & type,
                   double dt,
                   bool asBackup = false,
                   const std::string & observerName = "MCVanytEstimator");

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

  /**
   * @brief Updates the frames that are necessary for the state estimation.
   * @details In particular the kinematics of the anchor in the IMU frame.
   *
   * @param ctl Controller.
   * @param odomRobot
   */
  void updateNecessaryFramesOdom(const mc_control::MCController & ctl, const mc_rbdyn::Robot & odomRobot);

  /// @brief updates the pose and the velcoity of the floating base in the world frame using our estimation results
  /// @param localWorldImuLinVel estimated local linear velocity of the IMU in the world frame
  /// @param localWorldImuAngVelestimated measurement of the gyrometer
  void updatePoseAndVel(const stateObservation::Vector3 & localWorldImuLinVel,
                        const stateObservation::Vector3 & localWorldImuAngVel);

  /*! \brief update the robot pose in the world only for visualization purpose
   *
   * @param odomRobot Robot with the kinematics of the control robot but with updated joint values.
   */
  void runTiltEstimator(const mc_control::MCController & ctl, const mc_rbdyn::Robot & odomRobot);

  /// @brief Updates the real robot and/or the IMU signal using our estimation results
  /// @param ctl Controller
  void update(mc_control::MCController & ctl) override;

  /// @brief Sets the type of the odometry
  /// @param newOdometryType The new type of odometry to use.
  void setOdometryType(measurements::OdometryType newOdometryType);

  /// @brief Backup function that returns the estimated displacement of the floating base in the world wrt to the
  /// initial one over the backup interval.
  /// @param koBackupFbKinematics Buffer containing the pose of the floating base in the world estimated by the Kinetics
  /// Observer over the whole backup interval.
  /// @return const stateObservation::kine::Kinematics
  const stateObservation::kine::Kinematics backupFb(
      boost::circular_buffer<stateObservation::kine::Kinematics> * koBackupFbKinematics);

  /// @brief Re-estimates the current state using a delayed orientation measurement.
  /// @details Let us denote k the time on which the orientation measurement started to be computed, but is still not
  /// available. We replay the estimation at time k using the buffered state and measurements, this time using the newly
  /// available orientation measurement. We then apply the transformation between the time k+1 and the current
  /// iteration.
  /// @param delayedOriMeas The delayed orientation measurement.
  /// @param delayIters Number of iterations corresponding to the measurement delay.
  /// @param delayedOriGain The gain associated to the delayed orientation within the filter.
  void delayedOriMeasurementHandler(const stateObservation::Matrix3 & delayedOriMeas,
                                    unsigned long delayIters,
                                    double delayedOriGain);

protected:
  /*! \brief update the robot pose in the world only for visualization purpose
   *
   * @param robot Robot to update
   */

  void update(mc_rbdyn::Robot & robot);

  /*! \brief Add observer from logger
   *
   * @param category Category in which to log this observer
   */
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
  // name of the observer
  std::string observerName_;

  // container for our robots
  std::shared_ptr<mc_rbdyn::Robots> my_robots_;

  std::string robot_; // name of the robot
  bool updateRobot_ = true; // indicates whether we use our estimation to update the real robot or not
  std::string imuSensor_; // IMU used for the estimation
  bool updateSensor_ = true; // indicates whether we update the IMU signal or not

  /*!
   * parameter related to the convergence of the linear velocity
   * of the IMU expressed in the control frame
   */
  double finalAlpha_ = 5;
  ///  parameter related to the fast convergence of the tilt
  double finalBeta_ = 1;
  /// parameter related to the orthogonality
  double finalRho_ = 2;

  /*!
   * initial value of the parameter related to the convergence of the linear velocity
   * of the IMU expressed in the control frame
   */
  double alpha_ = 5;
  /// initial value of the parameter related to the fast convergence of the tilt
  double beta_ = 1;
  /// initial value of the parameter related to the orthogonality
  double rho_ = 2;

  // flag indicating the variables we want in the resulting Kinematics object
  stateObservation::kine::Kinematics::Flags::Byte flagPoseVels_ =
      stateObservation::kine::Kinematics::Flags::position | stateObservation::kine::Kinematics::Flags::orientation
      | stateObservation::kine::Kinematics::Flags::linVel | stateObservation::kine::Kinematics::Flags::angVel;

  // function used to compute the anchor frame of the robot in the world.
  std::string anchorFrameFunction_;
  // instance of the Tilt Estimator for humanoid robots.
  stateObservation::VanytEstimator estimator_;

  /* kinematics used for computation */
  // kinematics of the IMU in the floating base after the encoders update
  stateObservation::kine::Kinematics fbImuKine_;
  // kinematics of the floating base in the world after the encoders update
  stateObservation::kine::Kinematics worldFbKine_;
  // kinematics of the anchor frame in the IMU frame after the encoders update
  stateObservation::kine::Kinematics imuAnchorKine_;
  // kinematics of the IMU in the world after the encoders update
  stateObservation::kine::Kinematics worldImuKine_;

  /* Estimation results */

  // The observed tilt of the sensor
  Eigen::Matrix3d estimatedRotationIMU_;
  /// State vector estimated by the Tilt Observer
  stateObservation::Vector xk_;
  // estimated kinematics of the floating base in the world
  stateObservation::kine::Kinematics correctedWorldFbKine_;
  // estimated kinematics of the IMU in the world
  stateObservation::kine::Kinematics correctedWorldImuKine_;

  /* Floating base's kinematics */
  Eigen::Matrix3d R_0_fb_; // estimated orientation of the floating base in the world frame
  sva::PTransformd poseW_; ///< Estimated pose of the floating-base in world frame */
  sva::PTransformd prevPoseW_; ///< Estimated pose of the floating-base in world frame */
  sva::MotionVecd velW_; ///< Estimated velocity of the floating-base in world frame */

  // anchor frame's variables
  double maxAnchorFrameDiscontinuity_ =
      0.01; ///< Threshold (norm) above wich the anchor frame is considered to have had a discontinuity
  bool anchorFrameJumped_; /** Detects whether the anchor frame had a discontinuity */
  int iter_; // iterations ellapsed since the beginning of the  estimation. We don't compute the anchor frame
             // velocity while it is below "itersBeforeAnchorsVel_"
  int itersBeforeAnchorsVel_ = 10; // iteration from which we start to compute the velocity of the anchor frame. Avoids
                                   // initial jumps due to the finite differences.

  /* Odometry parameters */
  odometry::LeggedOdometryManager odometryManager_; // manager for the legged odometry

  double contactDetectionThreshold_; // threshold used for the contacts detection

  /* Variables for the use as a backup */
  // indicates if the estimator is used as a backup or not
  bool asBackup_ = false;
  // Buffer containing the estimated pose of the floating base in the world over the whole backup interval.
  boost::circular_buffer<stateObservation::kine::Kinematics> backupFbKinematics_;

  // Buffer containing the estimated pose of the floating base in the world over the whole backup interval.
  boost::circular_buffer<DelayedOriMeasBufferedIter> delayedOriMeasBuffer_;
  unsigned long delayedOriBufferCapacity_;

  /* Debug variables */
  // "measured" local linear velocity of the IMU
  stateObservation::Vector3 yv_;
  // velocity of the IMU in the anchor frame
  sva::MotionVecd imuVelC_;
  // pose of the IMU in the anchor frame
  sva::PTransformd X_C_IMU_;

  stateObservation::kine::Orientation measuredOri_ = stateObservation::kine::Orientation::zeroRotation();
  stateObservation::Vector measurements_;

  double mu_contacts_ = 2;
  double mu_gyroscope_ = 2;
  double lambda_contacts_ = 2;
  double gamma_contacts_ = 2;
};

} // namespace mc_state_observation
