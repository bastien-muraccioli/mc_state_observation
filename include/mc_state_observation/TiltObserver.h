#pragma once

#include <mc_observers/Observer.h>
#include <boost/circular_buffer.hpp>
#include <mc_state_observation/observersTools/leggedOdometryTools.h>
#include <state-observation/observer/tilt-estimator.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

namespace mc_state_observation
{

struct TiltObserver : public mc_observers::Observer
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  TiltObserver(const std::string & type, double dt);

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

  void updateAnchorFrame(const mc_control::MCController & ctl, const mc_rbdyn::Robot & updatedRobot);

  void updateAnchorFrameOdometry(const mc_control::MCController & ctl);
  void updateAnchorFrameNoOdometry(const mc_control::MCController & ctl, const mc_rbdyn::Robot & updatedRobot);

  void updatePoseAndVel(const mc_control::MCController & ctl,
                        const stateObservation::Vector3 & localWorldImuLinVel,
                        const stateObservation::Vector3 & localWorldImuAngVel);

  /*! \brief update the robot pose in the world only for visualization purpose
   *
   * @param updatedRobot Robot with the kinematics of the control robot but with updated joint values.
   */
  void runTiltEstimator(const mc_control::MCController & ctl, const mc_rbdyn::Robot & updatedRobot);

  void update(mc_control::MCController & ctl) override;

  const stateObservation::kine::Kinematics backupFb(const mc_control::MCController & ctl);

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
  std::string robot_;
  bool updateRobot_ = true;
  std::string updateRobotName_;
  std::string imuSensor_;
  bool updateSensor_ = true;
  std::string updateSensorName_;
  /*!
   * parameter related to the convergence of the linear velocity
   * of the IMU expressed in the control frame
   */
  double finalAlpha_ = 50;
  ///  parameter related to the fast convergence of the tilt
  double finalBeta_ = 5;
  /// parameter related to the orthogonality
  double finalGamma_ = 15;

  /*!
   * initial value of the parameter related to the convergence of the linear velocity
   * of the IMU expressed in the control frame
   */
  double alpha_ = 50;
  /// initial value of the parameter related to the fast convergence of the tilt
  double beta_ = 5;
  /// initial value of the parameter related to the orthogonality
  double gamma_ = 15;

  std::string anchorFrameFunction_;
  stateObservation::TiltEstimator estimator_;

  // values used for computation
  stateObservation::kine::Kinematics updatedFbImuKine_;
  sva::MotionVecd imuVelC_ = sva::MotionVecd::Zero();
  sva::PTransformd X_C_IMU_ = sva::PTransformd::Identity();
  sva::PTransformd X_0_C_ = sva::PTransformd::Identity(); // control anchor frame
  sva::PTransformd X_0_C_updated_ = sva::PTransformd::Identity(); // anchor frame updated by the other observers
  sva::PTransformd X_0_C_updated_previous_ = sva::PTransformd::Identity(); // previous updated anchor frame

  sva::PTransformd newWorldAnchorPose = sva::PTransformd::Identity(); // control anchor frame

  stateObservation::kine::Kinematics::Flags::Byte flagPoseVels_ =
      stateObservation::kine::Kinematics::Flags::position | stateObservation::kine::Kinematics::Flags::orientation
      | stateObservation::kine::Kinematics::Flags::linVel | stateObservation::kine::Kinematics::Flags::angVel;

  stateObservation::kine::Kinematics worldAnchorKine_ =
      stateObservation::kine::Kinematics::zeroKinematics(flagPoseVels_);
  stateObservation::kine::Kinematics updatedWorldAnchorKine_ =
      stateObservation::kine::Kinematics::zeroKinematics(flagPoseVels_);
  stateObservation::kine::Kinematics worldFbKine_;
  stateObservation::kine::Kinematics updatedWorldFbKine_;
  stateObservation::kine::Kinematics correctedWorldImuKine_;

  stateObservation::Vector3 estimatedWorldImuLocalLinVel_;
  stateObservation::Vector3 virtualMeasureWorldImuLocalLinVel_;
  stateObservation::Vector3 updatedRobotWorldImuLocalLinVel_;
  stateObservation::Vector3 updatedRobotWorldImuLocalAngVel_;

  // result
  // The observed tilt of the sensor
  Eigen::Matrix3d estimatedRotationIMU_;

  stateObservation::Vector3 m_pF_prev;
  /// Instance of the Tilt Estimator
  stateObservation::Vector xk_;

  bool firstSample_;

private:
  std::shared_ptr<mc_rbdyn::Robots> my_robots_;

  Eigen::Matrix3d R_0_fb_; // estimated orientation of the floating base in the world frame
  sva::PTransformd poseW_ = sva::PTransformd::Identity(); ///< Estimated pose of the floating-base in world frame */
  sva::PTransformd prevPoseW_ = sva::PTransformd::Identity(); ///< Estimated pose of the floating-base in world frame */
  sva::MotionVecd velW_ = sva::MotionVecd::Zero();

  double maxAnchorFrameDiscontinuity_ =
      0.01; ///< Threshold (norm) above wich the anchor frame is considered to have had a discontinuity
  bool anchorFrameJumped_ = false; /** Detects whether the anchor frame had a discontinuity */

  int iter_ = 0;
  int itersBeforeAnchorsVel_ = 10;

  bool withOdometry_ = false;

  // Debug
  stateObservation::Vector3 x1_;
  stateObservation::kine::Kinematics updatedWorldImuKine_;
  stateObservation::kine::Kinematics worldImuKine_;
  stateObservation::kine::Kinematics updatedImuAnchorKine_ =
      stateObservation::kine::Kinematics::zeroKinematics(flagPoseVels_);
  stateObservation::kine::Kinematics fbAnchorKine_;

  stateObservation::kine::Kinematics newWorldAnchorKine_;
  stateObservation::kine::Kinematics newUpdatedWorldAnchorKine_;

  double contactDetectionPropThreshold_ = 0.11;

  boost::circular_buffer<sva::PTransformd> backupFbKinematics_ = boost::circular_buffer<sva::PTransformd>(100);

  leggedOdometry::LeggedOdometryManager odometryManager_;
};

} // namespace mc_state_observation
