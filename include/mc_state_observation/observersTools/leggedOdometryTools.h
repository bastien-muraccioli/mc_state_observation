#pragma once

#include <mc_state_observation/observersTools/measurementsTools.h>
#include <state-observation/dynamics-estimators/kinetics-observer.hpp>

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

struct LeggedOdometryManager
{
public:
  LeggedOdometryManager() {}

private:
  class LoContactWithSensor : public measurements::ContactWithSensor
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    LoContactWithSensor() {}

  public:
    LoContactWithSensor(int id, std::string name)
    {
      id_ = id;
      name_ = name;
      resetContact();
    }

  public:
    stateObservation::kine::Kinematics worldRefKine_;
    bool useForOrientation_ = false;
    double forceNorm_ = 0.0;
    stateObservation::kine::Orientation currentWorldOrientation_;
  };

  class loContactWithoutSensor : public measurements::ContactWithoutSensor
  {
  public:
    loContactWithoutSensor(int id, std::string name)
    {
      BOOST_ASSERT(false && "The legged odometry requires to use only contacts with sensors.");
    }

  private:
    loContactWithoutSensor()
    {
      BOOST_ASSERT(false && "The legged odometry requires to use only contacts with sensors.");
    }
  };

  typedef measurements::ContactsManager<LoContactWithSensor, loContactWithoutSensor> ContactsManager;

  class LeggedOdometryContactsManager : public ContactsManager
  {
  public:
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
  void init(const mc_control::MCController & ctl,
            const std::string robotName,
            const std::string & odometryName,
            const bool odometry6d,
            const bool withNaiveYawEstimation,
            const std::string contactsDetection,
            std::vector<std::string> surfacesForContactDetection,
            std::vector<std::string> contactsSensorDisabledInit,
            const double contactDetectionThreshold)
  {
    robotName_ = robotName;
    odometry6d_ = odometry6d;
    withNaiveYawEstimation_ = withNaiveYawEstimation;
    odometryName_ = odometryName;
    const auto & realRobot = ctl.realRobot(robotName);
    X_0_fb_.translation() = realRobot.posW().translation();
    X_0_fb_.rotation() = realRobot.posW().rotation();
    contactsManager_.init(ctl, robotName, odometryName_, contactsDetection, surfacesForContactDetection,
                          contactsSensorDisabledInit, contactDetectionThreshold);
  }

  void init(const mc_control::MCController & ctl,
            const std::string robotName,
            const std::string & odometryName,
            const bool odometry6d,
            const bool withNaiveYawEstimation,
            const std::string contactsDetection,
            std::vector<std::string> contactsSensorDisabledInit,
            const double contactDetectionThreshold)
  {
    robotName_ = robotName;
    odometry6d_ = odometry6d;
    withNaiveYawEstimation_ = withNaiveYawEstimation;
    odometryName_ = odometryName;
    const auto & realRobot = ctl.realRobot(robotName);
    X_0_fb_.translation() = realRobot.posW().translation();
    X_0_fb_.rotation() = realRobot.posW().rotation();
    contactsManager_.init(ctl, robotName, odometryName_, contactsDetection, contactsSensorDisabledInit,
                          contactDetectionThreshold);
  }

  void setNewContact(const mc_rbdyn::Robot & odometryRobot, const mc_rbdyn::ForceSensor forceSensor);
  void updateContacts(const mc_control::MCController & ctl, mc_rbdyn::Robot & odometryRobot, mc_rtc::Logger & logger);

  stateObservation::kine::Kinematics getContactKinematics(LoContactWithSensor & contact,
                                                          const mc_rbdyn::Robot & robot,
                                                          const mc_rbdyn::Robot & odometryRobot);
  void selectForOrientationOdometry();
  void addContactLogEntries(mc_rtc::Logger & logger, const std::string & contactName);
  void removeContactLogEntries(mc_rtc::Logger & logger, const std::string & contactName);

  LeggedOdometryContactsManager & contactsManager()
  {
    return contactsManager_;
  }

protected:
  std::string odometryName_;
  std::string robotName_;
  bool odometry6d_;
  bool withNaiveYawEstimation_;
  // tracked pose of the floating base
  sva::PTransformd X_0_fb_ = sva::PTransformd::Identity();

private:
  LeggedOdometryContactsManager contactsManager_;
};

LeggedOdometryManager odometryManager_;

} // namespace leggedOdometry

} // namespace mc_state_observation
