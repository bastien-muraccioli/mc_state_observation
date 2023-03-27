/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#pragma once

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Robot.h>

#include <state-observation/dynamics-estimators/kinetics-observer.hpp>

#include <mc_observers/Observer.h>

namespace mc_state_observation
{
namespace so = stateObservation;
/** Flexibility observer from:
 *
 *    "Tilt estimator for 3D non-rigid pendulum based on a tri-axial
 *    accelerometer and gyrometer". Mehdi Benallegue, Abdelaziz Benallegue,
 *    Yacine Chitour. IEEE-RAS Humanoids 2017. <hal-01499167>
 *
 */

struct Contact
{
  Contact(double filteredForceZ) : filteredForceZ_(filteredForceZ) {}
  so::kine::Kinematics worldPositionKine_;
  double filteredForceZ_;
  double lambda_ = 100;
};
class MapContactsIMU
{
  /*
  Care with the use of getNameFromNum() : the mapping remains the same but the list of contacts returned buy the
  controller might differ over time (contacts broken, etc) and the id of a contact in this list might not match with its
  rank in the controller's list. Use this function preferably only to perform tasks performed on all the contacts and
  that require their name.
  */

public:
  inline const int & getNumFromName(std::string name)
  {
    return map_.find(name)->second;
  }

  inline const std::string & getNameFromNum(int num)
  {
    return insertOrder.at(num);
  }

  inline const std::vector<std::string> & getList()
  {
    return insertOrder;
  }

  inline void insertPair(std::string name)
  {
    if(map_.find(name) != map_.end()) return;
    insertOrder.push_back(name);
    map_.insert(std::make_pair(name, num));
    num++;
  }

  inline void setMaxElements(int maxElements)
  {
    // maxElements_ = maxElements_;
  }

private:
  std::vector<std::string> insertOrder;
  std::map<std::string, int> map_;
  // int maxElements_ = 4;
  int num = 0;
};

struct NaiveOdometry : public mc_observers::Observer
{

  NaiveOdometry(const std::string & type, double dt);

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

  void update(mc_control::MCController & ctl) override;

protected:
  void update(mc_rbdyn::Robot & robot);

  /*! \brief Add observer from logger
   *
   * @param category Category in which to log this observer
   */

  void plotVariablesBeforeUpdate(const mc_control::MCController & ctl, mc_rtc::Logger & logger);

  void plotVariablesAfterUpdate(const mc_control::MCController & ctl, mc_rtc::Logger & logger);

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
  /**
   * Find established contacts between the observed robot and the fixed robots
   *
   * \param ctl Controller that defines the contacts
   * \return Name of surfaces in contact with the environment
   */
  std::set<std::string> findContacts(const mc_control::MCController & solver);

  void updateContacts(const mc_rbdyn::Robot & robot,
                      const mc_rbdyn::Robot & odometryRobot,
                      std::set<std::string> contacts);

  void setNewContact(const mc_rbdyn::Robot & robot,
                     const mc_rbdyn::Robot & odometryRobot,
                     const mc_rbdyn::ForceSensor forceSensor);

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

  /** Set debug flag.
   *
   * \param flag New debug flag.
   *
   */
  inline void debug(bool flag)
  {
    debug_ = flag;
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
  sva::PTransformd zeroPose_;
  sva::MotionVecd zeroMotion_;

  std::string category_ = "NaiveOdometry";
  /* custom list of robots to display */
  std::shared_ptr<mc_rbdyn::Robots> my_robots_;

  std::set<std::string> contactsFound_; // contacts found on each iteration

  bool debug_ = false;
  bool verbose_ = true;

  double mass_ = 42; // [kg]
  // std::set<std::string> contacts_; ///< Sorted list of contacts
  std::set<std::string> oldContacts_;
  MapContactsIMU mapContacts_;

  sva::MotionVecd v_fb_0_ = sva::MotionVecd::Zero();
  sva::PTransformd X_0_fb_ = sva::PTransformd::Identity();
  sva::MotionVecd a_fb_0_ = sva::MotionVecd::Zero();

  std::unordered_map<std::string, Contact> contacts_;

  bool withFlatOdometry_ = false;
  bool withContactsDetection_ = true;
  bool withFilteredForcesContactDetection_ = false;
};

} // namespace mc_state_observation
