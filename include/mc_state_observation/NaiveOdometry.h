/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#pragma once

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Robot.h>

#include <mc_state_observation/observersTools/leggedOdometryTools.h>
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

struct Sensor
{

public:
  inline const int & getID()
  {
    return id_;
  }

protected:
  Sensor() {}
  ~Sensor() {}
  Sensor(int id, std::string name) : id_(id), name_(name) {}

protected:
  int id_;
  std::string name_;
};

struct IMU : virtual public Sensor
{
public:
  IMU(int id, std::string name)
  {
    id_ = id;
    name_ = name;
    gyroBias = so::Vector3::Zero();
  }
  ~IMU() {}

public:
  so::Vector3 gyroBias;
};

struct Contact : virtual public Sensor
{
protected:
  Contact() {}
  ~Contact() {}
  Contact(int id, std::string name)
  {
    id_ = id;
    name_ = name;
    zmp = so::Vector3::Zero();
    resetContact();
  }

public:
  inline void resetContact()
  {
    wasAlreadySet_ = false;
    isSet_ = false;
  }
  inline const so::Vector3 & getZMP()
  {
    return zmp;
  }

public:
  bool isSet_ = false;
  bool wasAlreadySet_ = false;
  so::Vector3 zmp;
  so::kine::Kinematics worldRefKine_;
};

struct ContactWithSensor : virtual public Contact
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  ContactWithSensor(int id, std::string name)
  {
    id_ = id;
    name_ = name;
    resetContact();
  }
  ~ContactWithSensor() {}
  inline void resetContact()
  {
    wasAlreadySet_ = false;
    isSet_ = false;

    // also filtered force? see when this feature will be corrected
  }

public:
  bool isAttachedToSurface = true;
  std::string surface;

  /* Force filtering for the contact detection */
  so::Vector3 filteredForce = so::Vector3::Zero();
  double lambda = 0.0;
};

struct ContactWithoutSensor : virtual public Contact
{
public:
  ContactWithoutSensor(int id, std::string name)
  {
    id_ = id;
    name_ = name;
  }
  ~ContactWithoutSensor() {}
};

struct MapContacts
{
public:
  inline ContactWithSensor & contactWithSensor(const std::string & name)
  {
    return mapContactsWithSensors_.at(name);
  }

  inline ContactWithSensor & contactWithSensor(const int & num)
  {
    return mapContactsWithSensors_.at(getNameFromNum(num));
  }

  inline std::map<std::string, ContactWithSensor> & contactsWithSensors()
  {
    return mapContactsWithSensors_;
  }

  inline const std::string & getNameFromNum(const int & num)
  {
    return insertOrder.at(num);
  }

  inline const std::vector<std::string> & getList()
  {
    return insertOrder;
  }

  inline const int & getNumFromName(const std::string & name)
  {
    return mapContactsWithSensors_.at(name).getID();
  }

  inline const so::Vector3 & getZMPFromName(const std::string & name)
  {
    return mapContactsWithSensors_.at(name).getZMP();
  }

  inline bool hasElement(const std::string & element)
  {
    return mapContactsWithSensors_.find(element) != mapContactsWithSensors_.end();
  }

  inline void insertContact(const std::string & name)
  {
    if(checkAlreadyExists(name)) return;
    insertOrder.push_back(name);
    insertElement(name);

    num++;
  }

private:
  inline void insertElement(const std::string & name)
  {
    mapContactsWithSensors_.insert(std::make_pair(name, ContactWithSensor(num, name)));
  }
  inline virtual bool checkAlreadyExists(const std::string & name)
  {
    if(mapContactsWithSensors_.find(name) != mapContactsWithSensors_.end())
    {
      return true;
    }
    else
    {
      return false;
    }
  }

private:
  std::map<std::string, ContactWithSensor> mapContactsWithSensors_;
  std::vector<std::string> insertOrder;
  int num = 0;
};

struct MapIMUs
{
public:
  inline const int & getNumFromName(const std::string & name)
  {
    return mapIMUs_.find(name)->second.getID();
  }
  inline bool hasIMU(const std::string & element)
  {
    return mapIMUs_.find(element) != mapIMUs_.end();
  }

  inline const std::string & getNameFromNum(const int & num)
  {
    return insertOrder.at(num);
  }

  inline const std::vector<std::string> & getList()
  {
    return insertOrder;
  }
  inline virtual bool hasElement(const std::string & name)
  {
    checkAlreadyExists(name);
  }

  inline void insertIMU(std::string name)
  {
    if(checkAlreadyExists(name)) return;
    insertOrder.push_back(name);
    mapIMUs_.insert(std::make_pair(name, IMU(num, name)));
    num++;
  }

  inline IMU & operator()(std::string name)
  {
    BOOST_ASSERT(checkAlreadyExists(name) && "The requested sensor doesn't exist");
    return mapIMUs_.at(name);
  }

private:
  inline virtual bool checkAlreadyExists(const std::string & name)
  {
    if(mapIMUs_.find(name) != mapIMUs_.end())
      return true;
    else
      return false;
  }

private:
  std::vector<std::string> insertOrder;
  std::map<std::string, IMU> mapIMUs_;
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

  void addContactLogEntries(mc_rtc::Logger & logger, const std::string & contactName);

  void removeContactLogEntries(mc_rtc::Logger & logger, const std::string & contactName);

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

  /*
    void updateContacts(const mc_control::MCController & ctl,
                        mc_rbdyn::Robot & odometryRobot,
                        std::set<std::string> contacts,
                        mc_rtc::Logger & logger);

    void setNewContact(const mc_rbdyn::Robot & odometryRobot, const mc_rbdyn::ForceSensor forceSensor);
  */
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
  so::Vector6 contactWrenchVector_;
  std::string contactsDetection_;
  std::vector<std::string> surfacesForContactDetection_;
  sva::PTransformd zeroPose_;
  sva::MotionVecd zeroMotion_;

  std::string category_ = "NaiveOdometry_";
  /* custom list of robots to display */
  std::shared_ptr<mc_rbdyn::Robots> my_robots_;

  std::set<std::string> contactsFound_; // contacts found on each iteration

  bool debug_ = false;
  bool verbose_ = true;

  double contactDetectionPropThreshold_ = 0.11;

  double mass_ = 42; // [kg]
  // std::set<std::string> contacts_; ///< Sorted list of contacts
  std::set<std::string> oldContacts_;
  MapContacts mapContacts_;

  sva::MotionVecd v_fb_0_ = sva::MotionVecd::Zero();
  sva::PTransformd X_0_fb_ = sva::PTransformd::Identity();
  sva::MotionVecd a_fb_0_ = sva::MotionVecd::Zero();

  bool withFlatOdometry_ = false;
  bool withNaiveYawEstimation_ = true;
  bool withContactsDetection_ = true;
  bool withFilteredForcesContactDetection_ = false;
  leggedOdometry::LeggedOdometryManager odometryManager_;
};

} // namespace mc_state_observation
