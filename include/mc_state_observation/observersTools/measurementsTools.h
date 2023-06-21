#include <mc_control/MCController.h>
#include <mc_observers/Observer.h>
#include <mc_rbdyn/Robot.h>

namespace mc_state_observation
{
namespace measurements
{

///////////////////////////////////////////////////////////////////////
/// -----------------------------Sensors-------------------------------
///////////////////////////////////////////////////////////////////////
struct Sensor
{
public:
  inline const int & getID()
  {
    return id_;
  }
  inline const std::string & getName()
  {
    return name_;
  }

protected:
  Sensor() {}
  ~Sensor() {}
  Sensor(int id, std::string name) : id_(id), name_(name) {}

protected:
  int id_;
  std::string name_;
};

///////////////////////////////////////////////////////////////////////
/// --------------------------------IMUs-------------------------------
///////////////////////////////////////////////////////////////////////

/* Contains the important variables associated to the IMU */
struct IMU : public Sensor
{
public:
  IMU(int id, std::string name)
  {
    id_ = id;
    name_ = name;
    gyroBias << 0.0, 0.0, 0.0;
  }
  ~IMU() {}

public:
  Eigen::Vector3d gyroBias;
};

struct MapIMUs
{
public:
  /// @brief Get the index of the IMU given its name.
  /// @param name The name of the IMU
  /// @return const int &
  inline const int & getNumFromName(const std::string & name)
  {
    return mapIMUs_.find(name)->second.getID();
  }
  /// @brief Get the name of the IMU given its index.
  /// @param num_ The index of the IMU
  /// @return const std::string &
  inline const std::string & getNameFromNum(const int & num_)
  {
    return insertOrder_.at(num_);
  }

  /// @brief Get the list of all the IMUs.
  /// @return const std::vector<std::string> &
  inline const std::vector<std::string> & getList()
  {
    return insertOrder_;
  }

  /// @brief Checks if the required IMU exists.
  /// @param name The name of the IMU
  /// @return bool
  inline bool hasElement(const std::string & name)
  {
    return checkAlreadyExists(name);
  }

  /// @brief Inserts an IMU to the map.
  /// @param name The name of the IMU
  inline void insertIMU(std::string name)
  {
    if(checkAlreadyExists(name)) return;
    insertOrder_.push_back(name);
    mapIMUs_.insert(std::make_pair(name, IMU(num_, name)));
    num_++;
  }

  /// @brief Accessor for an IMU in the list.
  /// @param name The name of the IMU
  /// @return bool
  inline IMU & operator()(std::string name)
  {
    BOOST_ASSERT(checkAlreadyExists(name) && "The requested sensor doesn't exist");
    return mapIMUs_.at(name);
  }

private:
  /// @brief Checks if the required IMU exists.
  /// @param name The name of the IMU
  /// @return bool
  inline bool checkAlreadyExists(const std::string & name)
  {
    return mapIMUs_.find(name) != mapIMUs_.end();
  }

private:
  // list of all the IMUs.
  std::vector<std::string> insertOrder_;
  // map associating all the IMUs to their names.
  std::map<std::string, IMU> mapIMUs_;
  // Index generator, incremented everytime a new IMU is added
  int num_ = 0;
};

///////////////////////////////////////////////////////////////////////
/// ------------------------------Contacts-----------------------------
///////////////////////////////////////////////////////////////////////

/* Contains the important variables associated to the contact */

struct Contact : public Sensor
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  Contact() {}
  ~Contact() {}
  Contact(int id, std::string name)
  {
    id_ = id;
    name_ = name;
    resetContact();
  }

public:
  inline void resetContact()
  {
    wasAlreadySet = false;
    isSet = false;
  }
  inline const Eigen::Vector3d & getZMP()
  {
    return zmp;
  }

public:
  bool isSet = false;
  bool wasAlreadySet = false;
  Eigen::Vector3d zmp;
};

struct ContactWithSensor : public Contact
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  

public:
ContactWithSensor() {}
  ContactWithSensor(int id, std::string name)
  {
    id_ = id;
    name_ = name;
    resetContact();
  }
  ~ContactWithSensor() {}
  inline void resetContact()
  {
    wasAlreadySet = false;
    isSet = false;
    sensorWasEnabled = false;

    // also filtered force? see when this feature will be corrected
  }

public:
  Eigen::Matrix<double, 6, 1> wrenchInCentroid = Eigen::Matrix<double, 6, 1>::Zero(); // for debug only
  double normForce = 0.0; // for debug only
  // the measurement of the sensor has to be considered as an input for the Kinetics Observer
  bool isExternalWrench = true;
  // the sensor measurement have to be used in the correction by the Kinetics Observer
  bool sensorEnabled = true;
  // allows to know if the contact's measurements have to be added during the update
  bool sensorWasEnabled = false;

  // indicates if the sensor is directly attached to a surface (true) or not (false). Default is true because in the
  // case of detection of contacts by thresholding the measured force (@contactsDetection_ = fromThreshold), we cannot
  // know precisely the surface of contact, so we will consider that the kinematics of the contact surface are the
  // ones of the sensor
  bool sensorAttachedToSurface = true;
  // surface of contact
  std::string surface;

  /* Force filtering for the contact detection */
  Eigen::Vector3d filteredForce = Eigen::Vector3d::Zero();
  double lambda = 0.0;
};

struct ContactWithoutSensor : public Contact
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
  /// @brief Accessor for the a contact associated to a sensor contained in the map
  ///
  /// @param name The name of the contact to access
  /// @return ContactWithSensor&
  inline ContactWithSensor & contactWithSensor(const std::string & name)
  {
    return mapContactsWithSensors_.at(name);
  }
  /// @brief Accessor for the a contact associated to a sensor contained in the map
  ///
  /// @param num_ The index of the contact to access
  /// @return ContactWithSensor&
  inline ContactWithSensor & contactWithSensor(const int & num_)
  {
    return mapContactsWithSensors_.at(getNameFromNum(num_));
  }

  /// @brief Accessor for the a contact that is not associated to a sensor contained in the map
  /// @param name The name of the contact to access
  /// @return ContactWithoutSensor&
  inline ContactWithoutSensor & contactWithoutSensor(const std::string & name)
  {
    return mapContactsWithoutSensors_.at(name);
  }

  /// @brief Accessor for the a contact that is not associated to a sensor contained in the map
  /// @param num_ The index of the contact to access
  /// @return ContactWithoutSensor&
  inline ContactWithoutSensor & contactWithoutSensor(const int & num_)
  {
    return mapContactsWithoutSensors_.at(getNameFromNum(num_));
  }

  /// @brief Get the map of all the contacts associated to a sensor
  ///
  /// @return std::map<std::string, ContactWithSensor>&
  inline std::map<std::string, ContactWithSensor> & contactsWithSensors()
  {
    return mapContactsWithSensors_;
  }
  /// @brief Get the map of all the contacts that are not associated to a sensor
  ///
  /// @return std::map<std::string, ContactWithSensor>&
  inline std::map<std::string, ContactWithoutSensor> & contactsWithoutSensors()
  {
    return mapContactsWithoutSensors_;
  }

  /// @brief Get the list of all the contacts (with and without sensors)
  ///
  /// @return const std::vector<std::string> &
  inline const std::vector<std::string> & getList()
  {
    return insertOrder_;
  }

  /// @brief Returns true if the contact is associated to a sensor
  ///
  /// @param name The index of the contact to access
  /// @return bool
  inline bool hasSensor(const std::string & element)
  {
    BOOST_ASSERT(hasElement(element) && "This contact does not belong to the list.");
    return hasSensor_.at(element);
  }

  /// @brief Get the name of a contact given its index
  ///
  /// @param num_ The index of the contact
  /// @return const std::string &
  inline const std::string & getNameFromNum(const int & num_)
  {
    return insertOrder_.at(num_);
  }

  /// @brief Get the index of a contact given its name
  ///
  /// @param name The name of the contact
  /// @return const int &
  inline const int & getNumFromName(const std::string & name)
  {
    if(hasSensor_.at(name))
    {
      return mapContactsWithSensors_.at(name).getID();
    }
    else
    {
      return mapContactsWithoutSensors_.at(name).getID();
    }
  }

  /// @brief Get the measured zmp of a contact given its name
  ///
  /// @param name The name of the contact
  /// @return const Eigen::Vector3d &
  inline const Eigen::Vector3d & getZMPFromName(const std::string & name)
  {
    if(hasSensor_.at(name))
    {
      return mapContactsWithSensors_.at(name).getZMP();
    }
    else
    {
      return mapContactsWithoutSensors_.at(name).getZMP();
    }
  }

  /// @brief Checks if the given contact exists
  ///
  /// @param element The name of the contact
  /// @return bool
  inline bool hasElement(const std::string & element)
  {
    return hasSensor_.find(element) != hasSensor_.end();
  }

  /// @brief Check that a contact still does not exist, if so, insert a contact to the map of contacts. The contact
  /// can either be associated to a sensor or not.
  ///
  /// @param element The name of the contact
  /// @param hasSensor True if the contact is attached to a sensor.
  inline void insertContact(const std::string & name, const bool & hasSensor)
  {
    if(checkAlreadyExists(name, hasSensor)) return;
    insertElement(name, hasSensor);

    num_++;
  }

private:
  /// @brief Insert a contact to the map of contacts. The contact can either be associated to a sensor or not.
  ///
  /// @param element The name of the contact
  /// @param hasSensor True if the contact is attached to a sensor.
  inline void insertElement(const std::string & name, const bool & hasSensor)
  {
    insertOrder_.push_back(name);

    if(hasSensor)
    {
      mapContactsWithSensors_.insert(std::make_pair(name, ContactWithSensor(num_, name)));
      hasSensor_.insert(std::make_pair(name, true));
    }
    else
    {
      mapContactsWithoutSensors_.insert(std::make_pair(name, ContactWithoutSensor(num_, name)));
      hasSensor_.insert(std::make_pair(name, true));
    }
  }

  /// @brief Check if a contact already exists in the list. If it already exists, checks that the association to a
  /// sensor remained unchanged.
  ///
  /// @param element The name of the contact
  /// @param hasSensor True if the contact is attached to a sensor.
  /// @return bool
  inline bool checkAlreadyExists(const std::string & name, const bool & hasContact)
  {
    if(hasSensor_.find(name) != hasSensor_.end())
    {
      if(hasContact)
      {
        BOOST_ASSERT((mapContactsWithoutSensors_.find(name) == mapContactsWithoutSensors_.end())
                     && "The contact already exists but was associated to no sensor");
        return true;
      }
      else
      {
        BOOST_ASSERT((mapContactsWithSensors_.find(name) == mapContactsWithSensors_.end())
                     && "The contact already exists but was associated to a sensor");
        return true;
      }
    }
    else
    {
      return false;
    }
  }

private:
  // map containing all the contacts and indicating if each sensor has a contact or not
  std::map<std::string, bool> hasSensor_;
  // map containing all the contacts associated to a sensor
  std::map<std::string, ContactWithSensor> mapContactsWithSensors_;
  // map containing all the contacts that are not associated to a sensor
  std::map<std::string, ContactWithoutSensor> mapContactsWithoutSensors_;
  // List of all the contacts
  std::vector<std::string> insertOrder_;
  // Index generator, incremented everytime a new contact is created
  int num_ = 0;
};

struct ContactsManager
{

public:
  ContactsManager() {}
  ~ContactsManager() {}
  // initialization for a detection based on contact surfaces
  void init(const mc_control::MCController & ctl,
            const std::string & robotName,
            const std::string & observerName,
            const std::string contactsDetection,
            std::vector<std::string> surfacesForContactDetection,
            std::vector<std::string> contactsSensorDisabledInit,
            const double contactDetectionThreshold);
  // initialization for a detection based on a threshold on the measured contact forces or for contacts given by the
  // controller
  void init(const mc_control::MCController & ctl,
            const std::string & robotName,
            const std::string & observerName,
            const std::string contactsDetection,
            std::vector<std::string> contactsSensorDisabledInit,
            const double contactDetectionThreshold);

  const std::set<std::string> & findContacts(const mc_control::MCController & ctl, const std::string & robotName);
  /// @brief Updates the list @contactsFound_ of currently set contacts directly from the controller.
  /// @details Called by \ref findContacts(const mc_control::MCController & ctl) if @contactsDetection_ is equal to
  /// "fromSolver". The contacts are given by the controller directly (then thresholded based on the measured force).
  /// @return std::set<std::string> &
  void findContactsFromSolver(const mc_control::MCController & ctl, const std::string & robotName);
  /// @brief Updates the list @contactsFound_ of currently set contacts from the surfaces given by the controller.
  /// @details Called by \ref findContacts(const mc_control::MCController & ctl) if @contactsDetection_ is equal to
  /// "fromSurfaces". The contacts are given by the controller directly from a surface (then thresholded based on the
  /// measured force).
  /// @return std::set<std::string> &
  void findContactsFromSurfaces(const mc_control::MCController & ctl, const std::string & robotName);
  /// @brief Updates the list @contactsFound_ of currently set contacts from a thresholding of the measured forces.
  /// @details Called by \ref findContacts(const mc_control::MCController & ctl) if @contactsDetection_ is equal to
  /// "fromThreshold". The contacts are not required to be given by the controller (the detection is based on a
  /// thresholding of the measured force).
  /// @return std::set<std::string> &
  void findContactsFromThreshold(const mc_control::MCController & ctl, const std::string & robotName);
  void (ContactsManager::*contactsFinder_)(const mc_control::MCController &, const std::string &) = 0;

public:
  MapContacts mapContacts_;
  // contacts found on each iteration
  std::set<std::string> contactsFound_;

protected:
  double contactDetectionThreshold_;

  // list of surfaces used for contacts detection if @contactsDetection_ is set to "fromSurfaces"
  std::vector<std::string> surfacesForContactDetection_;
  // list of sensors that must not be used from the start of the observer
  std::vector<std::string> contactsSensorDisabledInit_;
};

} // namespace measurements
} // namespace mc_state_observation