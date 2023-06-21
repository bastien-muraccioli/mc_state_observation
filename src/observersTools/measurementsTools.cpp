#include <mc_observers/Observer.h>
#include <mc_rbdyn/Robot.h>
#include <mc_state_observation/observersTools/measurementsTools.h>
#include <state-observation/dynamics-estimators/kinetics-observer.hpp>

namespace mc_state_observation
{

namespace measurements
{

///////////////////////////////////////////////////////////////////////
/// ------------------------------Contacts-----------------------------
///////////////////////////////////////////////////////////////////////

void ContactsManager::init(const mc_control::MCController & ctl,
                           const std::string & robotName,
                           const std::string & observerName,
                           const std::string contactsDetection,
                           std::vector<std::string> surfacesForContactDetection,
                           std::vector<std::string> contactsSensorDisabledInit,
                           const double contactDetectionThreshold)
{

  contactsFinder_ = &mc_state_observation::measurements::ContactsManager::findContactsFromSurfaces;

  contactDetectionThreshold_ = contactDetectionThreshold;
  surfacesForContactDetection_ = surfacesForContactDetection;
  contactsSensorDisabledInit_ = contactsSensorDisabledInit;

  const auto & robot = ctl.robot(robotName);

  if(contactsDetection != "fromSolver" && contactsDetection != "fromThreshold" && contactsDetection != "fromSurfaces")
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Contacts detection type not allowed. Please pick among : [fromSolver, fromThreshold, fromSurfaces] or "
        "initialize a list of surfaces with the variable surfacesForContactDetection");
  }

  if(surfacesForContactDetection.size() > 0)
  {
    if(contactsDetection != "fromSurfaces")
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "The list of potential contact surfaces was given but the detection using surfaces is not selected");
    }
  }
  else if(contactsDetection == "fromSurfaces")
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "You selected the contacts detection using surfaces but didn't add the list of surfaces, please add it using "
        "the variable surfacesForContactDetection");
  }

  if(contactsDetection == "fromSurfaces")
  {
    for(const std::string & surface : surfacesForContactDetection)
    {
      // if the surface is associated to a force sensor (for example LeftFootCenter or RightFootCenter)
      if(robot.surfaceHasForceSensor(surface))
      {
        const mc_rbdyn::ForceSensor & forceSensor = robot.surfaceForceSensor(surface);
        const std::string & fsName = forceSensor.name();
        mapContacts_.insertContact(fsName, true);
        mapContacts_.contactWithSensor(fsName).surface = surface;

        ctl.gui()->addElement(
            {observerName, "Contacts"},
            mc_rtc::gui::Checkbox(
                fsName + " : "
                    + (mapContacts_.contactWithSensor(fsName).isSet ? "Contact is set" : "Contact is not set")
                    + ": Use wrench sensor: ",
                [this, fsName]() { return mapContacts_.contactWithSensor(fsName).sensorEnabled; },
                [this, fsName]()
                {
                  mapContacts_.contactWithSensor(fsName).sensorEnabled =
                      !mapContacts_.contactWithSensor(fsName).sensorEnabled;
                  std::cout << std::endl
                            << "Enable / disable :" + fsName + " "
                                   + std::to_string(mapContacts_.contactWithSensor(fsName).sensorEnabled)
                            << std::endl;
                }));
      }
      else // if the surface is not associated to a force sensor, we will fetch the force sensor indirectly attached to
           // the surface
      {
        const mc_rbdyn::ForceSensor & forceSensor = robot.indirectSurfaceForceSensor(surface);
        const std::string & fsName = forceSensor.name();
        mapContacts_.insertContact(forceSensor.name(), true);
        mapContacts_.contactWithSensor(fsName).sensorAttachedToSurface = false;
        mapContacts_.contactWithSensor(fsName).surface = surface;

        ctl.gui()->addElement(
            {observerName, "Contacts"},
            mc_rtc::gui::Checkbox(
                fsName + " : "
                    + (mapContacts_.contactWithSensor(fsName).isSet ? "Contact is set" : "Contact is not set")
                    + ": Use wrench sensor: ",
                [this, fsName]() { return mapContacts_.contactWithSensor(fsName).sensorEnabled; },
                [this, fsName]()
                {
                  mapContacts_.contactWithSensor(fsName).sensorEnabled =
                      !mapContacts_.contactWithSensor(fsName).sensorEnabled;
                  std::cout << std::endl
                            << "Enable / disable :" + fsName + " "
                                   + std::to_string(mapContacts_.contactWithSensor(fsName).sensorEnabled)
                            << std::endl;
                }));
      }
    }
  }

  for(auto const & contactSensorDisabledInit : contactsSensorDisabledInit)
  {
    BOOST_ASSERT(mapContacts_.hasElement(contactSensorDisabledInit) && "This sensor is not attached to the robot");
    mapContacts_.contactWithSensor(contactSensorDisabledInit).sensorEnabled = false;
  }
}

void ContactsManager::init(const mc_control::MCController & ctl,
                           const std::string & robotName,
                           const std::string & observerName,
                           const std::string contactsDetection,
                           std::vector<std::string> contactsSensorDisabledInit,
                           const double contactDetectionThreshold)
{
  if(contactsDetection == "fromSolver")
  {
    contactsFinder_ = &mc_state_observation::measurements::ContactsManager::findContactsFromSolver;
  }
  if(contactsDetection == "fromThreshold")
  {
    contactsFinder_ = &mc_state_observation::measurements::ContactsManager::findContactsFromThreshold;
  }

  contactDetectionThreshold_ = contactDetectionThreshold;
  contactsSensorDisabledInit_ = contactsSensorDisabledInit;

  const auto & robot = ctl.robot(robotName);

  if(contactsDetection != "fromSolver" && contactsDetection != "fromThreshold" && contactsDetection != "fromSurfaces")
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Contacts detection type not allowed. Please pick among : [fromSolver, fromThreshold, fromSurfaces] or "
        "initialize a list of surfaces with the variable surfacesForContactDetection");
  }

  if(contactsDetection == "fromSurfaces")
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "You selected the contacts detection using surfaces but didn't add the list of surfaces, please use the "
        "ContactsManager constructor that receives this surfaces list");
  }

  if(contactsDetection == "fromThreshold")
  {
    for(auto forceSensor : robot.forceSensors())
    {
      const std::string & fsName = forceSensor.name();

      mapContacts_.insertContact(forceSensor.name(), true);
      ctl.gui()->addElement(
          {observerName, "Contacts"},
          mc_rtc::gui::Checkbox(
              fsName + " : " + (mapContacts_.contactWithSensor(fsName).isSet ? "Contact is set" : "Contact is not set")
                  + ": Use wrench sensor: ",
              [this, fsName]() { return mapContacts_.contactWithSensor(fsName).sensorEnabled; },
              [this, fsName]() {
                mapContacts_.contactWithSensor(fsName).sensorEnabled =
                    !mapContacts_.contactWithSensor(fsName).sensorEnabled;
              }));
    }
  }

  for(auto const & contactSensorDisabledInit : contactsSensorDisabledInit)
  {
    BOOST_ASSERT(mapContacts_.hasElement(contactSensorDisabledInit) && "This sensor is not attached to the robot");
    mapContacts_.contactWithSensor(contactSensorDisabledInit).sensorEnabled = false;
  }
}

const std::set<std::string> & ContactsManager::findContacts(const mc_control::MCController & ctl,
                                                            const std::string & robotName)
{
  // Detection of the contacts depending on the configured mode
  (this->*contactsFinder_)(ctl, robotName);

  return contactsFound_; // list of currently set contacts
}

void ContactsManager::findContactsFromSolver(const mc_control::MCController & ctl, const std::string & robotName)
{
  const auto & measRobot = ctl.robot(robotName);

  contactsFound_.clear();
  for(const auto & contact : ctl.solver().contacts())
  {
    if(ctl.robots().robot(contact.r1Index()).name() == measRobot.name())
    {
      if(ctl.robots().robot(contact.r2Index()).mb().joint(0).type() == rbd::Joint::Fixed)
      {
        const auto & ifs = measRobot.indirectSurfaceForceSensor(contact.r1Surface()->name());
        mapContacts_.contactWithSensor(ifs.name()).isExternalWrench = true;
        if(!mapContacts_.contactWithSensor(ifs.name()).sensorEnabled)
        {
          // if the sensor attached to the contact is not enabled, it must not be considered as an external perturbation
          // in any case
          mapContacts_.contactWithSensor(ifs.name()).isExternalWrench = false;
        }
        if(ifs.wrenchWithoutGravity(measRobot).force().norm() > contactDetectionThreshold_)
        {
          // the contact is added to the map of contacts using the name of the associated surface
          contactsFound_.insert(ifs.name());
          // if the measured force is higher than the detection threshold, the contact is detected and not counted and
          // the measurement is not considered as an external input inside \ref inputAdditionalWrench(const
          // mc_rbdyn::Robot & inputRobot, const mc_rbdyn::Robot & measRobot).
          mapContacts_.contactWithSensor(ifs.name()).isExternalWrench = false;
        }
      }
    }
    else if(ctl.robots().robot(contact.r2Index()).name() == measRobot.name())
    {
      if(ctl.robots().robot(contact.r1Index()).mb().joint(0).type() == rbd::Joint::Fixed)
      {
        const auto & ifs = measRobot.indirectSurfaceForceSensor(contact.r2Surface()->name());
        if(!mapContacts_.contactWithSensor(ifs.name()).sensorEnabled)
        {
          // if the sensor attached to the contact is not enabled, it must not be considered as an external perturbation
          // in any case
          mapContacts_.contactWithSensor(ifs.name()).isExternalWrench = false;
        }
        if(ifs.wrenchWithoutGravity(measRobot).force().norm() > contactDetectionThreshold_)
        {
          // the contact is added to the map of contacts using the name of the associated surface
          contactsFound_.insert(ifs.name());
          // if the measured force is higher than the detection threshold, the contact is detected and not counted and
          // the measurement is not considered as an external input inside \ref inputAdditionalWrench(const
          // mc_rbdyn::Robot & inputRobot, const mc_rbdyn::Robot & measRobot).
          mapContacts_.contactWithSensor(ifs.name()).isExternalWrench = false;
        }
      }
    }
  }
}

void ContactsManager::findContactsFromSurfaces(const mc_control::MCController & ctl, const std::string & robotName)
{
  const auto & measRobot = ctl.robot(robotName);

  contactsFound_.clear();

  for(auto & contact : mapContacts_.contactsWithSensors())
  {
    const std::string & fsName = contact.first;
    mapContacts_.contactWithSensor(fsName).isExternalWrench = true;
    const mc_rbdyn::ForceSensor forceSensor = measRobot.forceSensor(fsName);
    if(!mapContacts_.contactWithSensor(fsName).sensorEnabled)
    {
      // if the sensor attached to the contact is not enabled, it must not be considered as an external perturbation in
      // any case
      mapContacts_.contactWithSensor(fsName).isExternalWrench = false;
    }
    if(forceSensor.wrenchWithoutGravity(measRobot).force().norm() > contactDetectionThreshold_)
    {
      // the contact is added to the map of contacts using the name of the associated surface
      contactsFound_.insert(fsName);
      // if the measured force is higher than the detection threshold, the contact is detected and not counted and the
      // measurement is not considered as an external input inside \ref inputAdditionalWrench(const mc_rbdyn::Robot &
      // inputRobot, const mc_rbdyn::Robot & measRobot).
      mapContacts_.contactWithSensor(fsName).isExternalWrench = false;
    }
  }
}

void ContactsManager::findContactsFromThreshold(const mc_control::MCController & ctl, const std::string & robotName)
{
  const auto & measRobot = ctl.robot(robotName);

  contactsFound_.clear();

  for(auto & contact : mapContacts_.contactsWithSensors())
  {
    const std::string & fsName = contact.first;
    mapContacts_.contactWithSensor(fsName).isExternalWrench = true;
    const mc_rbdyn::ForceSensor forceSensor = measRobot.forceSensor(fsName);
    if(!mapContacts_.contactWithSensor(fsName).sensorEnabled)
    {
      // if the sensor attached to the contact is not enabled, it must not be considered as an external perturbation
      // in any case
      mapContacts_.contactWithSensor(fsName).isExternalWrench = false;
    }
    if(forceSensor.wrenchWithoutGravity(measRobot).force().norm() > contactDetectionThreshold_)
    {
      // the contact is added to the map of contacts using the name of the associated sensor
      contactsFound_.insert(fsName);
      // if the measured force is higher than the detection threshold, the contact is detected and not counted and
      // the measurement is not considered as an external input inside \ref inputAdditionalWrench(const
      // mc_rbdyn::Robot & inputRobot, const mc_rbdyn::Robot & measRobot).
      mapContacts_.contactWithSensor(fsName).isExternalWrench = false;
    }
  }
}
} // namespace measurements

} // namespace mc_state_observation
