

namespace mc_state_observation
{

namespace measurements
{

///////////////////////////////////////////////////////////////////////
/// ------------------------------Contacts-----------------------------
///////////////////////////////////////////////////////////////////////

template<typename ContactWithSensorT, typename ContactWithoutSensorT, typename FoundContactsListType>
void ContactsManager<ContactWithSensorT, ContactWithoutSensorT, FoundContactsListType>::init(
    const mc_control::MCController & ctl,
    const std::string & robotName,
    const std::string & observerName,
    const std::string contactsDetection,
    std::vector<std::string> surfacesForContactDetection,
    std::vector<std::string> contactsSensorDisabledInit,
    const double contactDetectionThreshold)
{

  contactsFinder_ =
      &ContactsManager<ContactWithSensorT, ContactWithoutSensorT, FoundContactsListType>::findContactsFromSurfaces;

  contactDetectionThreshold_ = contactDetectionThreshold;
  surfacesForContactDetection_ = surfacesForContactDetection;
  contactsSensorDisabledInit_ = contactsSensorDisabledInit;
  observerName_ = observerName;

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

template<typename ContactWithSensorT, typename ContactWithoutSensorT, typename FoundContactsListType>
void ContactsManager<ContactWithSensorT, ContactWithoutSensorT, FoundContactsListType>::init(
    const mc_control::MCController & ctl,
    const std::string & robotName,
    const std::string & observerName,
    const std::string contactsDetection,
    std::vector<std::string> contactsSensorDisabledInit,
    const double contactDetectionThreshold)
{
  if(contactsDetection == "fromSolver")
  {
    contactsFinder_ =
        &ContactsManager<ContactWithSensorT, ContactWithoutSensorT, FoundContactsListType>::findContactsFromSolver;
  }
  if(contactsDetection == "fromThreshold")
  {
    contactsFinder_ =
        &ContactsManager<ContactWithSensorT, ContactWithoutSensorT, FoundContactsListType>::findContactsFromThreshold;
  }

  contactDetectionThreshold_ = contactDetectionThreshold;
  contactsSensorDisabledInit_ = contactsSensorDisabledInit;
  observerName_ = observerName;

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

template<typename ContactWithSensorT, typename ContactWithoutSensorT, typename FoundContactsListType>
const std::set<FoundContactsListType,
               typename ContactsManager<ContactWithSensorT, ContactWithoutSensorT, FoundContactsListType>::cmp> &
    ContactsManager<ContactWithSensorT, ContactWithoutSensorT, FoundContactsListType>::findContacts(
        const mc_control::MCController & ctl,
        const std::string & robotName)
{
  // Detection of the contacts depending on the configured mode
  (this->*contactsFinder_)(ctl, robotName);
  updateContacts();
  return contactsFound_; // list of currently set contacts
}

template<typename ContactWithSensorT, typename ContactWithoutSensorT, typename FoundContactsListType>
void ContactsManager<ContactWithSensorT, ContactWithoutSensorT, FoundContactsListType>::findContactsFromSolver(
    const mc_control::MCController & ctl,
    const std::string & robotName)
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
        ContactWithSensor & contactWS = mapContacts_.contactWithSensor(ifs.name());
        contactWS.forceNorm_ = ifs.wrenchWithoutGravity(measRobot).force().norm();
        if(contactWS.forceNorm_ > contactDetectionThreshold_)
        {
          // the contact is added to the map of contacts using the name of the associated sensor
          contactsFound_.insert(ifs.name());
        }
      }
    }
    else if(ctl.robots().robot(contact.r2Index()).name() == measRobot.name())
    {
      if(ctl.robots().robot(contact.r1Index()).mb().joint(0).type() == rbd::Joint::Fixed)
      {
        const auto & ifs = measRobot.indirectSurfaceForceSensor(contact.r2Surface()->name());
        ContactWithSensor & contactWS = mapContacts_.contactWithSensor(ifs.name());
        contactWS.forceNorm_ = ifs.wrenchWithoutGravity(measRobot).force().norm();
        if(contactWS.forceNorm_ > contactDetectionThreshold_)
        {
          // the contact is added to the map of contacts using the name of the associated surface
          contactsFound_.insert(ifs.name());
        }
      }
    }
  }
}

template<typename ContactWithSensorT, typename ContactWithoutSensorT, typename FoundContactsListType>
void ContactsManager<ContactWithSensorT, ContactWithoutSensorT, FoundContactsListType>::findContactsFromSurfaces(
    const mc_control::MCController & ctl,
    const std::string & robotName)
{
  const auto & measRobot = ctl.robot(robotName);

  contactsFound_.clear();

  for(auto & contact : mapContacts_.contactsWithSensors())
  {
    const std::string & fsName = contact.first;
    const mc_rbdyn::ForceSensor forceSensor = measRobot.forceSensor(fsName);

    contact.second.forceNorm_ = forceSensor.wrenchWithoutGravity(measRobot).force().norm();
    if(contact.second.forceNorm_ > contactDetectionThreshold_)
    {
      // the contact is added to the map of contacts using the name of the associated surface
      contactsFound_.insert(fsName);
    }
  }
}

template<typename ContactWithSensorT, typename ContactWithoutSensorT, typename FoundContactsListType>
void ContactsManager<ContactWithSensorT, ContactWithoutSensorT, FoundContactsListType>::findContactsFromThreshold(
    const mc_control::MCController & ctl,
    const std::string & robotName)
{
  const auto & measRobot = ctl.robot(robotName);

  contactsFound_.clear();

  for(auto & contact : mapContacts_.contactsWithSensors())
  {
    const std::string & fsName = contact.first;
    const mc_rbdyn::ForceSensor forceSensor = measRobot.forceSensor(fsName);
    contact.second.forceNorm_ = forceSensor.wrenchWithoutGravity(measRobot).force().norm();
    if(contact.second.forceNorm_ > contactDetectionThreshold_)
    {
      // the contact is added to the map of contacts using the name of the associated sensor
      contactsFound_.insert(fsName);
    }
  }
}

template<typename ContactWithSensorT, typename ContactWithoutSensorT, typename FoundContactsListType>
void ContactsManager<ContactWithSensorT, ContactWithoutSensorT, FoundContactsListType>::updateContacts()
{
  // List of the contact that were set on last iteration but are not set anymore on the current one
  std::set<std::string> diffs;
  std::set_difference(oldContacts_.begin(), oldContacts_.end(), contactsFound_.begin(), contactsFound_.end(),
                      std::inserter(diffs, diffs.end()));
  for(const auto & diff : diffs)
  {
    int numDiff = mapContacts_.getNumFromName(diff);
    contactWithSensor(diff).resetContact();
  }
  // update the list of previously set contacts
  if(diffs.size() != 0)
  {
    oldContacts_ = contactsFound_;
  }
}

} // namespace measurements
} // namespace mc_state_observation