/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#include <mc_control/MCController.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/io_utils.h>
#include <mc_rtc/logging.h>
#include <mc_state_observation/NaiveOdometry.h>
#include <mc_state_observation/gui_helpers.h>

#include <RBDyn/CoM.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <iostream>

namespace mc_state_observation
{
NaiveOdometry::NaiveOdometry(const std::string & type, double dt) : mc_observers::Observer(type, dt) {}

///////////////////////////////////////////////////////////////////////
/// --------------------------Core functions---------------------------
///////////////////////////////////////////////////////////////////////

void NaiveOdometry::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  config("debug", debug_);
  config("verbose", verbose_);

  config("withContactsDetection", withContactsDetection_);
  config("withNaiveYawEstimation", withNaiveYawEstimation_);
  config("contactDetectionPropThreshold", contactDetectionPropThreshold_);
  config("withFilteredForcesContactDetection", withFilteredForcesContactDetection_);

  std::string odometryType = static_cast<std::string>(config("odometryType"));
  if(odometryType == "flatOdometry")
  {
    withFlatOdometry_ = true;
  }
  else if(odometryType != "6dOdometry")
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Odometry type not allowed. Please pick among : [flatOdometry, 6dOdometry]");
  }

  contactsDetection_ = static_cast<std::string>(config("contactsDetection"));
  if(contactsDetection_ != "fromSolver" && contactsDetection_ != "fromThreshold"
     && contactsDetection_ != "fromSurfaces")
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Contacts detection type not allowed. Please pick among : [fromSolver, fromThreshold, fromSurfaces] or "
        "initialize a list of surfaces with the variable surfacesForContactDetection");
  }
  config("surfacesForContactDetection", surfacesForContactDetection_);
  if(surfacesForContactDetection_.size() > 0)
  {
    if(contactsDetection_ != "fromSurfaces")
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "Another type of contacts detection is currently used, please change it to 'fromSurfaces' or empty the "
          "surfacesForContactDetection variable");
    }
  }
  else if(contactsDetection_ == "fromSurfaces")
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "You selected the contacts detection using surfaces but didn't add the list of surfaces, please add it usign "
        "the variable surfacesForContactDetection");
  }

  zeroPose_.translation().setZero();
  zeroPose_.rotation().setIdentity();
  zeroMotion_.linear().setZero();
  zeroMotion_.angular().setZero();
}

void NaiveOdometry::reset(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);
  const auto & realRobotModule = realRobot.module();

  rbd::MultiBodyGraph mergeMbg(realRobotModule.mbg);
  std::map<std::string, std::vector<double>> jointPosByName;
  for(int i = 0; i < realRobotModule.mb.nrJoints(); ++i)
  {
    auto jointName = realRobotModule.mb.joint(i).name();
    auto jointIndex = static_cast<unsigned long>(realRobotModule.mb.jointIndexByName(jointName));
    jointPosByName[jointName] = realRobotModule.mbc.q[jointIndex];
  }

  std::vector<std::string> rootJoints = {};
  int nbJoints = static_cast<int>(realRobot.mb().joints().size());
  for(int i = 0; i < nbJoints; ++i)
  {
    if(realRobot.mb().predecessor(i) == 0)
    {
      rootJoints.push_back(realRobot.mb().joint(i).name());
    }
  }
  for(const auto & joint : rootJoints)
  {
    if(!realRobot.hasJoint(joint))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Robot does not have a joint named {}", joint);
    }
    mergeMbg.mergeSubBodies(realRobotModule.mb.body(0).name(), joint, jointPosByName);
  }

  for(auto forceSensor : realRobot.forceSensors())
  {
    if(contactsDetection_ == "fromThreshold")
    {
      for(auto forceSensor : realRobot.forceSensors())
      {
        const std::string & fsName = forceSensor.name();

        mapContacts_.insertContact(forceSensor.name());
      }
    }
    if(contactsDetection_ == "fromSurfaces")
    {
      for(const std::string & surface : surfacesForContactDetection_)
      {
        if(robot.surfaceHasForceSensor(surface))
        {
          const mc_rbdyn::ForceSensor & forceSensor = robot.surfaceForceSensor(surface);
          const std::string & fsName = forceSensor.name();
          mapContacts_.insertContact(forceSensor.name());
          mapContacts_.contactWithSensor(fsName).surface = surface;
        }
        else
        {
          const mc_rbdyn::ForceSensor & forceSensor = robot.indirectSurfaceForceSensor(surface);
          const std::string & fsName = forceSensor.name();

          mapContacts_.insertContact(forceSensor.name());
          mapContacts_.contactWithSensor(fsName).isAttachedToSurface = false;
          mapContacts_.contactWithSensor(fsName).surface = surface;
        }
      }
    }
  }

  mass(ctl.realRobot(robot_).mass());

  my_robots_ = mc_rbdyn::Robots::make();
  my_robots_->robotCopy(robot, robot.name());
  my_robots_->robotCopy(realRobot, "odometryRobot");
  ctl.gui()->addElement(
      {"Robots"},
      mc_rtc::gui::Robot("NaiveOdometry", [this]() -> const mc_rbdyn::Robot & { return my_robots_->robot(); }));

  X_0_fb_.translation() = realRobot.posW().translation();
  X_0_fb_.rotation() = realRobot.posW().rotation();
}

bool NaiveOdometry::run(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);
  auto & odometryRobot = my_robots_->robot("odometryRobot");
  auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();

  odometryRobot.mbc() = realRobot.mbc();
  odometryRobot.mb() = realRobot.mb();

  odometryRobot.velW(realRobot.velW());
  odometryRobot.accW(realRobot.accW());

  findContacts(ctl); // retrieves the list of contacts and set simStarted to true once a contact is detected

  /** Contacts
   * Note that when we use force sensors, this should be the position of the force sensor!
   */
  updateContacts(ctl, odometryRobot, contactsFound_, logger);

  // X_0_fb_.rotation() = odometryRobot.posW().rotation();
  //  X_0_fb_.translation() = mcko_K_0_fb.position();

  v_fb_0_.linear() =
      odometryRobot.velW().linear(); //  X_0_fb_.rotation() = mcko_K_0_fb.orientation.toMatrix3().transpose()
  v_fb_0_.angular() = odometryRobot.velW().angular();

  a_fb_0_.linear() =
      odometryRobot.accW().linear(); //  X_0_fb_.rotation() = mcko_K_0_fb.orientation.toMatrix3().transpose()
  a_fb_0_.angular() = odometryRobot.accW().angular();

  /* Update of the visual representation (only a visual feature) of the observed robot */
  my_robots_->robot().mbc().q = ctl.realRobot().mbc().q;

  update(my_robots_->robot());
  return true;
}

///////////////////////////////////////////////////////////////////////
/// -------------------------Called functions--------------------------
///////////////////////////////////////////////////////////////////////

void NaiveOdometry::update(mc_control::MCController & ctl) // this function is called by the pipeline if the
                                                           // update is set to true in the configuration file
{
  auto & realRobot = ctl.realRobot(robot_);
  update(realRobot);
}

void NaiveOdometry::update(mc_rbdyn::Robot & robot)
{
  robot.posW(X_0_fb_);
  robot.velW(v_fb_0_.vector());
}

const std::set<std::string> & NaiveOdometry::findContactsFromSolver(const mc_control::MCController & ctl)
{
  const auto & measRobot = ctl.robot(robot_);
  auto & odometryRobot = my_robots_->robot("odometryRobot");

  contactsFound_.clear();
  for(const auto & contact : ctl.solver().contacts())
  {
    // std::cout << std::endl << contact.toStr() << std::endl;
    if(ctl.robots().robot(contact.r1Index()).name() == measRobot.name())
    {
      if(ctl.robots().robot(contact.r2Index()).mb().joint(0).type() == rbd::Joint::Fixed)
      {
        const auto & ifs = measRobot.indirectSurfaceForceSensor(contact.r1Surface()->name());
        if(ifs.wrenchWithoutGravity(measRobot).force().norm()
           > measRobot.mass() * so::cst::gravityConstant * contactDetectionPropThreshold_)
        {
          contactsFound_.insert(ifs.name());
        }
      }
    }
    else if(ctl.robots().robot(contact.r2Index()).name() == measRobot.name())
    {
      if(ctl.robots().robot(contact.r1Index()).mb().joint(0).type() == rbd::Joint::Fixed)
      {
        const auto & ifs = measRobot.indirectSurfaceForceSensor(contact.r2Surface()->name());

        if(ifs.wrenchWithoutGravity(measRobot).force().norm()
           > measRobot.mass() * so::cst::gravityConstant * contactDetectionPropThreshold_)
        {
          contactsFound_.insert(ifs.name());
        }
      }
    }
  }
}

const std::set<std::string> & NaiveOdometry::findContactsFromSurfaces(const mc_control::MCController & ctl)
{
  const auto & measRobot = ctl.robot(robot_);
  auto & odometryRobot = my_robots_->robot("odometryRobot");

  contactsFound_.clear();

  for(auto & contact : mapContacts_.contactsWithSensors())
  {
    const std::string & fsName = contact.first;

    const mc_rbdyn::ForceSensor forceSensor = measRobot.forceSensor(fsName);

    if(withFilteredForcesContactDetection_) // NOT WORKING FOR NOW
    {
      ContactWithSensor & contact = mapContacts_.contactWithSensor(fsName);
      contact.filteredForce = (1 - ctl.timeStep * contact.lambda) * contact.filteredForce
                              + contact.lambda * ctl.timeStep * forceSensor.wrenchWithoutGravity(measRobot).force();

      if(contact.filteredForce.norm() > measRobot.mass() * so::cst::gravityConstant * contactDetectionPropThreshold_)
      {
        contactsFound_.insert(fsName);
      }
    }
    else
    {
      if(forceSensor.wrenchWithoutGravity(measRobot).force().norm()
         > measRobot.mass() * so::cst::gravityConstant * contactDetectionPropThreshold_)
      {
        contactsFound_.insert(fsName);
      }
    }
  }
}

const std::set<std::string> & NaiveOdometry::findContactsFromThreshold(const mc_control::MCController & ctl)
{
  const auto & measRobot = ctl.robot(robot_);
  auto & odometryRobot = my_robots_->robot("odometryRobot");

  contactsFound_.clear();

  for(auto & contact : mapContacts_.contactsWithSensors())
  {
    const std::string & fsName = contact.first;
    const mc_rbdyn::ForceSensor forceSensor = measRobot.forceSensor(fsName);
    if(withFilteredForcesContactDetection_) // NOT WORKING FOR NOW
    {
      ContactWithSensor & contact = mapContacts_.contactWithSensor(fsName);
      contact.filteredForce = (1 - ctl.timeStep * contact.lambda) * contact.filteredForce
                              + contact.lambda * ctl.timeStep * forceSensor.wrenchWithoutGravity(measRobot).force();

      if(contact.filteredForce.norm() > measRobot.mass() * so::cst::gravityConstant * contactDetectionPropThreshold_)
      {
        contactsFound_.insert(fsName);
      }
    }
    else
    {
      if(forceSensor.wrenchWithoutGravity(measRobot).force().norm()
         > measRobot.mass() * so::cst::gravityConstant * contactDetectionPropThreshold_)
      {
        contactsFound_.insert(fsName);
      }
    }
  }
}

const std::set<std::string> & NaiveOdometry::findContacts(const mc_control::MCController & ctl)
{
  const auto & measRobot = ctl.robot(robot_);
  auto & odometryRobot = my_robots_->robot("odometryRobot");

  if(contactsDetection_ == "fromSolver")
  {
    findContactsFromSolver(ctl);
  }
  if(contactsDetection_ == "fromThreshold")
  {
    findContactsFromThreshold(ctl);
  }
  if(contactsDetection_ == "fromSurfaces")
  {
    findContactsFromSurfaces(ctl);
  }

  return contactsFound_;
}

void NaiveOdometry::setNewContact(const mc_rbdyn::Robot & odometryRobot, const mc_rbdyn::ForceSensor forceSensor)
{

  if(mapContacts_.contactWithSensor(forceSensor.name()).isAttachedToSurface)
  {
    so::kine::Kinematics worldNewContactKineOdometryRobot;
    so::kine::Kinematics worldContactKineRef;
    worldContactKineRef.setZero(so::kine::Kinematics::Flags::position);

    // getting the position in the world of the new contact
    const sva::PTransformd & bodyNewContactPoseRobot = forceSensor.X_p_f();
    so::kine::Kinematics bodyNewContactKine;
    bodyNewContactKine.setZero(so::kine::Kinematics::Flags::all);
    bodyNewContactKine.position = bodyNewContactPoseRobot.translation();
    bodyNewContactKine.orientation = so::Matrix3(bodyNewContactPoseRobot.rotation().transpose());

    so::kine::Kinematics worldBodyKineOdometryRobot;

    worldBodyKineOdometryRobot.position =
        odometryRobot.mbc().bodyPosW[odometryRobot.bodyIndexByName(forceSensor.parentBody())].translation();
    worldBodyKineOdometryRobot.orientation = so::Matrix3(
        odometryRobot.mbc().bodyPosW[odometryRobot.bodyIndexByName(forceSensor.parentBody())].rotation().transpose());
    worldBodyKineOdometryRobot.linVel =
        odometryRobot.mbc().bodyVelW[odometryRobot.bodyIndexByName(forceSensor.parentBody())].linear();
    worldBodyKineOdometryRobot.angVel =
        odometryRobot.mbc().bodyVelW[odometryRobot.bodyIndexByName(forceSensor.parentBody())].angular();
    worldBodyKineOdometryRobot.linAcc =
        worldBodyKineOdometryRobot.orientation.toMatrix3()
        * odometryRobot.mbc().bodyAccB[odometryRobot.bodyIndexByName(forceSensor.parentBody())].linear();
    worldBodyKineOdometryRobot.angAcc =
        worldBodyKineOdometryRobot.orientation.toMatrix3()
        * odometryRobot.mbc().bodyAccB[odometryRobot.bodyIndexByName(forceSensor.parentBody())].angular();

    worldNewContactKineOdometryRobot = worldBodyKineOdometryRobot * bodyNewContactKine;

    mapContacts_.contactWithSensor(forceSensor.name()).worldRefKine_.position =
        worldNewContactKineOdometryRobot.position();
    mapContacts_.contactWithSensor(forceSensor.name()).worldRefKine_.orientation =
        worldNewContactKineOdometryRobot.orientation;
  }
  else
  {
    sva::PTransformd worldSurfacePoseOdometryRobot =
        odometryRobot.surfacePose(mapContacts_.contactWithSensor(forceSensor.name()).surface);

    mapContacts_.contactWithSensor(forceSensor.name()).worldRefKine_.position =
        worldSurfacePoseOdometryRobot.translation();
    mapContacts_.contactWithSensor(forceSensor.name()).worldRefKine_.orientation =
        so::Matrix3(worldSurfacePoseOdometryRobot.rotation().transpose());
  }

  if(withFlatOdometry_)
  {
    mapContacts_.contactWithSensor(forceSensor.name()).worldRefKine_.position()(2) = 0.0;
  }
}

void NaiveOdometry::updateContacts(const mc_control::MCController & ctl,
                                   mc_rbdyn::Robot & odometryRobot,
                                   std::set<std::string> updatedContacts,
                                   mc_rtc::Logger & logger)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);
  odometryRobot.posW(X_0_fb_);
  /** Debugging output **/
  if(verbose_ && updatedContacts != oldContacts_)
    mc_rtc::log::info("[{}] Contacts changed: {}", name(), mc_rtc::io::to_string(updatedContacts));

  std::map<std::string, so::kine::Kinematics>
      alreadySetContactsPositionOdometry; // the Kinematics part is the one of the floating base obtained from the
                                          // contact
  std::map<std::string, so::kine::Kinematics>
      alreadySetContactsOrientationOdometry; // the Kinematics part is the one of the floating base obtained from the
                                             // contact
  double sumAlreadySetForcesNormPositionOdometry =
      0.0; // two different lists for position and orientation as the orientation list can contain only two contacts
  double sumAlreadySetForcesNormOrientationOdometry = 0.0;
  so::Vector3 totalFbPosition = so::Vector3::Zero();
  for(const std::string & updatedContact : updatedContacts)
  {
    if(oldContacts_.find(updatedContact)
       != oldContacts_.end()) // the contact already exists so we will use it to estimate the floating base pose
    {
      if(withNaiveYawEstimation_
         && updatedContact.find("Hand") == std::string::npos) // we don't use hands for the orientation odometry
      {
        if(alreadySetContactsOrientationOdometry.size()
           == 2) // this is to keep a list of at most two contacts in the case we use the
                 // naive yaw estimation as the expressions used impose this constraint
        {
          std::string minForceContact = alreadySetContactsOrientationOdometry.begin()->first;
          double minForce = robot.forceSensor(alreadySetContactsOrientationOdometry.begin()->first)
                                .wrenchWithoutGravity(odometryRobot)
                                .force()
                                .norm();
          for(auto & contact : alreadySetContactsOrientationOdometry)
          {
            if(robot.forceSensor(contact.first).wrenchWithoutGravity(odometryRobot).force().norm() < minForce)
            {
              minForce = robot.forceSensor(contact.first).wrenchWithoutGravity(odometryRobot).force().norm();
              minForceContact = contact.first;
            }
          }
          if(robot.forceSensor(updatedContact).wrenchWithoutGravity(odometryRobot).force().norm() > minForce)
          {
            alreadySetContactsOrientationOdometry.erase(minForceContact);
            so::kine::Kinematics kine;
            alreadySetContactsOrientationOdometry.insert(std::make_pair(updatedContact, kine));
          }
        }
        else
        {
          so::kine::Kinematics kine;
          alreadySetContactsOrientationOdometry.insert(std::make_pair(updatedContact, kine));
        }
      }
    }
  }

  for(const std::string & updatedContact : updatedContacts)
  {
    const mc_rbdyn::ForceSensor & fs = robot.forceSensor(updatedContact);
    ContactWithSensor & currentContact = mapContacts_.contactWithSensor(fs.name());
    sva::ForceVecd measuredWrench = fs.wrenchWithoutGravity(robot);
    if(oldContacts_.find(updatedContact)
       != oldContacts_.end()) // the contact already exists so we will use it to estimate the floating base pose
    {
      if(withNaiveYawEstimation_
         && updatedContact.find("Hand") == std::string::npos) // we don't use hands for the orientation odometry
      {
        if(alreadySetContactsOrientationOdometry.size()
           == 2) // this is to keep a list of at most two contacts in the case we use the
                 // naive yaw estimation as the expressions used impose this constraint
        {
          std::string minForceContact = alreadySetContactsOrientationOdometry.begin()->first;
          double minForce = robot.forceSensor(alreadySetContactsOrientationOdometry.begin()->first)
                                .wrenchWithoutGravity(odometryRobot)
                                .force()
                                .norm();
          for(auto & contact : alreadySetContactsOrientationOdometry)
          {
            if(robot.forceSensor(contact.first).wrenchWithoutGravity(odometryRobot).force().norm() < minForce)
            {
              minForce = robot.forceSensor(contact.first).wrenchWithoutGravity(odometryRobot).force().norm();
              minForceContact = contact.first;
            }
          }
          if(robot.forceSensor(updatedContact).wrenchWithoutGravity(odometryRobot).force().norm() > minForce)
          {
            alreadySetContactsOrientationOdometry.erase(minForceContact);
            so::kine::Kinematics kine;
            alreadySetContactsOrientationOdometry.insert(std::make_pair(updatedContact, kine));
          }
        }
        else
        {
          so::kine::Kinematics kine;
          alreadySetContactsOrientationOdometry.insert(std::make_pair(updatedContact, kine));
        }
      }

      so::kine::Kinematics kine;
      alreadySetContactsPositionOdometry.insert(std::make_pair(updatedContact, kine));
      const sva::PTransformd & bodyAlreadySetContactSensorPose = fs.X_p_f();
      so::kine::Kinematics bodyAlreadySetContactSensorKine;
      bodyAlreadySetContactSensorKine.setZero(so::kine::Kinematics::Flags::position
                                              | so::kine::Kinematics::Flags::orientation);
      bodyAlreadySetContactSensorKine.position = bodyAlreadySetContactSensorPose.translation();
      bodyAlreadySetContactSensorKine.orientation = so::Matrix3(bodyAlreadySetContactSensorPose.rotation().transpose());

      so::kine::Kinematics worldBodyKineOdometryRobot;

      worldBodyKineOdometryRobot.position =
          odometryRobot.mbc().bodyPosW[odometryRobot.bodyIndexByName(fs.parentBody())].translation();
      worldBodyKineOdometryRobot.orientation = so::Matrix3(
          odometryRobot.mbc().bodyPosW[odometryRobot.bodyIndexByName(fs.parentBody())].rotation().transpose());

      so::kine::Kinematics worldAlreadySetContactSensorKineOdometryRobot =
          worldBodyKineOdometryRobot * bodyAlreadySetContactSensorKine;

      so::kine::Kinematics worldAlreadySetContactKineOdometryRobot;
      if(currentContact.isAttachedToSurface)
      {
        worldAlreadySetContactKineOdometryRobot = worldAlreadySetContactSensorKineOdometryRobot;
      }
      else
      {
        sva::PTransformd worldSurfacePoseOdometryRobot = odometryRobot.surfacePose(currentContact.surface);
        so::kine::Kinematics worldSurfaceKineOdometryRobot;
        worldSurfaceKineOdometryRobot.setZero(so::kine::Kinematics::Flags::all);
        worldSurfaceKineOdometryRobot.position = worldSurfacePoseOdometryRobot.translation();
        worldSurfaceKineOdometryRobot.orientation = so::Matrix3(worldSurfacePoseOdometryRobot.rotation().transpose());
        worldAlreadySetContactKineOdometryRobot = worldSurfaceKineOdometryRobot;

        so::kine::Kinematics surfaceSensorKine;
        surfaceSensorKine = worldSurfaceKineOdometryRobot.getInverse() * worldAlreadySetContactSensorKineOdometryRobot;

        contactWrenchVector_.segment<3>(0) = surfaceSensorKine.orientation * measuredWrench.force();
        contactWrenchVector_.segment<3>(3) = surfaceSensorKine.orientation * measuredWrench.moment()
                                             + surfaceSensorKine.position().cross(contactWrenchVector_.segment<3>(0));
      }
      const so::Vector3 & worldAlreadySetContactPositionOdometryRobot =
          worldAlreadySetContactKineOdometryRobot.position();

      const so::Vector3 & worldFbPositionOdometryRobot = odometryRobot.posW().translation();

      so::Vector3 worldFbPositionOdometry =
          mapContacts_.contactWithSensor(updatedContact).worldRefKine_.position()
          + (worldFbPositionOdometryRobot - worldAlreadySetContactPositionOdometryRobot);
      alreadySetContactsPositionOdometry.at(updatedContact).position = worldFbPositionOdometry;
      alreadySetContactsPositionOdometry.at(updatedContact).orientation =
          so::Matrix3(mapContacts_.contactWithSensor(updatedContact).worldRefKine_.orientation.toMatrix3()
                      * worldAlreadySetContactKineOdometryRobot.orientation.toMatrix3().transpose()
                      * odometryRobot.posW().rotation().transpose());
      if(withNaiveYawEstimation_
         && alreadySetContactsOrientationOdometry.find(updatedContact) != alreadySetContactsOrientationOdometry.end())
      {
        alreadySetContactsOrientationOdometry.at(updatedContact).orientation =
            so::Matrix3(mapContacts_.contactWithSensor(updatedContact).worldRefKine_.orientation.toMatrix3()
                        * worldAlreadySetContactKineOdometryRobot.orientation.toMatrix3().transpose()
                        * odometryRobot.posW().rotation().transpose());
        alreadySetContactsOrientationOdometry.at(updatedContact).position = worldFbPositionOdometry;

        sumAlreadySetForcesNormOrientationOdometry +=
            fs.wrenchWithoutGravity(robot).force().norm(); // robot because odometryRobot has no sensors
      }

      // with the names convention used in state observation : worldFbOriOdometry = worldContactOriRef * contactFbOri
      // = worldContactOriRef * contactWorldOri * worldFbOri;

      totalFbPosition += worldFbPositionOdometry * fs.wrenchWithoutGravity(odometryRobot).force().norm();
      sumAlreadySetForcesNormPositionOdometry +=
          fs.wrenchWithoutGravity(robot).force().norm(); // robot because odometryRobot has no sensors
    }
  }

  if(alreadySetContactsPositionOdometry.size() > 0)
  {
    X_0_fb_.translation() = totalFbPosition / sumAlreadySetForcesNormPositionOdometry;

    if(withNaiveYawEstimation_)
    {
      if(alreadySetContactsOrientationOdometry.size() == 1)
      {
        const so::Matrix3 & realRobotOri =
            realRobot.posW().rotation().transpose(); // the odometryRobot's orientation is overwritten by the
                                                     // realRobot's one on every iteration

        X_0_fb_.rotation() = so::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(
                                 realRobotOri, alreadySetContactsOrientationOdometry.begin()->second.orientation)
                                 .transpose();
      }
      if(alreadySetContactsOrientationOdometry.size() == 2) // else no nothing
      {
        const auto & contact1 = alreadySetContactsOrientationOdometry.begin();
        const auto & contact2 = std::next(alreadySetContactsOrientationOdometry.begin(), 1);

        const auto & R1 = contact1->second.orientation.toMatrix3();
        const auto & R2 = contact2->second.orientation.toMatrix3();

        double u = robot.forceSensor(contact1->first).force().norm() / sumAlreadySetForcesNormOrientationOdometry;
        so::Matrix3 diffRot = R1.transpose() * R2;

        so::Vector3 diffRotVector = (1.0 - u)
                                    * so::kine::skewSymmetricToRotationVector(
                                        diffRot); // we perform the multiplication by the weighting coefficient now so a
                                                  // zero coefficient gives a unit rotation matrix and not a zero matrix
        so::AngleAxis diffRotAngleAxis = so::kine::rotationVectorToAngleAxis(diffRotVector);

        so::Matrix3 diffRotMatrix =
            so::kine::Orientation(diffRotAngleAxis).toMatrix3(); // exp( (1 - u) * log(R1^T R2) )

        so::Matrix3 meanOri = R1 * diffRotMatrix;

        const so::Matrix3 & realRobotOri =
            realRobot.posW().rotation().transpose(); // the odometryRobot's orientation is overwritten by the
                                                     // realRobot's one on every iteration
        X_0_fb_.rotation() = so::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(realRobotOri, meanOri).transpose();
      }
    }
  }
  odometryRobot.posW(X_0_fb_);

  for(const std::string & updatedContact : updatedContacts)
  {
    if(alreadySetContactsPositionOdometry.find(updatedContact) == alreadySetContactsPositionOdometry.end())
    {
      const mc_rbdyn::ForceSensor & fs = robot.forceSensor(updatedContact);
      mapContacts_.contactWithSensor(robot.forceSensor(updatedContact).name()).isSet = true;
      setNewContact(odometryRobot, fs);
      addContactLogEntries(logger, robot.forceSensor(updatedContact).name());
    }
  }

  std::set<std::string>
      diffs; // List of the contact that were available on last iteration but are not set anymore on the current one
  std::set_difference(oldContacts_.begin(), oldContacts_.end(), updatedContacts.begin(), updatedContacts.end(),
                      std::inserter(diffs, diffs.end()));
  for(const auto & diff : diffs)
  {
    mapContacts_.contactWithSensor(robot.forceSensor(diff).name()).isSet = false;
    removeContactLogEntries(logger, robot.forceSensor(diff).name());
  }

  oldContacts_ = updatedContacts;
  unsigned nbContacts = static_cast<unsigned>(oldContacts_.size());
  if(debug_)
  {
    mc_rtc::log::info("nbContacts = {}", nbContacts);
  }
}

void NaiveOdometry::addContactLogEntries(mc_rtc::Logger & logger, const std::string & contactName)
{
  logger.addLogEntry(category_ + contactName + "_position",
                     [this, contactName]() -> Eigen::Vector3d
                     { return mapContacts_.contactWithSensor(contactName).worldRefKine_.position(); });
  logger.addLogEntry(
      category_ + contactName + "_orientation",
      [this, contactName]() -> so::Quaternion
      { return mapContacts_.contactWithSensor(contactName).worldRefKine_.orientation.toQuaternion().inverse(); });
}

void NaiveOdometry::removeContactLogEntries(mc_rtc::Logger & logger, const std::string & contactName)
{
  logger.removeLogEntry(category_ + contactName + "_position");
  logger.removeLogEntry(category_ + contactName + "_orientation");
}

void NaiveOdometry::mass(double mass)
{
  mass_ = mass;
}

///////////////////////////////////////////////////////////////////////
/// -------------------------------Logs--------------------------------
///////////////////////////////////////////////////////////////////////

void NaiveOdometry::addToLogger(const mc_control::MCController &, mc_rtc::Logger & logger, const std::string & category)
{
  logger.addLogEntry(category + "_naive_fb_posW", [this]() -> const sva::PTransformd & { return X_0_fb_; });
  logger.addLogEntry(category + "_naive_fb_velW", [this]() -> const sva::MotionVecd & { return v_fb_0_; });
  logger.addLogEntry(category + "_naive_fb_accW", [this]() -> const sva::MotionVecd & { return a_fb_0_; });

  logger.addLogEntry(category + "_naive_fb_yaw",
                     [this]() -> const double
                     { return -so::kine::rotationMatrixToYawAxisAgnostic(X_0_fb_.rotation()); });

  logger.addLogEntry(category + "_constants_forceThreshold",
                     [this]() -> double { return mass_ * so::cst::gravityConstant * contactDetectionPropThreshold_; });
}

void NaiveOdometry::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_posW");
  logger.removeLogEntry(category + "_velW");
  logger.removeLogEntry(category + "_mass");
  logger.removeLogEntry(category + "_flexStiffness");
  logger.removeLogEntry(category + "_flexDamping");
}

void NaiveOdometry::addToGUI(const mc_control::MCController &,
                             mc_rtc::gui::StateBuilder & gui,
                             const std::vector<std::string> & category)
{
  using namespace mc_rtc::gui;
  // clang-format off

  // clang-format on
}

} // namespace mc_state_observation

EXPORT_OBSERVER_MODULE("NaiveOdometry", mc_state_observation::NaiveOdometry)