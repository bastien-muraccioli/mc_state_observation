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
  config("withFilteredForcesContactDetection", withFilteredForcesContactDetection_);

  std::string odometryType = static_cast<std::string>(config("odometryType"));
  if(odometryType == "flatOdometry")
  {
    withFlatOdometry_ = true;
  }
  else if(odometryType != "6dOdometry")
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Odometry type not allowed. Please pick among : [None, flatOdometry, 6dOdometry]");
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
    contacts_.insert(std::make_pair(forceSensor.name(), Contact(forceSensor.wrenchWithoutGravity(robot).force().z())));
  }

  mass(ctl.realRobot(robot_).mass());

  my_robots_ = mc_rbdyn::Robots::make();
  my_robots_->robotCopy(robot, robot.name());
  my_robots_->robotCopy(realRobot, "odometryRobot");
  ctl.gui()->addElement(
      {"Robots"},
      mc_rtc::gui::Robot("NaiveOdometry", [this]() -> const mc_rbdyn::Robot & { return my_robots_->robot(); }));

  X_0_fb_.translation() = realRobot.posW().translation();
}

bool NaiveOdometry::run(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);
  auto & odometryRobot = my_robots_->robot("odometryRobot");
  auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();

  odometryRobot.mbc() = realRobot.mbc();
  odometryRobot.mb() = realRobot.mb();

  X_0_fb_.rotation() = realRobot.posW().rotation();
  odometryRobot.velW(realRobot.velW());
  odometryRobot.accW(realRobot.accW());

  findContacts(ctl); // retrieves the list of contacts and set simStarted to true once a contact is detected

  /** Contacts
   * Note that when we use force sensors, this should be the position of the force sensor!
   */
  updateContacts(robot, odometryRobot, contactsFound_);

  X_0_fb_.rotation() = odometryRobot.posW().rotation();
  // X_0_fb_.translation() = mcko_K_0_fb.position();

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

std::set<std::string> NaiveOdometry::findContacts(const mc_control::MCController & ctl)
{
  const auto & measRobot = ctl.robot(robot_);
  auto & odometryRobot = my_robots_->robot("odometryRobot");

  contactsFound_.clear();

  if(withContactsDetection_)
  {
    for(const auto & contact : ctl.solver().contacts())
    {
      if(ctl.robots().robot(contact.r1Index()).name() == measRobot.name())
      {
        if(ctl.robots().robot(contact.r2Index()).mb().joint(0).type() == rbd::Joint::Fixed)
        {
          const auto & ifs = measRobot.indirectSurfaceForceSensor(contact.r1Surface()->name());
          if(ifs.wrenchWithoutGravity(measRobot).force().z() > measRobot.mass() * so::cst::gravityConstant * 0.05)
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
          if(ifs.wrenchWithoutGravity(measRobot).force().z() > measRobot.mass() * so::cst::gravityConstant * 0.05)
          {
            contactsFound_.insert(ifs.name());
          }
        }
      }
    }
  }
  else
  {
    for(auto forceSensor : measRobot.forceSensors())
    {
      auto forceSignal = contacts_.at(forceSensor.name());

      if(withFilteredForcesContactDetection_)
      {
        forceSignal.filteredForceZ_ =
            (1 - ctl.timeStep * forceSignal.lambda_) * forceSignal.filteredForceZ_
            + forceSignal.lambda_ * ctl.timeStep * forceSensor.wrenchWithoutGravity(measRobot).force().z();

        if(forceSignal.filteredForceZ_ > measRobot.mass() * so::cst::gravityConstant * 0.3)
        {
          contactsFound_.insert(forceSensor.name());
        }
      }
      else
      {
        if(forceSensor.wrenchWithoutGravity(measRobot).force().z() > measRobot.mass() * so::cst::gravityConstant * 0.3)
        {
          contactsFound_.insert(forceSensor.name());
        }
      }
    }
  }

  return contactsFound_;
}

void NaiveOdometry::setNewContact(const mc_rbdyn::Robot & robot,
                                  const mc_rbdyn::Robot & odometryRobot,
                                  const mc_rbdyn::ForceSensor forceSensor)
{
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

  so::kine::Kinematics worldNewContactKineOdometryRobot = worldBodyKineOdometryRobot * bodyNewContactKine;

  contacts_.at(forceSensor.name()).worldPositionKine_.position = worldNewContactKineOdometryRobot.position();
  if(withFlatOdometry_)
  {
    contacts_.at(forceSensor.name()).worldPositionKine_.position()(2) = 0.0;
  }
}

void NaiveOdometry::updateContacts(const mc_rbdyn::Robot & robot,
                                   const mc_rbdyn::Robot & odometryRobot,
                                   std::set<std::string> updatedContacts)
{
  /** Debugging output **/
  if(verbose_ && updatedContacts != oldContacts_)
    mc_rtc::log::info("[{}] Contacts changed: {}", name(), mc_rtc::io::to_string(updatedContacts));

  std::vector<std::string> alreadySetContacts;
  double sumAlreadySetForcesNorm = 0.0;
  so::Vector3 totalFbPosition = so::Vector3::Zero();
  for(const std::string & updatedContact : updatedContacts)
  {
    if(oldContacts_.find(updatedContact)
       != oldContacts_.end()) // checks if the contact already exists, if yes, it is updated
    {
      alreadySetContacts.push_back(updatedContact);
      const mc_rbdyn::ForceSensor & fs = robot.forceSensor(updatedContact);
      const sva::PTransformd & bodyAlreadySetContactPoseRobot = fs.X_p_f();
      so::kine::Kinematics bodyAlreadySetContactKine;
      bodyAlreadySetContactKine.setZero(so::kine::Kinematics::Flags::all);
      bodyAlreadySetContactKine.position = bodyAlreadySetContactPoseRobot.translation();
      bodyAlreadySetContactKine.orientation = so::Matrix3(bodyAlreadySetContactPoseRobot.rotation().transpose());

      so::kine::Kinematics worldBodyKineOdometryRobot;

      worldBodyKineOdometryRobot.position =
          odometryRobot.mbc().bodyPosW[odometryRobot.bodyIndexByName(fs.parentBody())].translation();
      worldBodyKineOdometryRobot.orientation = so::Matrix3(
          odometryRobot.mbc().bodyPosW[odometryRobot.bodyIndexByName(fs.parentBody())].rotation().transpose());

      so::kine::Kinematics worldAlreadySetContactKineOdometryRobot =
          worldBodyKineOdometryRobot * bodyAlreadySetContactKine;

      const so::Vector3 & worldAlreadySetContactPositionOdometryRobot =
          worldAlreadySetContactKineOdometryRobot.position();

      const so::Vector3 & worldFbOdometryRobot = odometryRobot.posW().translation();

      so::Vector3 worldFbOdometry = contacts_.at(updatedContact).worldPositionKine_.position()
                                    + (worldFbOdometryRobot - worldAlreadySetContactPositionOdometryRobot);

      totalFbPosition += worldFbOdometry * fs.wrenchWithoutGravity(odometryRobot).force().norm();

      sumAlreadySetForcesNorm +=
          fs.wrenchWithoutGravity(robot).force().norm(); // robot because odometryRobot has no sensors
    }
  }

  if(alreadySetContacts.size() > 0)
  {
    X_0_fb_.translation() = totalFbPosition / sumAlreadySetForcesNorm;
  }

  for(const std::string & updatedContact : updatedContacts)
  {
    if(std::find(alreadySetContacts.begin(), alreadySetContacts.end(), updatedContact) == alreadySetContacts.end())
    {
      const mc_rbdyn::ForceSensor & fs = odometryRobot.forceSensor(updatedContact);
      mapContacts_.insertPair(robot.forceSensor(updatedContact).name());
      setNewContact(robot, odometryRobot, fs);
    }
  }

  oldContacts_ = updatedContacts;
  unsigned nbContacts = static_cast<unsigned>(oldContacts_.size());
  if(debug_)
  {
    mc_rtc::log::info("nbContacts = {}", nbContacts);
  }
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
  logger.addLogEntry(category + "_mcko_fb_posW", [this]() -> const sva::PTransformd & { return X_0_fb_; });
  logger.addLogEntry(category + "_mcko_fb_velW", [this]() -> const sva::MotionVecd & { return v_fb_0_; });
  logger.addLogEntry(category + "_mcko_fb_accW", [this]() -> const sva::MotionVecd & { return a_fb_0_; });
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