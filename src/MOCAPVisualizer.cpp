/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#include <mc_control/MCController.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/io_utils.h>
#include <mc_rtc/logging.h>
#include <mc_state_observation/MOCAPVisualizer.h>
#include <mc_state_observation/gui_helpers.h>

#include <RBDyn/CoM.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <iostream>

namespace mc_state_observation
{
MOCAPVisualizer::MOCAPVisualizer(const std::string & type, double dt) : mc_observers::Observer(type, dt) {}

///////////////////////////////////////////////////////////////////////
/// --------------------------Core functions---------------------------
///////////////////////////////////////////////////////////////////////

void MOCAPVisualizer::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  std::string numExpe;
  config("numExpe", numExpe);
  csvPath_ = "/home/arnaud/Documents/KineticObserver/logs/expHRP5/" + numExpe
             + "/MoCap/c++Reader/resultingData/aligned_data.csv";
}

void MOCAPVisualizer::reset(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);

  my_robots_ = mc_rbdyn::Robots::make();
  my_robots_->robotCopy(robot, robot.name());
  ctl.gui()->addElement(
      {"Robots"},
      mc_rtc::gui::Robot("MOCAPVisualizer", [this]() -> const mc_rbdyn::Robot & { return my_robots_->robot(); }));
  ctl.gui()->addElement({"Robots"},
                        mc_rtc::gui::Robot("Real", [&ctl]() -> const mc_rbdyn::Robot & { return ctl.realRobot(); }));

  // MOCAP DATA TEST
  tempMocapData_.kine.setZero(so::kine::Kinematics::Flags::position | so::kine::Kinematics::Flags::orientation);
  extractMocapData();

  X_0_fb_ = realRobot.posW(); // we initialize the mocap robot with the real robot
  X_0_fb_init_ = realRobot.posW(); // we initialize the mocap robot with the real robot
}

bool MOCAPVisualizer::run(const mc_control::MCController & ctl)
{
  // std::cout << "\033[1;31m" << std::endl << "New iteration: " << std::endl << "\033[0m\n";
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);
  auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();

  std::string mocapTime = std::to_string(currentMocapDataTime_);
  if(mocapTime[mocapTime.find_last_not_of('0')] == '.')
  {
    mocapTime.erase(mocapTime.find_last_not_of('0') + 2, std::string::npos);
  }
  else
  {
    mocapTime.erase(mocapTime.find_last_not_of('0') + 1, std::string::npos);
  }
  if(mocapTime.back() == '.')
  {
    mocapTime.pop_back();
  }

  if(!mocapFinished)
  {
    if(mocapDataTable_.find(mocapTime) != mocapDataTable_.end())
    {

      tempMocapData_ = mocapDataTable_.at(mocapTime);
    }
    else
    {
      mocapFinished = true;
      logger.removeLogEntry(category_ + "_MOCAP_pos");
      logger.removeLogEntry(category_ + "_MOCAP_ori");
    }
  }
  currentMocapDataTime_ += ctl.timeStep;

  // tempMocapData_ is an increment in position and orientation on the initial pose
  std::cout << std::endl << "t: " << mocapTime << std::endl;
  std::cout << std::endl << "pos1: " << X_0_fb_.translation().transpose() << std::endl;
  std::cout << std::endl << "ori1: " << X_0_fb_.rotation().transpose() << std::endl;

  std::cout << std::endl << "dx: " << tempMocapData_.kine.position().transpose() << std::endl;
  std::cout << std::endl << "dtheta: " << tempMocapData_.kine.orientation.toMatrix3() << std::endl;

  so::kine::Kinematics newKine; // we initialize the new Kinematics with the initial ones
  newKine.position = X_0_fb_init_.translation();
  newKine.orientation = so::Matrix3(X_0_fb_init_.rotation().transpose());

  newKine = newKine * tempMocapData_.kine;
  /*
  so::Vector3 newPos = X_0_fb_init_.translation() + tempMocapData_.kine.position();
  so::Matrix3 newOri =
      (X_0_fb_init_.rotation().transpose() * tempMocapData_.kine.orientation.toMatrix3()).transpose();
  */

  // std::cout << std::endl << "newPos: " << newPos.transpose() << std::endl;
  // std::cout << std::endl << "newOri: " << newOri << std::endl;

  X_0_fb_.translation() = newKine.position;
  std::cout << std::endl << "pos2: " << X_0_fb_.translation().transpose() << std::endl;
  X_0_fb_.rotation() = newKine.orientation.toMatrix3().transpose();
  std::cout << std::endl << "ori2: " << X_0_fb_.rotation().transpose() << std::endl;

  my_robots_->robot().mbc().q = ctl.realRobot().mbc().q;
  update(my_robots_->robot());
}

///////////////////////////////////////////////////////////////////////
/// -------------------------Called functions--------------------------
///////////////////////////////////////////////////////////////////////

void MOCAPVisualizer::update(mc_control::MCController & ctl) // this function is called by the pipeline if the
                                                             // update is set to true in the configuration file
{
  auto & realRobot = ctl.realRobot(robot_);
  update(realRobot);
}

void MOCAPVisualizer::update(mc_rbdyn::Robot & robot)
{
  robot.posW(X_0_fb_);
  // robot.velW(v_fb_0_.vector());
}

///////////////////////////////////////////////////////////////////////
/// -------------------------------Logs--------------------------------
///////////////////////////////////////////////////////////////////////

void MOCAPVisualizer::addToLogger(const mc_control::MCController &,
                                  mc_rtc::Logger & logger,
                                  const std::string & category)
{
  logger.addLogEntry(category + "_MOCAP_pos",
                     [this]() -> const so::Vector3 & { return tempMocapData_.kine.position(); });
  logger.addLogEntry(category + "_MOCAP_ori",
                     [this]() -> so::Quaternion { return tempMocapData_.kine.orientation.toQuaternion().inverse(); });
  logger.addLogEntry(category + "_mocap_fb_posW", [this]() -> const sva::PTransformd & { return X_0_fb_; });
}

void MOCAPVisualizer::removeFromLogger(mc_rtc::Logger & logger, const std::string & category) {}

void MOCAPVisualizer::addToGUI(const mc_control::MCController &,
                               mc_rtc::gui::StateBuilder & gui,
                               const std::vector<std::string> & category)
{
  using namespace mc_rtc::gui;
  // clang-format off

  // clang-format on
}

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

void MOCAPVisualizer::extractMocapData()
{
  std::string fname;
  // cout << "Enter the file name: ";
  //  cin >> fname;
  fname = csvPath_;
  std::ofstream myfile;
  myfile.open("/home/arnaud/Documents/KineticObserver/logs/expHRP5/MoCap/c++Reader/example.txt");
  std::vector<std::vector<std::string>> content;
  std::vector<std::string> row;

  std::vector<std::vector<std::string>> newFile;
  // vector<string> newRow(8);
  std::string line, word;

  std::fstream file(fname, std::ios::in);

  if(file.is_open())
  {
    int i = 0;
    so::kine::Kinematics currentKine;
    currentKine.setZero(so::kine::Kinematics::Flags::position | so::kine::Kinematics::Flags::orientation);
    so::kine::Kinematics initKine;
    initKine.setZero(so::kine::Kinematics::Flags::position | so::kine::Kinematics::Flags::orientation);
    so::kine::Kinematics realignOrientation;
    realignOrientation.setZero(so::kine::Kinematics::Flags::position);
    so::Matrix3 realignmentMatrix = so::Matrix3::Zero();

    // realignmentMatrix(0, 0) = 1.0; // realignmentMatrix(0, 0) = 1.0; // realignmentMatrix(0, 1) = -1.0;
    // realignmentMatrix(1, 1) = 1.0; // realignmentMatrix(1, 1) = 1.0; // realignmentMatrix(1, 0) = 1.0;
    // realignmentMatrix(2, 2) = 1.0;
    // realignOrientation.orientation = realignmentMatrix;
    so::Matrix3 realignMat;
    realignMat << 0, 0, 1, 1, 0, 0, 0, 1, 0;
    realignOrientation.orientation =
        so::Matrix3(realignMat); // so::kine::rotationVectorToRotationMatrix(so::Vector3(0, 0, 0)
                                 //  * so::kine::rotationVectorToRotationMatrix(so::Vector3(0, 0, 0)));
    bool intitialized = false;
    while(getline(file, line))
    {
      row.clear();
      std::vector<std::string> newRow(8);
      std::stringstream str(line);

      while(getline(str, word, ',')) row.push_back(word);
      i++;
      if(i > 1)
      {
        so::Vector4 quat;
        if(mocapDataTable_.find(row.at(1)) != mocapDataTable_.end()) // if timecode already exists we compute the mean
        {
          /*          auto & alreadyExistingMocapData = mocapDataTable_.at(row.at(1));

          alreadyExistingMocapData.time = (alreadyExistingMocapData.time + std::stod(row.at(1))) / 2;
          quat(0) = (alreadyExistingMocapData.ori.toQuaternion().w() + std::stod(row.at(5))) / 2;
          quat(1) = (alreadyExistingMocapData.ori.toQuaternion().x() + std::stod(row.at(2))) / 2;
          quat(2) = (alreadyExistingMocapData.ori.toQuaternion().y() + std::stod(row.at(3))) / 2;
          quat(3) = (alreadyExistingMocapData.ori.toQuaternion().z() + std::stod(row.at(4))) / 2;
          tempMocapData_.ori.fromVector4(quat);
          alreadyExistingMocapData.pos(0) = (alreadyExistingMocapData.pos(0) + std::stod(row.at(6))) / 2;
          alreadyExistingMocapData.pos(1) = (alreadyExistingMocapData.pos(1) + std::stod(row.at(7))) / 2;
          alreadyExistingMocapData.pos(2) = (alreadyExistingMocapData.pos(2) + std::stod(row.at(8))) / 2;
          */
        }
        else // new row
        {
          tempMocapData_.indexReader = std::stoi(row.at(0));
          tempMocapData_.time = std::stoi(row.at(1));
          quat(0) = std::stod(row.at(5)); // w
          quat(1) = std::stod(row.at(2)); // x
          quat(2) = std::stod(row.at(3)); // y
          quat(3) = std::stod(row.at(4)); // z

          currentKine.orientation = Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3));

          so::kine::Orientation oriTransfo( // so::Quaternion(Eigen::AngleAxis(-M_PI_2, Eigen::Vector3d::UnitZ()))
                                            //*
              so::Quaternion(quat(0), quat(1), quat(2), quat(3)));

          {

            so::Matrix3 tempOri;
            tempOri << 1, 0, 0, 0, 0, -1, 0, 1, 0; // 0, 0, -1, 0, 1, 0, 1, 0, 0;
            so::kine::Orientation fixOrientations(tempOri);
            oriTransfo = so::Matrix3(
                tempOri * so::Quaternion(Eigen::AngleAxis(M_PI_2, Eigen::Vector3d::UnitY())).toRotationMatrix()
                * oriTransfo.toMatrix3() * tempOri.transpose());

            /*
             oriTransfo.toQuaternion();
             oriTransfo.getMatrixRefUnsafe().reset();
             oriTransfo.getQuaternionRefUnsafe().coeffs().head<3>() =
                 tempOri * oriTransfo.toQuaternion().coeffs().head<3>();
                 */
          }

          currentKine.orientation = oriTransfo;

          currentKine.position()(0) = std::stod(row.at(6));
          currentKine.position()(1) = std::stod(row.at(7));
          currentKine.position()(2) = std::stod(row.at(8));
          std::cout << std::endl << "currentKine.position1: " << currentKine.position().transpose() << std::endl;

          currentKine.position = realignOrientation.orientation * currentKine.position();
          // currentKine = realignOrientation * currentKine;

          std::cout << std::endl << "realignOrientation: " << realignOrientation.orientation.toMatrix3() << std::endl;
          std::cout << std::endl << "currentKine.position2: " << currentKine.position().transpose() << std::endl;

          if(!intitialized)
          {
            initKine = currentKine;
            tempMocapData_.kine.position().setZero();
            tempMocapData_.kine.orientation.setZeroRotation();

            intitialized = true;
          }

          tempMocapData_.kine = initKine.getInverse() * currentKine;

          mocapDataTable_.insert(std::make_pair(row.at(1), tempMocapData_));

          newRow.at(0) = row.at(1);
          newRow.at(1) = row.at(2);
          newRow.at(2) = row.at(3);
          newRow.at(3) = row.at(4);
          newRow.at(4) = row.at(5);
          newRow.at(5) = row.at(6);
          newRow.at(6) = row.at(7);
          newRow.at(7) = row.at(8);

          newFile.push_back(newRow);

          for(int j = 0; j < newRow.size(); j++)
          {
            myfile << newRow[j] << " ";
          }
          myfile << std::endl;
        }
      }
    }
    myfile.close();
  }
  else
    std::cout << "Could not open the file\n";
}

} // namespace mc_state_observation

EXPORT_OBSERVER_MODULE("MOCAPVisualizer", mc_state_observation::MOCAPVisualizer)