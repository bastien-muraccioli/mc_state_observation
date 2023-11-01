#include <mc_control/MCController.h>
#include <mc_state_observation/MocapObserver.h>
#include <mc_state_observation/gui_helpers.h>

namespace mc_state_observation
{

MocapObserver::MocapObserver(const std::string & type, double dt) : mc_observers::Observer(type, dt) {}

void MocapObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  updateRobot_ = robot_;
  config("useReal", useReal_);
  config("updateRobot", updateRobot_);
  body_ = config("body", robot(ctl).mb().body(0).name());
  desc_ = name_ + " (Marker TF: {} -> {})";
}

void MocapObserver::reset(const mc_control::MCController & ctl)
{
  calibrated_ = false;
  run(ctl);
}

bool MocapObserver::run(const mc_control::MCController &)
{
  if(!calibrated_) { error_ = fmt::format("[{}] Please calibrate the body to marker pose first", name()); }
  X_0_marker_ = X_m_marker_ * X_0_mocap_;
  return calibrated_;
}

void MocapObserver::update(mc_control::MCController & ctl)
{
  auto & updateRobot = ctl.realRobot(updateRobot_);

  auto X_0_body = updateRobot.bodyPosW(body_);
  auto X_body_fb = updateRobot.posW() * X_0_body.inv();
  // Note 1: This renormalizes and reorthogonalizes the rotation matrix to
  // avoid numerical errors slowly adding up over the course of many
  // iterations. It would however be better to compute the relative pose
  // X_b_fb withoug passing through the world frame, which would not be
  // subject from this denormalization issue.
  // Note 2: This issue only occurs for fixed-base robots, as the posW()
  // performs the quaternion conversion and normalization itself
  {
    Eigen::Quaterniond rot(X_body_fb.rotation());
    rot.normalize();
    X_body_fb.rotation() = rot.toRotationMatrix();
  }
  X_0_fb_ = X_body_fb * X_marker_body_ * X_0_marker_;
  updateRobot.posW(X_0_fb_);
}

void MocapObserver::addToLogger(const mc_control::MCController &, mc_rtc::Logger & logger, const std::string & category)
{
  logger.addLogEntry(category + "_MocapToMarker", [this]() -> const sva::PTransformd & { return X_m_marker_; });
  logger.addLogEntry(category + "_markerPosW", [this]() -> const sva::PTransformd & { return X_0_marker_; });
  logger.addLogEntry(category + "_posW", [this]() -> const sva::PTransformd & { return X_0_fb_; });
  logger.addLogEntry(category + "_MocapOrigin", [this]() -> const sva::PTransformd & { return X_0_mocap_; });
  logger.addLogEntry(category + "_marker_to_body", [this]() -> const sva::PTransformd & { return X_marker_body_; });
}

void MocapObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_MocapToMarker");
  logger.removeLogEntry(category + "_markerPosW");
  logger.removeLogEntry(category + "_posW");
  logger.removeLogEntry(category + "_MocapOrigin");
  logger.removeLogEntry(category + "_marker_to_body");
}

void MocapObserver::addToGUI(const mc_control::MCController & ctl,
                             mc_rtc::gui::StateBuilder & gui,
                             const std::vector<std::string> & category)
{
  gui.addElement(
      category,
      mc_rtc::gui::Button("Calibrate Marker (put robot at mocap origin)",
                          [this, &ctl]()
                          {
                            mc_rtc::log::info("[{}] Calibration triggerred", name());
                            calibrated_ = calibrateMarkerToBody(ctl);
                          }),
      mc_rtc::gui::Button("Initialize origin",
                          [this, &ctl]()
                          {
                            mc_rtc::log::info("[{}] Initialize origin triggerred", name());
                            originInitialized_ = initializeOrigin(ctl);
                          }),
      mc_rtc::gui::Button("Reset",
                          [this, &ctl]()
                          {
                            mc_rtc::log::info("[{}] Manual reset triggerred", name());
                            reset(ctl);
                          }),
      mc_rtc::gui::Transform(
          "Mocap Origin", [this]() -> const sva::PTransformd & { return X_0_mocap_; },
          [this](const sva::PTransformd & pose) { X_0_mocap_ = pose; }),
      mc_rtc::gui::Transform("Mocap Marker World", [this]() -> const sva::PTransformd & { return X_0_marker_; }),
      mc_rtc::gui::Transform("Mocap Marker Frame",
                             [this, &ctl]() -> const sva::PTransformd
                             {
                               auto & realRobot = ctl.realRobot(updateRobot_);
                               auto X_0_body = realRobot.bodyPosW(body_);
                               return X_marker_body_.inv() * X_0_body;
                             }));
}

bool MocapObserver::checkPipelines(const mc_control::MCController & ctl)
{
  bool pipelineSuccess = true;
  for(const auto & pipeline : ctl.observerPipelines())
  {
    if(!pipeline.hasObserver(name_)) { pipelineSuccess = pipelineSuccess && pipeline.success(); }
  }
  return pipelineSuccess;
}

bool MocapObserver::calibrateMarkerToBody(const mc_control::MCController & ctl)
{
  if(!checkPipelines(ctl))
  {
    mc_rtc::log::error("[{}] Calibration failed: other pipelines are not ready", name());
    return false;
  }
  if(!gotMarker_)
  {
    mc_rtc::log::error("[{}] Calibration failed: didn't receive any mocap marker pose yet", name());
    return false;
  }

  auto X_0_body = robot(ctl).bodyPosW(body_);
  // In general, the robot should be placed at the origin of the mocap system
  // before this operation.
  // If this is not the case, set X_0_mocap_ appropriately manually
  X_marker_body_ = X_0_body * X_0_mocap_.inv() * X_m_marker_.inv();
  mc_rtc::log::success("[{}] calibrated.", name());
  mc_rtc::log::info("[{}] Transformation between mocap marker and body {} \ntranslation: {}\nrotation: {}", name(),
                    body_, X_marker_body_.translation().transpose(),
                    mc_rbdyn::rpyFromMat(X_marker_body_.rotation()).transpose());
  return true;
}

bool MocapObserver::initializeOrigin(const mc_control::MCController & ctl)
{
  if(!checkPipelines(ctl)) return false;
  if(!gotMarker_) return false;

  auto X_0_body = robot(ctl).bodyPosW(body_);
  X_0_mocap_ = X_m_marker_.inv() * X_marker_body_.inv() * X_0_body;
  mc_rtc::log::success("[{}] World to mocap transformation.\ntranslation: {}\nrotation: {}", name(),
                       X_0_mocap_.translation().transpose(), mc_rbdyn::rpyFromMat(X_0_mocap_.rotation()).transpose());
  return true;
}

} // namespace mc_state_observation
