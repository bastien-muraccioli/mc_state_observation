#include <mc_state_observation/MocapObserver.h>

#include <mc_state_observation/ros.h>

#include <tf2_ros/transform_listener.h>

namespace mc_state_observation
{

struct MocapObserverROS : public MocapObserver
{
  MocapObserverROS(const std::string & type, double dt);

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

protected:
  std::string marker_ = "mocap/base_link"; ///< Name of the marker
  std::string markerOrigin_ = "mocap"; ///< Name of the origin frame for the marker

  mc_rtc::NodeHandlePtr nh_ = nullptr;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_{tfBuffer_};
};

} // namespace mc_state_observation
