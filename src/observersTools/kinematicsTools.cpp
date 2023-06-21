#include <mc_state_observation/observersTools/kinematicsTools.h>

namespace so = stateObservation;

namespace mc_state_observation
{
namespace kinematicsTools
{

///////////////////////////////////////////////////////////////////////
/// -------------------Sva to Kinematics conversion--------------------
///////////////////////////////////////////////////////////////////////

so::kine::Kinematics poseFromSva(const sva::PTransformd & pTransform, so::kine::Kinematics::Flags::Byte zeroKine)
{
  so::kine::Kinematics kine;
  kine.setZero(zeroKine);
  kine.position = pTransform.translation();
  kine.orientation = so::Matrix3(pTransform.rotation().transpose());
  return kine;
}

so::kine::Kinematics poseAndVelFromSva(const sva::PTransformd & pTransform,
                                       const sva::MotionVecd & vel,
                                       bool velIsGlobal)
{
  so::kine::Kinematics kine;
  kine.position = pTransform.translation();
  kine.orientation = so::Matrix3(pTransform.rotation().transpose());
  switch(velIsGlobal)
  {
    case true: // the velocity is expressed in the global frame
      kine.linVel = vel.linear();
      kine.angVel = vel.angular();
      break;
    case false: // the velocity is expressed in the local frame
      kine.linVel = kine.orientation.toMatrix3() * vel.linear();
      kine.angVel = kine.orientation.toMatrix3() * vel.angular();
  }

  return kine;
}

so::kine::Kinematics kinematicsFromSva(const sva::PTransformd & pTransform,
                                       const sva::MotionVecd & vel,
                                       const sva::MotionVecd & acc,
                                       bool velIsGlobal,
                                       bool accIsGlobal)
{
  so::kine::Kinematics kine;
  kine.position = pTransform.translation();
  kine.orientation = so::Matrix3(pTransform.rotation().transpose());
  switch(velIsGlobal)
  {
    case true: // the velocity is expressed in the global frame
      kine.linVel = vel.linear();
      kine.angVel = vel.angular();
      break;
    case false: // the velocity is expressed in the local frame
      kine.linVel = kine.orientation.toMatrix3() * vel.linear();
      kine.angVel = kine.orientation.toMatrix3() * vel.angular();
  }
  switch(accIsGlobal)
  {
    case true: // the velocity is expressed in the global frame
      kine.linAcc = acc.linear();
      kine.angAcc = acc.angular();
      break;
    case false: // the velocity is expressed in the local frame
      kine.linAcc = kine.orientation.toMatrix3() * acc.linear();
      kine.angAcc = kine.orientation.toMatrix3() * acc.angular();
  }

  return kine;
}

so::kine::Kinematics & addVelocities(so::kine::Kinematics & kine, const sva::MotionVecd & vel, bool velIsGlobal)
{
  BOOST_ASSERT((kine.position.isSet() && kine.orientation.isSet())
               && "The position and the orientation are not set, please give them first");
  switch(velIsGlobal)
  {
    case true: // the velocity is expressed in the global frame
      kine.linVel = vel.linear();
      kine.angVel = vel.angular();
      break;
    case false: // the velocity is expressed in the local frame
      kine.linVel = kine.orientation.toMatrix3() * vel.linear();
      kine.angVel = kine.orientation.toMatrix3() * vel.angular();
  }

  return kine;
}

so::kine::Kinematics & addVelsAndAccs(so::kine::Kinematics & kine,
                                      const sva::MotionVecd & vel,
                                      const sva::MotionVecd & acc,
                                      bool velIsGlobal,
                                      bool accIsGlobal) // bodyAccB is local
{
  BOOST_ASSERT((kine.position.isSet() && kine.orientation.isSet())
               && "The position and the orientation are not set, please give them first");
  switch(velIsGlobal)
  {
    case true: // the velocity is expressed in the global frame
      kine.linVel = vel.linear();
      kine.angVel = vel.angular();
      break;
    case false: // the velocity is expressed in the local frame
      kine.linVel = kine.orientation.toMatrix3() * vel.linear();
      kine.angVel = kine.orientation.toMatrix3() * vel.angular();
  }
  switch(accIsGlobal)
  {
    case true: // the velocity is expressed in the global frame
      kine.linAcc = acc.linear();
      kine.angAcc = acc.angular();
      break;
    case false: // the velocity is expressed in the local frame
      kine.linAcc = kine.orientation.toMatrix3() * acc.linear();
      kine.angAcc = kine.orientation.toMatrix3() * acc.angular();
  }

  return kine;
}

} // namespace kinematicsTools
} // namespace mc_state_observation
