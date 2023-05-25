#include <mc_state_observation/svaKinematicsConversion.h>

namespace svaKinematicsConversion
{
so::kine::Kinematics svaKinematicsConversion::kinematicsFromPtransformd(const sva::PTransformd & pTransform)
{
  so::kine::Kinematics kine;
  kine.position = pTransform.translation();
  kine.orientation = so::Matrix3(pTransform.rotation().transpose());
}
} // namespace svaKinematicsConversion