#include <state-observation/dynamics-estimators/kinetics-observer.hpp>

namespace svaKinematicsConversion
{
namespace so = stateObservation;

so::kine::Kinematics kinematicsFromPtransformd(const sva::PTransformd & pTransform);
} // namespace svaKinematicsConversion