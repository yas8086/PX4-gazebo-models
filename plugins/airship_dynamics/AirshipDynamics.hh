#ifndef GZ_SIM_SYSTEMS_AIRSHIP_DYNAMICS_HH_
#define GZ_SIM_SYSTEMS_AIRSHIP_DYNAMICS_HH_

#include <gz/sim/System.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Matrix3.hh>
#include <gz/math/Pose3.hh>
#include <memory>
#include <mutex>

namespace gz
{
namespace sim
{
namespace systems
{

class AirshipDynamics :
  public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
public:
  AirshipDynamics();
  ~AirshipDynamics() override = default;

  void Configure(const Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &_ecm,
                 EventManager &_eventMgr) override;

  void PreUpdate(const UpdateInfo &_info,
                 EntityComponentManager &_ecm) override;

private:
  math::Vector3d LocalVelocity(math::Vector3d lin_vel,
                                math::Vector3d ang_vel,
                                math::Vector3d dist);
  double DynamicPressure(math::Vector3d vec);
  double Sign(double val);
  math::Matrix3d SkewSymmetric(math::Vector3d v);

  class Impl;
  std::unique_ptr<Impl> dataPtr;
};

}
}
}

#endif
