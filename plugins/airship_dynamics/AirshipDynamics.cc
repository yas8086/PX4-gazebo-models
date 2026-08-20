#include "AirshipDynamics.hh"

#include <string>
#include <cmath>
#include <limits>
#include <cstdio>


#include <gz/plugin/Register.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/Utility.hh>

#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>

#include <gz/math/Matrix3.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

using namespace gz;
using namespace sim;
using namespace systems;

class AirshipDynamics::Impl
{
public:
  Entity linkEntity{kNullEntity};
  Entity modelEntity{kNullEntity};

  double airDensity{1.225};
  double hullVolume{1.0};
  double netBuoyancy{0.0};
  double effectiveVolume{1.0};

  double forceInviscidCoeff{0.0};
  double forceViscousCoeff{0.0};
  double momentInviscidCoeff{0.0};
  double momentViscousCoeff{0.0};
  double epsV{1.0};
  double axialDragCoeff{0.0};

  // 旋转阻尼系数 (N·m·s/rad)
  // 飞艇大表面积旋转时推动空气产生阻尼力矩: M = -C_rot * omega
  // 估算: 0.5 * rho * Cd * A * L^2
  // Y轴(俯仰): ~118000, Z轴(偏航): ~150000, X轴(滚转): ~50000
  double rotDampingX{50000.0};
  double rotDampingY{118000.0};
  double rotDampingZ{150000.0};

  double distCOV{0.0};

  bool tetherEnabled{false};
  double tetherStiffness{0.0};
  double tetherDamping{0.0};
  math::Vector3d tetherAnchor{0, 0, 0};

  // 浮力作用点偏移 (相对link原点, body frame)
  // 默认(0,0,0)=link原点, 设为质心位置则浮力与重力共线
  math::Vector3d buoyancyOffset{0, 0, 0};

  // 浮力中心 (相对link原点, body frame)
  // 浮力在此点施加, 产生恢复力矩(摆锤效应)
  // 对于飞艇, 浮力中心通常在质心上方
  math::Vector3d buoyancyCenter{0, 0, 0};

  math::Matrix3d m11;
  math::Matrix3d m12;
  math::Matrix3d m21;
  math::Matrix3d m22;

  transport::Node node;
  math::Vector3d windVector{0, 0, 0};
  std::mutex mtx;

  // 动态净浮力命令 (由 ballast_control 通过 gz_transport 发布)
  double netBuoyancyCmd{0.0};
  bool netBuoyancyCmdValid{false};

  // 四气囊空气质量 (kg) - 索引: 0=LI, 1=LO, 2=RI, 3=RO
  // 由 PX4 ballast_control 积分估计并通过 ballast_actuator topic 发送
  // 4气囊完全同步充放气, 总质量作为可变载荷影响垂直浮力
  double ballastMass[4]{0.0, 0.0, 0.0, 0.0};

  // 四气囊执行器状态 (0或1) - V2: 每囊1风机+1阀门
  // 充气 = 风机+阀门(鼓风), 排气 = 仅阀门(自然排气)
  uint8_t blowerState[4]{0, 0, 0, 0};
  uint8_t valveState[4]{0, 0, 0, 0};

  // 单气囊最大质量 (kg)
  double ballastMassMax{128.5};

  // === P3c: 动态惯量 (气囊质量计入 base_link Inertial) ===
  // 原始 base_link 质量/惯量 (Configure时从Inertial读取, 不随气囊变化)
  double baseMass{0.0};
  double baseIxx{0.0};
  double baseIyy{0.0};
  double baseIzz{0.0};
  // 上一次设置的动态质量 (避免每帧重复SetComponentData)
  double lastSetMass{-1.0};

  void UpdateWind(const msgs::Vector3d &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mtx);
    this->windVector = gz::msgs::Convert(_msg);
  }

  void UpdateBallast(const msgs::Vector3d &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mtx);
    // x = net_buoyancy 调整量 (N), y 已废弃
    this->netBuoyancyCmd = _msg.x();
    this->netBuoyancyCmdValid = true;
  }

  void UpdateBallastActuator(const msgs::Vector3d &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mtx);
    // x = 气囊索引 (0=左主囊, 1=右主囊, 2=左副囊, 3=右副囊)
    // y = 执行器状态位图: bit0=blower(风机), bit1=valve(阀门)
    // z = 当前空气质量 (kg)
    int idx = static_cast<int>(_msg.x());
    if (idx < 0 || idx > 3) return;

    uint8_t state = static_cast<uint8_t>(_msg.y());
    // V2: bit0=blower(风机), bit1=valve(阀门)
    blowerState[idx] = state & 0x01;
    valveState[idx] = (state >> 1) & 0x01;

    ballastMass[idx] = _msg.z();
  }
};

AirshipDynamics::AirshipDynamics()
    : dataPtr(std::make_unique<Impl>())
{
}

math::Matrix3d AirshipDynamics::SkewSymmetric(math::Vector3d v)
{
  return math::Matrix3d(
      0, -v.Z(), v.Y(),
      v.Z(), 0, -v.X(),
      -v.Y(), v.X(), 0);
}

math::Vector3d AirshipDynamics::LocalVelocity(
    math::Vector3d lin_vel, math::Vector3d ang_vel, math::Vector3d dist)
{
  return lin_vel + ang_vel.Cross(dist);
}

double AirshipDynamics::DynamicPressure(math::Vector3d vec)
{
  return 0.5 * this->dataPtr->airDensity * vec.SquaredLength();
}

double AirshipDynamics::Sign(double val)
{
  return (val >= 0.0) ? 1.0 : -1.0;
}

void AirshipDynamics::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager & /*_eventMgr*/)
{
  auto model = Model(_entity);
  this->dataPtr->modelEntity = _entity;

  if (!_sdf->HasElement("link_name"))
  {
    gzerr << "[AirshipDynamics] Please specify a <link_name>." << std::endl;
    return;
  }
  auto linkName = _sdf->Get<std::string>("link_name");
  this->dataPtr->linkEntity = model.LinkByName(_ecm, linkName);
  if (this->dataPtr->linkEntity == kNullEntity)
  {
    gzerr << "[AirshipDynamics] Link [" << linkName << "] not found." << std::endl;
    return;
  }

  if (_sdf->HasElement("air_density"))
    this->dataPtr->airDensity = _sdf->Get<double>("air_density");

  if (_sdf->HasElement("hull_volume"))
    this->dataPtr->hullVolume = _sdf->Get<double>("hull_volume");

  if (_sdf->HasElement("net_buoyancy"))
    this->dataPtr->netBuoyancy = _sdf->Get<double>("net_buoyancy");

  if (_sdf->HasElement("buoyancy_offset"))
    this->dataPtr->buoyancyOffset = _sdf->Get<math::Vector3d>("buoyancy_offset");

  if (_sdf->HasElement("buoyancy_center"))
    this->dataPtr->buoyancyCenter = _sdf->Get<math::Vector3d>("buoyancy_center");
  else
    this->dataPtr->buoyancyCenter = this->dataPtr->buoyancyOffset;

  if (_sdf->HasElement("force_inviscid_coeff"))
    this->dataPtr->forceInviscidCoeff = _sdf->Get<double>("force_inviscid_coeff");

  if (_sdf->HasElement("force_viscous_coeff"))
    this->dataPtr->forceViscousCoeff = _sdf->Get<double>("force_viscous_coeff");

  if (_sdf->HasElement("moment_inviscid_coeff"))
    this->dataPtr->momentInviscidCoeff = _sdf->Get<double>("moment_inviscid_coeff");

  if (_sdf->HasElement("moment_viscous_coeff"))
    this->dataPtr->momentViscousCoeff = _sdf->Get<double>("moment_viscous_coeff");

  if (_sdf->HasElement("eps_v"))
    this->dataPtr->epsV = _sdf->Get<double>("eps_v");

  if (_sdf->HasElement("axial_drag_coeff"))
    this->dataPtr->axialDragCoeff = _sdf->Get<double>("axial_drag_coeff");

  if (_sdf->HasElement("rot_damping_x"))
    this->dataPtr->rotDampingX = _sdf->Get<double>("rot_damping_x");
  if (_sdf->HasElement("rot_damping_y"))
    this->dataPtr->rotDampingY = _sdf->Get<double>("rot_damping_y");
  if (_sdf->HasElement("rot_damping_z"))
    this->dataPtr->rotDampingZ = _sdf->Get<double>("rot_damping_z");

  if (_sdf->HasElement("dist_cov"))
    this->dataPtr->distCOV = _sdf->Get<double>("dist_cov");

  if (_sdf->HasElement("tether_stiffness"))
  {
    this->dataPtr->tetherEnabled = true;
    this->dataPtr->tetherStiffness = _sdf->Get<double>("tether_stiffness");
  }
  if (_sdf->HasElement("tether_damping"))
    this->dataPtr->tetherDamping = _sdf->Get<double>("tether_damping");
  if (_sdf->HasElement("tether_anchor"))
  {
    auto anchorVec = _sdf->Get<math::Vector3d>("tether_anchor");
    this->dataPtr->tetherAnchor = anchorVec;
  }

  // 附加质量矩阵映射 (Kirchhoff方程命名约定):
  //   m11/m22/m33 = 线性附加质量 (X/Y/Z方向) -> M11对角 (力矩阵)
  //   m44/m55/m66 = 附加转动惯量 (roll/pitch/yaw轴) -> M22对角 (力矩矩阵)
  //   m26/m35/m53/m62 = 线性-角速度耦合项 -> M12/M21
  // V2修复: 原实现将m22(Y向线性附加质量1496)误写入m22(1,1)(力矩矩阵),
  // 随即被m55=5000覆盖 -> M11(1,1)=0, Y向线性附加质量丢失,
  // 导致横滚Munk力矩符号反转(-787*vy*vz, 正确应为+709*vy*vz),
  // 快速爬升+侧漂时产生数千N·m的反向横滚力矩(起飞倾斜失控根因之一).
  if (_sdf->HasElement("m11"))
    this->dataPtr->m11(0, 0) = _sdf->Get<double>("m11");
  if (_sdf->HasElement("m22"))
    this->dataPtr->m11(1, 1) = _sdf->Get<double>("m22");
  if (_sdf->HasElement("m26"))
    this->dataPtr->m12(1, 2) = _sdf->Get<double>("m26");
  if (_sdf->HasElement("m33"))
    this->dataPtr->m11(2, 2) = _sdf->Get<double>("m33");
  if (_sdf->HasElement("m35"))
    this->dataPtr->m12(2, 1) = _sdf->Get<double>("m35");
  if (_sdf->HasElement("m44"))
    this->dataPtr->m22(0, 0) = _sdf->Get<double>("m44");
  if (_sdf->HasElement("m53"))
    this->dataPtr->m21(1, 2) = _sdf->Get<double>("m53");
  if (_sdf->HasElement("m55"))
    this->dataPtr->m22(1, 1) = _sdf->Get<double>("m55");
  if (_sdf->HasElement("m62"))
    this->dataPtr->m21(2, 1) = _sdf->Get<double>("m62");
  if (_sdf->HasElement("m66"))
    this->dataPtr->m22(2, 2) = _sdf->Get<double>("m66");

  if (!_ecm.Component<components::WorldPose>(this->dataPtr->linkEntity))
    _ecm.CreateComponent(this->dataPtr->linkEntity, components::WorldPose());
  if (!_ecm.Component<components::Inertial>(this->dataPtr->linkEntity))
    _ecm.CreateComponent(this->dataPtr->linkEntity, components::Inertial());
  if (!_ecm.Component<components::AngularVelocity>(this->dataPtr->linkEntity))
    _ecm.CreateComponent(this->dataPtr->linkEntity, components::AngularVelocity());
  if (!_ecm.Component<components::WorldAngularVelocity>(this->dataPtr->linkEntity))
    _ecm.CreateComponent(this->dataPtr->linkEntity, components::WorldAngularVelocity());
  if (!_ecm.Component<components::WorldLinearVelocity>(this->dataPtr->linkEntity))
    _ecm.CreateComponent(this->dataPtr->linkEntity, components::WorldLinearVelocity());

  // 动态获取 world 名字, 构建 wind topic (避免硬编码)
  // 默认 world 名字为 "default", 飞艇仿真使用 "airship_world" 等
  std::string worldName = "default";
  Entity worldEntity = _ecm.EntityByComponents(components::World());

  if (worldEntity != kNullEntity) {
    auto nameComp = _ecm.Component<components::Name>(worldEntity);

    if (nameComp) {
      worldName = nameComp->Data();
    }
  }

  std::string windTopic = "/world/" + worldName + "/wind";
  this->dataPtr->node.Subscribe(windTopic,
      &Impl::UpdateWind, this->dataPtr.get());
  gzmsg << "[AirshipDynamics] Subscribed to wind topic: " << windTopic << std::endl;

  // 订阅 ballast_control 发布的动态浮力命令
  // topic 格式: /model/{model_name}/ballast_cmd
  // 消息类型: gz::msgs::Vector3d (x=net_buoyancy N, y 已废弃)
  std::string modelName = model.Name(_ecm);
  std::string ballastTopic = "/model/" + modelName + "/ballast_cmd";
  this->dataPtr->node.Subscribe(ballastTopic,
      &Impl::UpdateBallast, this->dataPtr.get());
  gzmsg << "[AirshipDynamics] Subscribed to ballast topic: " << ballastTopic << std::endl;

  // 订阅 ballast_control 发布的四气囊执行器状态
  // topic 格式: /model/{model_name}/ballast_actuator
  // 消息类型: gz::msgs::Vector3d (x=气囊索引, y=执行器状态位图, z=空气质量kg)
  std::string ballastActuatorTopic = "/model/" + modelName + "/ballast_actuator";
  this->dataPtr->node.Subscribe(ballastActuatorTopic,
      &Impl::UpdateBallastActuator, this->dataPtr.get());
  gzmsg << "[AirshipDynamics] Subscribed to ballast_actuator topic: " << ballastActuatorTopic << std::endl;

  // 四气囊参数 (可从SDF覆盖默认值)
  if (_sdf->HasElement("ballast_mass_max"))
    this->dataPtr->ballastMassMax = _sdf->Get<double>("ballast_mass_max");

  auto gravityComp = _ecm.Component<components::Gravity>(
      _ecm.EntityByComponents(components::World()));
  double g = 9.8;
  if (gravityComp)
    g = gravityComp->Data().Length();

  // 计算总质量: base_link + 所有子link(电机等)
  // Gazebo对所有link施加重力, 所以浮力必须补偿总重力
  double mass = 0.0;
  auto baseLinkInertial = _ecm.Component<components::Inertial>(this->dataPtr->linkEntity);
  if (baseLinkInertial)
    mass = baseLinkInertial->Data().MassMatrix().Mass();

  // P3c: 保存原始 base_link 质量/惯量 (气囊质量动态计入)
  // 在Configure时从Inertial读取, 作为动态惯量更新的基准(不随气囊变化)
  if (baseLinkInertial) {
    auto baseMM = baseLinkInertial->Data().MassMatrix();
    this->dataPtr->baseMass = baseMM.Mass();
    this->dataPtr->baseIxx = baseMM.Ixx();
    this->dataPtr->baseIyy = baseMM.Iyy();
    this->dataPtr->baseIzz = baseMM.Izz();
    this->dataPtr->lastSetMass = this->dataPtr->baseMass;
  }

  // 遍历模型的所有link, 累加质量
  double totalMass = mass;
  auto modelLinks = Model(_entity).Links(_ecm);
  for (const auto &linkEnt : modelLinks) {
    if (linkEnt != this->dataPtr->linkEntity) {
      auto linkInertial = _ecm.Component<components::Inertial>(linkEnt);
      if (linkInertial) {
        totalMass += linkInertial->Data().MassMatrix().Mass();
      }
    }
  }

  // 浮力只施加在base_link上, 需要补偿所有link的总重力
  // 因为子link的重力通过joint传递给base_link, 最终由base_link承受
  // 所以浮力 = 总重力 + net_buoyancy
  // 浮力基于物理体积(hullVolume)计算

  gzmsg << "[AirshipDynamics] base_link mass: " << mass << " kg" << std::endl;
  gzmsg << "[AirshipDynamics] total mass (all links): " << totalMass << " kg" << std::endl;

  double fullBuoyancy = this->dataPtr->airDensity * this->dataPtr->hullVolume * g;

  // 浮力基于物理体积(hullVolume)计算, 不人为平衡
  // net_buoyancy参数: 额外浮力微调(正值=更多浮力=上升, 负值=更少浮力=下降)
  // effectiveVolume = hullVolume + net_buoyancy / (ρ×g)
  // 当net_buoyancy=0时, effectiveVolume = hullVolume, 浮力 = ρ×V×g
  if (std::abs(this->dataPtr->netBuoyancy) < 1e-6)
  {
    this->dataPtr->effectiveVolume = this->dataPtr->hullVolume;
  }
  else
  {
    this->dataPtr->effectiveVolume = this->dataPtr->hullVolume +
        this->dataPtr->netBuoyancy / (this->dataPtr->airDensity * g);
  }

  gzmsg << "[AirshipDynamics] Configured:" << std::endl;
  gzmsg << "  link_name: " << linkName << std::endl;
  gzmsg << "  air_density: " << this->dataPtr->airDensity << std::endl;
  gzmsg << "  hull_volume: " << this->dataPtr->hullVolume << std::endl;
  gzmsg << "  base_link_mass: " << mass << std::endl;
  gzmsg << "  total_mass: " << totalMass << std::endl;
  gzmsg << "  gravity: " << g << std::endl;
  gzmsg << "  total_weight: " << totalMass * g << " N" << std::endl;
  gzmsg << "  full_buoyancy: " << fullBuoyancy << " N" << std::endl;
  gzmsg << "  net_buoyancy: " << this->dataPtr->netBuoyancy << " N" << std::endl;
  gzmsg << "  buoyancy_offset: " << this->dataPtr->buoyancyOffset << std::endl;
  gzmsg << "  buoyancy_center: " << this->dataPtr->buoyancyCenter << std::endl;
  gzmsg << "  hull_volume: " << this->dataPtr->hullVolume << " m^3" << std::endl;
  gzmsg << "  effective_volume: " << this->dataPtr->effectiveVolume << " m^3" << std::endl;
  gzmsg << "  effective_buoyancy: "
        << this->dataPtr->airDensity * this->dataPtr->effectiveVolume * g
        << " N" << std::endl;
  gzmsg << "  eps_v: " << this->dataPtr->epsV << std::endl;
  gzmsg << "  axial_drag_coeff: " << this->dataPtr->axialDragCoeff << std::endl;
  gzmsg << "  rot_damping_x: " << this->dataPtr->rotDampingX << " N·m·s/rad" << std::endl;
  gzmsg << "  rot_damping_y: " << this->dataPtr->rotDampingY << " N·m·s/rad" << std::endl;
  gzmsg << "  rot_damping_z: " << this->dataPtr->rotDampingZ << " N·m·s/rad" << std::endl;
  gzmsg << "  dist_cov: " << this->dataPtr->distCOV << std::endl;
  gzmsg << "  ballast_mass_max: " << this->dataPtr->ballastMassMax << " kg" << std::endl;
  gzmsg << "  tether_enabled: " << (this->dataPtr->tetherEnabled ? "true" : "false")
        << std::endl;
  if (this->dataPtr->tetherEnabled)
  {
    gzmsg << "  tether_stiffness: " << this->dataPtr->tetherStiffness << std::endl;
    gzmsg << "  tether_damping: " << this->dataPtr->tetherDamping << std::endl;
    gzmsg << "  tether_anchor: " << this->dataPtr->tetherAnchor << std::endl;
  }
}

void AirshipDynamics::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  Link baseLink(this->dataPtr->linkEntity);

  auto linearVelocityComp = _ecm.Component<components::WorldLinearVelocity>(
      this->dataPtr->linkEntity);
  if (!linearVelocityComp)
    return;

  auto pose = baseLink.WorldPose(_ecm);
  if (!pose)
    return;

  auto worldLinVel = linearVelocityComp->Data();
  auto worldAngVel = baseLink.WorldAngularVelocity(_ecm);
  if (!worldAngVel)
    return;

  math::Vector3d linVel = pose->Rot().Inverse() * worldLinVel;
  math::Vector3d angVel = pose->Rot().Inverse() * *worldAngVel;

  math::Vector3d windBody{0, 0, 0};
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mtx);
    windBody = pose->Rot().Inverse() * this->dataPtr->windVector;
  }

  // === 1. Buoyancy Force ===
  // F_buoyancy = -rho_air * V_hull * g (in world frame, upward)
  // Then transform to body frame
  // 支持动态浮力调节: 当 ballast_control 发布 net_buoyancy 命令时,
  // 使用动态值替代静态 netBuoyancy
  auto gravityComp = _ecm.Component<components::Gravity>(
      _ecm.EntityByComponents(components::World()));
  math::Vector3d gravity{0, 0, -9.8};
  if (gravityComp)
    gravity = gravityComp->Data();

  // 计算当前有效浮力体积
  double currentEffectiveVolume = this->dataPtr->effectiveVolume;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mtx);
    if (this->dataPtr->netBuoyancyCmdValid) {
      // 动态浮力: effectiveVolume = hullVolume + (netBuoyancy + netBuoyancyCmd) / (rho * g)
      // V2修复: 静态 netBuoyancy(model.sdf, 基准微调补偿)必须与动态 netBuoyancyCmd 叠加,
      // 否则 ballast_control 周期发布 net_buoyancy 会完全覆盖静态参数(补偿失效)
      currentEffectiveVolume = this->dataPtr->hullVolume +
          (this->dataPtr->netBuoyancy + this->dataPtr->netBuoyancyCmd) /
          (this->dataPtr->airDensity * gravity.Length());
    }
  }

  math::Vector3d buoyancyWorld = -this->dataPtr->airDensity *
                                  currentEffectiveVolume * gravity;
  math::Vector3d buoyancyBody = pose->Rot().RotateVectorReverse(buoyancyWorld);

  // === 1.5 Four-Ballast Mass Weight (四气囊统一浮力调节) ===
  // 4气囊完全同步充放气, 总空气质量作为可变载荷
  // 充气(增重) -> 下沉; 放气(减重) -> 上升
  // P3c修改: 气囊质量动态计入 base_link Inertial 组件(见下方),
  // 由Gazebo物理引擎自动施加气囊重力(作用在质心, 不产生倾斜力矩).
  // 原V2实现手动施加 ballastWeightBody, 但Inertial质量不随气囊变化,
  // 导致加速度响应偏差(气囊最大514kg, 占2206kg的23%).
  // 现在改为动态惯量, 移除手动施力(避免双重计重).
  double totalBallastMass = 0.0;
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->mtx);
    for (int i = 0; i < 4; i++) {
      totalBallastMass += this->dataPtr->ballastMass[i];
    }
  }

  // === P3c: 气囊质量计入 base_link 惯量 (动态更新) ===
  // 气囊作为质点载荷分布在左右两侧(Y方向), 用平行轴定理计算惯量增量:
  //   四囊中心Y坐标(FLU, 相对base_link原点): LI=+3.3, LO=+8.6, RI=-3.3, RO=-8.6
  //   Ixx/Izz += sum(m_i * y_i^2)  (气囊X/Z偏移约0, 忽略微小贡献)
  //   Iyy 无贡献 (质点在Y轴上, dx=dz约0)
  // 质量计入Inertial后, Gazebo自动施加气囊重力(作用在base_link质心),
  // 与V2"气囊重量作用在CoM"等效且更精确.
  // 用 lastSetMass 阈值控制更新频率(0.2kg), 避免每帧SetComponentData.
  double newMass = this->dataPtr->baseMass + totalBallastMass;
  if (std::abs(newMass - this->dataPtr->lastSetMass) > 0.2) {
    const double ballastY[4] = {3.3, 8.6, -3.3, -8.6};
    double ballastIxx = 0.0;
    double ballastIzz = 0.0;
    {
      std::lock_guard<std::mutex> lock(this->dataPtr->mtx);
      for (int i = 0; i < 4; i++) {
        double y2 = ballastY[i] * ballastY[i];
        ballastIxx += this->dataPtr->ballastMass[i] * y2;
        ballastIzz += this->dataPtr->ballastMass[i] * y2;
      }
    }

    auto inertialComp = _ecm.Component<components::Inertial>(this->dataPtr->linkEntity);
    if (inertialComp) {
      gz::math::Inertiald inertial = inertialComp->Data();
      gz::math::MassMatrix3d mm = inertial.MassMatrix();
      mm.SetMass(newMass);
      mm.SetIxx(this->dataPtr->baseIxx + ballastIxx);
      mm.SetIyy(this->dataPtr->baseIyy);
      mm.SetIzz(this->dataPtr->baseIzz + ballastIzz);
      inertial.SetMassMatrix(mm);
      _ecm.SetComponentData<components::Inertial>(this->dataPtr->linkEntity, inertial);
      this->dataPtr->lastSetMass = newMass;
      gzmsg << "[AirshipDynamics] dynamic inertial: m=" << newMass
            << " ixx=" << (this->dataPtr->baseIxx + ballastIxx)
            << " iyy=" << this->dataPtr->baseIyy
            << " izz=" << (this->dataPtr->baseIzz + ballastIzz) << std::endl;
    }
  }

  // === 2. Added Mass Forces and Moments ===
  // Using Kirchhoff equations
  // F_am = -(M11*a_lin + M12*a_ang) - omega x (M11*v_lin + M12*omega)
  // M_am = -(M21*a_lin + M22*a_ang) - (v_lin x (M11*v_lin + M12*omega) + omega x (M21*v_lin + M22*omega))
  // Since we don't have acceleration directly, we use the velocity-only terms (Munk moment)
  math::Vector3d munkLin = this->dataPtr->m11 * linVel + this->dataPtr->m12 * angVel;
  math::Vector3d munkAng = this->dataPtr->m21 * linVel + this->dataPtr->m22 * angVel;

  math::Vector3d addedMassForce = -angVel.Cross(munkLin);
  math::Vector3d addedMassMoment = -linVel.Cross(munkLin) - angVel.Cross(munkAng);

  // === 3. Viscous Forces (Hull) ===
  math::Vector3d velEpsV = LocalVelocity(linVel, angVel,
      math::Vector3d(this->dataPtr->distCOV - this->dataPtr->epsV, 0, 0));
  velEpsV = velEpsV - windBody;

  double q0EpsV = DynamicPressure(velEpsV);
  double gammaEpsV = 0.0;
  if (std::abs(velEpsV.X()) > 1e-4)
  {
    gammaEpsV = std::atan2(
        std::sqrt(velEpsV.Y() * velEpsV.Y() + velEpsV.Z() * velEpsV.Z()),
        velEpsV.X());
  }

  double forceViscMag = q0EpsV *
      (-this->dataPtr->forceInviscidCoeff * std::sin(2.0 * gammaEpsV) +
        this->dataPtr->forceViscousCoeff * std::sin(gammaEpsV) * std::sin(gammaEpsV));
  double momentViscMag = q0EpsV *
      (-this->dataPtr->momentInviscidCoeff * std::sin(2.0 * gammaEpsV) +
        this->dataPtr->momentViscousCoeff * std::sin(gammaEpsV) * std::sin(gammaEpsV));

  double viscNormalMag = std::sqrt(velEpsV.Y() * velEpsV.Y() + velEpsV.Z() * velEpsV.Z());
  double viscNY = 0.0, viscNZ = 0.0;
  if (viscNormalMag > 1e-6)
  {
    viscNY = velEpsV.Y() / viscNormalMag;
    viscNZ = velEpsV.Z() / viscNormalMag;
  }

  math::Vector3d forceVisc = -forceViscMag * math::Vector3d(0, viscNY, viscNZ);
  math::Vector3d momentVisc = momentViscMag * math::Vector3d(0, viscNZ, -viscNY);

  // === 4. Axial Drag ===
  // V2修复: 原实现只沿X轴(cos^2(AoA)仅保留轴向分量), 垂直方向(Z)完全无阻力,
  // 导致飞艇垂直速度无约束(实测无推力也能高速上升, 快速运动失控).
  // 改为沿速度反方向三轴分解, 垂直运动同样受阻力.
  math::Vector3d airspeedLin = linVel - windBody;
  double q0 = DynamicPressure(airspeedLin);
  double dragMag = q0 * this->dataPtr->axialDragCoeff;
  double airspeedMag = airspeedLin.Length();
  math::Vector3d forceAxialDrag(0, 0, 0);

  if (airspeedMag > 1e-6) {
    forceAxialDrag = -dragMag * airspeedLin / airspeedMag;
  }

  // === 5. Tether Force ===
  math::Vector3d tetherForceWorld{0, 0, 0};
  if (this->dataPtr->tetherEnabled)
  {
    math::Vector3d posWorld = pose->Pos();
    math::Vector3d displacement = posWorld - this->dataPtr->tetherAnchor;
    tetherForceWorld = -this->dataPtr->tetherStiffness * displacement
                       - this->dataPtr->tetherDamping * worldLinVel;
  }

  // === 6. Rotational Damping ===
  // 飞艇大表面积旋转时推动空气产生阻尼力矩: M_damp = -C_rot * omega
  // 这是物理上必须的, 否则飞艇旋转几乎无阻力
  math::Vector3d rotDampingMoment(
      -this->dataPtr->rotDampingX * angVel.X(),
      -this->dataPtr->rotDampingY * angVel.Y(),
      -this->dataPtr->rotDampingZ * angVel.Z());

  // === 6.5 Roll Moment (已移除) ===
  // 新方案: 四气囊取消横滚调节, 全部同步充放气用于调节高度
  // 横滚不通过气囊控制 (见 AirshipDynamics 上方 1.5 节: 气囊总质量 -> 垂直浮力)

  // === 7. Total Forces and Moments ===
  // 策略:
  //   - 浮力在浮力中心(buoyancyCenter)施加, 产生恢复力矩(摆锤效应)
  //   - 其他力(附加质量、粘性、系绳)在质心(CoM)施加
  // AddWorldWrench在link原点施力, Gazebo计算相对CoM的力矩: (-comOffset) x F
  // 要让力等效在P点施加, 补偿力矩 = comOffset x F + (P - comOffset) x F = P x F
  //   即: 补偿力矩 = P x F, 其中P是力的作用点(相对link原点)

  // 非浮力: 在CoM施加, 补偿力矩 = comOffset x F
  // P3c修改: 气囊重量不再手动施加(见第1.5节), 由Gazebo通过动态Inertial质量
  // 自动施加重力(作用在base_link质心, 相对CoM力矩为0, 只贡献纯垂直力,
  // 不产生倾斜力矩, 不削弱摆锤被动稳定). 此处移除ballastWeightBody避免双重计重.
  math::Vector3d nonBuoyancyForce = addedMassForce + forceVisc + forceAxialDrag
                                    + pose->Rot().RotateVectorReverse(tetherForceWorld);
  math::Vector3d nonBuoyancyMoment = addedMassMoment + momentVisc + rotDampingMoment;

  // 浮力: 在buoyancyCenter施加, 产生恢复力矩
  // 补偿力矩 = buoyancyCenter x buoyancyBody
  // 这样Gazebo计算的力矩 = (-comOffset x buoyancyBody) + (buoyancyCenter x buoyancyBody)
  //                      = (buoyancyCenter - comOffset) x buoyancyBody
  // 这就是浮力中心相对质心的偏移产生的恢复力矩(摆锤效应)
  math::Vector3d buoyancyCompensatingMoment = this->dataPtr->buoyancyCenter.Cross(buoyancyBody);

  // 非浮力补偿: 在CoM施加
  math::Vector3d comOffset = this->dataPtr->buoyancyOffset;
  math::Vector3d nonBuoyancyCompensatingMoment = comOffset.Cross(nonBuoyancyForce);

  // 总力和总力矩
  math::Vector3d totalForceBody = buoyancyBody + nonBuoyancyForce;
  math::Vector3d totalMomentBody = nonBuoyancyMoment + buoyancyCompensatingMoment
                                   + nonBuoyancyCompensatingMoment;

  // TEMP-DEBUG: 打印各分力+真实位置 (诊断: 飞艇快速运动/净浮力/垂直阻力, 验证后删除)
  static int ad_dbg_cnt = 0;

  if (++ad_dbg_cnt % 10 == 1) {
    static FILE *adf = fopen("/tmp/adsim_dbg.log", "a");
    if (adf) {
      fprintf(adf,
          "pos_z=%.2f buoy=%.2f ballast_kg=%.2f amf=%.2f viscF=%.2f axial=%.2f"
          " totalF=%.2f vz=%.2f vx=%.2f vy=%.2f\n",
          pose->Z(), buoyancyBody.Z(), totalBallastMass, addedMassForce.Z(),
          forceVisc.Z(), forceAxialDrag.Z(), totalForceBody.Z(),
          linVel.Z(), linVel.X(), linVel.Y());
      fflush(adf);
    }
  }

  baseLink.AddWorldWrench(_ecm,
      pose->Rot() * totalForceBody,
      pose->Rot() * totalMomentBody);
}

GZ_ADD_PLUGIN(
    AirshipDynamics,
    System,
    AirshipDynamics::ISystemConfigure,
    AirshipDynamics::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    AirshipDynamics,
    "gz::sim::systems::AirshipDynamics")
