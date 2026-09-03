#ifndef CUSTOM_MOTOR_MODEL_HH_
#define CUSTOM_MOTOR_MODEL_HH_

#include <gz/sim/System.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/transport/Node.hh>
#include <gz/math/Vector3.hh>

#include <memory>
#include <mutex>
#include <string>

namespace custom
{
/// \brief 定制电机模型: 纯施力实现(不驱动joint), 推力方向由SDF参数指定.
///
/// 背景: gz官方MulticopterMotorModel在rotor link带大角度姿态旋转时
/// (lingyun01推进电机link绕Y转90度使推力水平), 电机旋转会触发ODE数值
/// 爆炸(虚假横滚力矩把飞艇掀翻, IMU角速度瞬间冲到100+度/s) — 详见
/// docs/lingyun/问题记录/airship_takeoff_control_failure_v3.md.
/// 本插件彻底绕开rotor link姿态依赖:
///   - 推力方向: SDF参数<thrustDirection>x y z</thrustDirection>(机体系),
///     用base_link世界姿态旋转, link无需旋转
///   - 施力方式: 对base_link直接AddWorldWrench(力+P×F补偿力矩),
///     不驱动joint, 不依赖rotor link姿态
///   - 转速命令: 订阅command/motor_speed(Actuators), 与官方插件一致,
///     thrust = motorConstant × vel × |vel| (支持负值反转差动)
///   - 一阶滤波: timeConstantUp/Down平滑转速响应(与官方一致)
///   - 反扭矩: momentConstant × thrust 绕推力轴(量级微小, 保留物理完整性)
///
/// SDF用法(推进电机示例, link pose无需旋转):
///   <plugin filename=".../libCustomMotorModel.so" name="custom::CustomMotorModel">
///     <link_name>base_link</link_name>          施力目标(机体)
///     <motorNumber>6</motorNumber>              Actuators数组索引
///     <thrustDirection>1 0 0</thrustDirection>  推力方向(机体系, 单位向量)
///     <motorPosition>2.23 5.888 -2.378</motorPosition> 施力点(机体系)
///     <motorConstant>8.677e-03</motorConstant>  推力系数 N/(rad/s)^2
///     <momentConstant>4.3e-4</momentConstant>   反扭矩系数
///     <maxRotVelocity>394.8</maxRotVelocity>    最大转速rad/s
///     <timeConstantUp>0.15</timeConstantUp>
///     <timeConstantDown>0.2</timeConstantDown>
///   </plugin>
class CustomMotorModel :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  CustomMotorModel();
  ~CustomMotorModel() override = default;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

private:
  class Impl;
  std::unique_ptr<Impl> dataPtr;
};

}  // namespace custom

#endif  // CUSTOM_MOTOR_MODEL_HH_
