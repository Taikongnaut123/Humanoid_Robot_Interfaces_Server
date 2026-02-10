/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Navigation Module - 导航模块
 * 处理机器人的导航相关命令，具体包括：
 *  获取机器人当前位姿
 *  获取2D栅格地图
 *  开始导航
 *  取消当前导航任务
 *  获取剩余路径距离
 *  开始充电
 *  结束充电
 */

#ifndef SDK_SERVER_NAVIGATION_MODULE_H
#define SDK_SERVER_NAVIGATION_MODULE_H

#include "modules/module_base.h"

namespace humanoid_robot {
namespace server {
namespace modules {
class NavigationModule : public ModuleBase {
 public:
  NavigationModule();
  virtual ~NavigationModule();
  // bool IsRunning() override;

 protected:
  bool Initialize() override;
  void Cleanup() override;

  ModuleResult ProcessCommand(
      grpc::ServerContext* context, const int32_t& command_id,
      const humanoid_robot::PB::common::Dictionary& input_data,
      const humanoid_robot::PB::common::Dictionary& params) override;

 private:
  // Private implementation details
  class NavigationImpl;
  std::unique_ptr<NavigationImpl> pImpl_;
  NavigationModule(const NavigationModule&) = delete;
  NavigationModule& operator=(const NavigationModule&) = delete;
};
}  // namespace modules
}  // namespace server
}  // namespace humanoid_robot
#endif  // HUMANOID_ROBOT_NAVIGATION_MODULE_H