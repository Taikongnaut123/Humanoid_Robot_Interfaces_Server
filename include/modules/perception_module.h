/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Perception Module - 感知模块
 * 处理视觉、听觉、触觉等感知相关命令
 */

#ifndef SDK_SERVER_PERCEPTION_MODULE_H
#define SDK_SERVER_PERCEPTION_MODULE_H

#include "modules/module_base.h"

namespace humanoid_robot {
namespace server {
namespace modules {

/**
 * 感知模块
 * 支持的命令：
 * - vision_detect: 视觉检测
 * - audio_analyze: 音频分析
 * - tactile_sense: 触觉感知
 * - environment_scan: 环境扫描
 * - object_recognition: 物体识别
 */
class PerceptionModule : public ModuleBase {
public:
  PerceptionModule();
  // virtual ~PerceptionModule() = default; // 需要在.cpp中定义，因为使用了PIMPL
  virtual ~PerceptionModule(); // 需要在.cpp中定义，因为使用了PIMPL

protected:
  bool Initialize() override;
  // bool IsRunning() override;
  void Cleanup() override;

  ModuleResult
  ProcessCommand(grpc::ServerContext *context, const int32_t &command_id,
                 const humanoid_robot::PB::common::Dictionary &input_data,
                 const humanoid_robot::PB::common::Dictionary &params) override;

private:
  // 视觉检测
  ModuleResult Perception(
      // grpc::ServerContext *&context,
      const humanoid_robot::PB::common::Dictionary &input_data,
      const humanoid_robot::PB::common::Dictionary &params);

  // 音频分析
  ModuleResult Detection(
      // grpc::ServerContext *&context,
      const humanoid_robot::PB::common::Dictionary &input_data,
      const humanoid_robot::PB::common::Dictionary &params);

  // 触觉感知
  ModuleResult Division(
      // grpc::ServerContext *&context,
      const humanoid_robot::PB::common::Dictionary &input_data,
      const humanoid_robot::PB::common::Dictionary &params);
  // 模拟感知处理延时
  void SimulateProcessingDelay(int min_ms = 100, int max_ms = 500) const;

private:
  // Private implementation details
  class PerceptionImpl;
  std::unique_ptr<PerceptionImpl> pImpl_;
  // Disable copy and assignment
  PerceptionModule(const PerceptionModule &) = delete;
  PerceptionModule &operator=(const PerceptionModule &) = delete;
};

} // namespace modules
} // namespace server
} // namespace humanoid_robot

#endif // HUMANOID_ROBOT_PERCEPTION_MODULE_H
