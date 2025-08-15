/**
 * Copyright (c) 2025 Humanoid Robot, Inc. All rights reserved.
 *
 * Module Base Implementation - 模块基类实现
 */

#include "modules/module_base.h"
#include <iostream>
#include <chrono>
#include <sstream>
#include <grpcpp/grpcpp.h>

namespace humanoid_robot
{
    namespace server
    {
        namespace modules
        {

            ModuleBase::ModuleBase(const std::string &module_name, size_t worker_threads)
                : module_name_(module_name), running_(false),
                  worker_thread_count_(worker_threads), task_id_counter_(0)
            {
            }

            ModuleBase::~ModuleBase()
            {
                Stop();
            }

            bool ModuleBase::Start()
            {
                if (running_.load())
                {
                    return true; // 已经运行中
                }

                std::cout << "[" << module_name_ << "] Starting module..." << std::endl;

                // 调用子类初始化
                if (!Initialize())
                {
                    std::cerr << "[" << module_name_ << "] Failed to initialize module" << std::endl;
                    return false;
                }

                running_.store(true);

                // 启动工作线程
                worker_threads_.reserve(worker_thread_count_);
                for (size_t i = 0; i < worker_thread_count_; ++i)
                {
                    worker_threads_.emplace_back(&ModuleBase::WorkerThread, this);
                }

                std::cout << "[" << module_name_ << "] Module started with "
                          << worker_thread_count_ << " worker threads" << std::endl;
                return true;
            }

            void ModuleBase::Stop()
            {
                if (!running_.load())
                {
                    return; // 已经停止
                }

                std::cout << "[" << module_name_ << "] Stopping module..." << std::endl;

                running_.store(false);

                // 唤醒所有工作线程
                queue_condition_.notify_all();

                // 等待所有工作线程结束
                for (auto &thread : worker_threads_)
                {
                    if (thread.joinable())
                    {
                        thread.join();
                    }
                }
                worker_threads_.clear();

                // 清理剩余任务
                std::lock_guard<std::mutex> lock(queue_mutex_);
                while (!task_queue_.empty())
                {
                    auto task = std::move(task_queue_.front());
                    task_queue_.pop();

                    // 设置任务被取消的结果
                    task->result_promise.set_value(
                        ModuleResult::Error(module_name_, task->command_id,
                                            -1, "Module stopped"));
                }

                // 调用子类清理
                Cleanup();

                std::cout << "[" << module_name_ << "] Module stopped" << std::endl;
            }

            std::future<ModuleResult> ModuleBase::ExecuteAsync(
                grpc::ServerContext *&context, const int32_t &command_id,
                humanoid_robot::PB::common::Dictionary &input_data,
                humanoid_robot::PB::common::Dictionary &params)
            {
                if (!running_.load())
                {
                    std::promise<ModuleResult> promise;
                    auto future = promise.get_future();
                    promise.set_value(ModuleResult::Error(module_name_, command_id,
                                                          -2, "Module not running"));
                    return future;
                }

                auto task = std::make_unique<ModuleTask>(command_id, GenerateTaskId());
                task->input_data = input_data;
                task->params = params;
                auto future = task->result_promise.get_future();

                // 添加任务到队列
                {
                    std::lock_guard<std::mutex> lock(queue_mutex_);
                    task_queue_.push(std::move(task));
                }
                queue_condition_.notify_one();

                return future;
            }

            ModuleResult ModuleBase::ExecuteSync(
                grpc::ServerContext *&context, const int32_t &command_id,
                humanoid_robot::PB::common::Dictionary &input_data,
                humanoid_robot::PB::common::Dictionary &params,
                int32_t timeout_ms)
            {
                auto future = ExecuteAsync(context, command_id, input_data, params);

                if (future.wait_for(std::chrono::milliseconds(timeout_ms)) == std::future_status::timeout)
                {
                    return ModuleResult::Error(module_name_, command_id, -3, "Command timeout");
                }

                return future.get();
            }

            std::unique_ptr<humanoid_robot::PB::common::Dictionary> ModuleBase::GetStatus() const
            {
                auto status = std::make_unique<humanoid_robot::PB::common::Dictionary>();
                auto kv_map = status->mutable_keyvaluelist();

                // 模块名称
                {
                    humanoid_robot::PB::common::Variant var;
                    var.set_stringvalue(module_name_);
                    kv_map->insert({"module_name", var});
                }

                // 运行状态
                {
                    humanoid_robot::PB::common::Variant var;
                    var.set_boolvalue(running_.load());
                    kv_map->insert({"running", var});
                }

                // 工作线程数
                {
                    humanoid_robot::PB::common::Variant var;
                    var.set_int32value(static_cast<int32_t>(worker_thread_count_));
                    kv_map->insert({"worker_threads", var});
                }

                // 队列中任务数
                {
                    std::lock_guard<std::mutex> lock(const_cast<std::mutex &>(queue_mutex_));
                    humanoid_robot::PB::common::Variant var;
                    var.set_int32value(static_cast<int32_t>(task_queue_.size()));
                    kv_map->insert({"pending_tasks", var});
                }

                return status;
            }

            void ModuleBase::WorkerThread()
            {
                std::cout << "[" << module_name_ << "] Worker thread started" << std::endl;

                while (running_.load())
                {
                    std::unique_ptr<ModuleTask> task;

                    // 从队列获取任务
                    {
                        std::unique_lock<std::mutex> lock(queue_mutex_);
                        queue_condition_.wait(lock, [this]
                                              { return !task_queue_.empty() || !running_.load(); });

                        if (!running_.load())
                        {
                            break;
                        }

                        if (!task_queue_.empty())
                        {
                            task = std::move(task_queue_.front());
                            task_queue_.pop();
                        }
                    }

                    if (task)
                    {
                        try
                        {
                            std::cout << "[" << module_name_ << "] Processing command: "
                                      << task->command_id << " (task: " << task->task_id << ")" << std::endl;
                            grpc::ServerContext context;
                            // 执行命令
                            ModuleResult result = ProcessCommand(&context, task->command_id, task->input_data, task->params);

                            // 设置结果
                            task->result_promise.set_value(std::move(result));
                        }
                        catch (const std::exception &e)
                        {
                            std::cerr << "[" << module_name_ << "] Exception in command "
                                      << task->command_id << ": " << e.what() << std::endl;

                            task->result_promise.set_value(
                                ModuleResult::Error(module_name_, task->command_id,
                                                    -4, "Exception: " + std::string(e.what())));
                        }
                        catch (...)
                        {
                            std::cerr << "[" << module_name_ << "] Unknown exception in command "
                                      << task->command_id << std::endl;

                            task->result_promise.set_value(
                                ModuleResult::Error(module_name_, task->command_id,
                                                    -5, "Unknown exception"));
                        }
                    }
                }

                std::cout << "[" << module_name_ << "] Worker thread stopped" << std::endl;
            }

            int32_t ModuleBase::GenerateTaskId()
            {
                return task_id_counter_.fetch_add(1);
            }

        } // namespace modules
    } // namespace server
} // namespace humanoid_robot
