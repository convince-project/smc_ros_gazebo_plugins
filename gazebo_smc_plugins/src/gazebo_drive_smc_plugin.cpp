/*
 * Copyright (c) 2025 - for information on the respective copyright owner
 * see the NOTICE file
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <smc_verifiable_plugins/smc_plugin_base.hpp>
#include <smc_verifiable_plugins/utils.hpp>
#include <sim_handling_interfaces/gazebo_node_instances_holder.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <memory>
#include <utility>
#include <stdexcept>
#include <cstdlib>

using smc_verifiable_plugins::DataExchange;

namespace gazebo_smc_plugins
{
/*!
 * @brief A plugin that instantiates a new Gazebo simulation on reset and steps it forward on demand.
 */
class GazeboDriveSmcPlugin : public smc_verifiable_plugins::SmcPluginBase {
  public:
    GazeboDriveSmcPlugin() {
    }

    ~GazeboDriveSmcPlugin() {
        if (_node_ptr) {
            resetROS();
        }
    }

    std::string getPluginName() const override {
        return "gazebo_drive_smc_plugin";
    }
    
    private:
    inline void throwError(const std::string& msg) {
        throw std::runtime_error("Plugin [" + getPluginName() + "]: " + std::to_string(_thread_id) + ": " + msg);
    }
    
    void resetROS() {
        _cmd_vel_pub.reset();
        _executor_ptr->remove_node(_node_ptr);
        _node_ptr.reset();
        _executor_ptr.reset();
    }

    void processInitParameters([[maybe_unused]] const DataExchange&) override {
        _thread_id = gettid();
        auto& gz_holder = GazeboNodeInstancesHolder::get_holder();
        gz_holder.assign_partition(_thread_id);
    }

    std::optional<DataExchange> processReset() override {
        if (!_node_ptr) {
            auto& gz_holder = GazeboNodeInstancesHolder::get_holder();
            const std::string partition = gz_holder.getGzPartition(_thread_id);
            _node_ptr = std::make_shared<rclcpp::Node>(getPluginName(), partition);
            _cmd_vel_pub = _node_ptr->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1u);
            _executor_ptr = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
            _executor_ptr->add_node(_node_ptr);
        }
        _executor_ptr->spin_all(1s);
        return DataExchange();
    }

    std::optional<DataExchange> processInputParameters(const DataExchange& input) override {
        const auto lin_vel_it = input.find("lin_vel");
        if (lin_vel_it == input.end()) {
            throwError("expected input param 'lin_vel'.");
        }
        const double lin_vel = std::get<double>(lin_vel_it->second);
        const auto ang_vel_it = input.find("ang_vel");
        if (ang_vel_it == input.end()) {
            throwError("expected input param 'ang_vel'.");
        }
        const double ang_vel = std::get<double>(ang_vel_it->second);
        geometry_msgs::msg::Twist out_msg;
        out_msg.linear.x = lin_vel;
        out_msg.linear.y = 0.0;
        out_msg.linear.z = 0.0;
        out_msg.angular.x = 0.0;
        out_msg.angular.y = 0.0;
        out_msg.angular.z = ang_vel;
        _cmd_vel_pub->publish(out_msg);
        _executor_ptr->spin_all(1s);
        return DataExchange();
    }

    // ==== Operation variables ====
    int _thread_id;
    rclcpp::Node::SharedPtr _node_ptr;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_pub;
    rclcpp::executors::SingleThreadedExecutor::UniquePtr _executor_ptr;
};
} // namespace gazebo_smc_plugins

namespace smc_verifiable_plugins {
GENERATE_PLUGIN_LOADER(gazebo_smc_plugins::GazeboDriveSmcPlugin);
}  // namespace smc_verifiable_plugins