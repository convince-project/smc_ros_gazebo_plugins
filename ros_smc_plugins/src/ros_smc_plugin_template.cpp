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

#include <rclcpp/rclcpp.hpp>
// Add messagges and other includes as needed
#include <tf2/utils.hpp>

#include <smc_verifiable_plugins/smc_plugin_base.hpp>
#include <smc_verifiable_plugins/utils.hpp>
#include <sim_handling_interfaces/gazebo_node_instances_holder.hpp>
#include <memory>
#include <utility>
#include <stdexcept>
#include <cstdlib>
#include <chrono>

using smc_verifiable_plugins::DataExchange;
using namespace std::chrono_literals;

namespace ros_smc_plugins
{
/*!
 * @brief Plugin skeleton: handles node lifecycle, subscriptions, and state reporting.
 */
class RosSmcPlugin : public smc_verifiable_plugins::SmcPluginBase {
  public:
    RosSmcPlugin() {
        _n_resets = 0u;
    }

    ~RosSmcPlugin() {
        if (_node_ptr) {
            auto& gz_holder = GazeboNodeInstancesHolder::get_holder();
            resetROS();
        }
    }
    
    std::string getPluginName() const override {
        return "ros_smc_plugin";
    }
    
  private:
    inline void throwError(const std::string& msg) {
        std::string partition = _node_ptr ? _node_ptr->get_namespace() : "";
        throw std::runtime_error("Plugin [" + getPluginName() + "]: part. " + partition + 
                                 " attempt " + std::to_string(_n_resets) + 
                                 " at t. " + std::to_string(_sim_time) + ": " + msg);
    }

    inline void printError(const std::string& msg) {
        std::string partition = _node_ptr ? _node_ptr->get_namespace() : "";
        std::cerr << "Plugin [" << getPluginName() << "]: part. " << partition 
                  << " attempt " << _n_resets << " at t. " << _sim_time 
                  << ": " << msg << std::endl << std::flush;
    }

    /*!
     * @brief Reset the ROS node and related subscribers.
     */
    void resetROS() {
        // Reset publishers, subscribers, clients, etc.
        if (_executor_ptr && _node_ptr) {
            _executor_ptr->remove_node(_node_ptr);
        }
        _executor_ptr.reset();
        _node_ptr.reset();
    }

    inline void generateNode() {
        auto& gz_holder = GazeboNodeInstancesHolder::get_holder();
        const std::string& partition = gz_holder.getGzPartition(_thread_id);
        _node_ptr = std::make_shared<rclcpp::Node>(getPluginName(), partition);

        // Put subscriptions, actions, and services definition here, i.e.
        // _sub = _node->create_subscription<MsgT>("topic", rclcpp::SensorDataQoS(),
        //     [this](MsgT::UniquePtr msg){ /* update local state */ });

        // Initialize executor
        _executor_ptr = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        _executor_ptr->add_node(_node_ptr);
    }

    inline DataExchange generateJaniState() const {
        return DataExchange({
            // Populate with relevant state variables
            // {"sim_time", _sim_time}
        });
    }

    void processInitParameters(const DataExchange& config) override {
        _thread_id = gettid();
        (void)config;
    }

    std::optional<DataExchange> processReset() override {
        _n_resets++;
        auto& gz_holder = GazeboNodeInstancesHolder::get_holder();
        const bool first_run = (_node_ptr == nullptr);
        _sim_time = -1.0;
        if (!first_run) {
          resetROS();
          
        }
        generateNode();
        _executor_ptr->spin_some();

        return generateJaniState();
    }

    std::optional<DataExchange> processInputParameters(const DataExchange&) override {
        // No stepping, just return current state snapshot
        return generateJaniState();
    }

    // ==== Operation variables ====
    int _thread_id;

    // Track the sim time from Gazebo
    double _sim_time = -1.0;
   
    rclcpp::Node::SharedPtr _node_ptr;
    rclcpp::executors::SingleThreadedExecutor::UniquePtr _executor_ptr;
    
    // ROS-Communication related
    // Example handles (leave commented in template)
    // rclcpp::Subscription<your_msg>::SharedPtr _sub;
    // rclcpp::Publisher<your_msg>::SharedPtr _pub;
    // rclcpp::Client<your_srv>::SharedPtr _client;
    // rclcpp_action::Client<your_action>::SharedPtr _action;

    // Better error messages
    uint64_t _n_resets = 0u;
};
} // namespace ros_smc_plugins

namespace smc_verifiable_plugins {
GENERATE_PLUGIN_LOADER(ros_smc_plugins::RosSmcPlugin);
}  // namespace smc_verifiable_plugins
