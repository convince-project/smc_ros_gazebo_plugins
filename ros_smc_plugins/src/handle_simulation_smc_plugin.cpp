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
#include <memory>
#include <utility>
#include <stdexcept>
#include <cstdlib>

using smc_verifiable_plugins::DataExchange;

namespace simulation_handler_smc_plugin
{
/*!
 * @brief A plugin that instantiates a new Gazebo simulation on reset and steps it forward on demand.
 */
class SimulationHandlerPlugin : public smc_verifiable_plugins::SmcPluginBase {
  public:
    SimulationHandlerPlugin() {}

    ~SimulationHandlerPlugin() {
        if (_simulation_running) {
            auto& simulation_holder = GazeboNodeInstancesHolder::get_holder();
            simulation_holder.stop_sim(_thread_id);
        }
        _simulation_running = false;
    };

    std::string getPluginName() const override {
        return "simulation_handler_smc_plugin";
    }
    
    private:
    inline void throwError(const std::string& msg) {
        throw std::runtime_error("Plugin [" + getPluginName() + "]: " + std::to_string(_thread_id) + ": " + msg);
    }

    void processInitParameters([[maybe_unused]] const DataExchange&) override {
        _thread_id = gettid();
        auto& simulation_holder = GazeboNodeInstancesHolder::get_holder();
        simulation_holder.assign_partition(_thread_id);
    }

    std::optional<DataExchange> processReset() override {
        _n_resets++;
        if (_simulation_running) {
            auto& simulation_holder = GazeboNodeInstancesHolder::get_holder();
            simulation_holder.stop_sim(_thread_id);
        }
        _simulation_running = false;
        return DataExchange();
    }

    std::optional<DataExchange> processInputParameters([[maybe_unused]] const DataExchange&) override {
        auto& simulation_holder = GazeboNodeInstancesHolder::get_holder();
        if (!_simulation_running) {
            simulation_holder.start_new_sim(_thread_id);
            _simulation_running = true;
        }
        return DataExchange();
    }

    // ==== Operation variables ====
    int _thread_id;
    int _n_resets = 0u;

    bool _simulation_running = false;
};
} // namespace gazebo_smc_plugins

namespace smc_verifiable_plugins {
GENERATE_PLUGIN_LOADER(simulation_handler_smc_plugin::SimulationHandlerPlugin);
}  // namespace smc_verifiable_plugins