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

#pragma once
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <sim_handling_interfaces/srv/start_gazebo.hpp>
#include <sim_handling_interfaces/srv/stop_gazebo.hpp>

using namespace std::chrono_literals;

class GazeboNodeInstancesHolder {
    public:
        // Get the singleton instance
        static GazeboNodeInstancesHolder& get_holder() {
            static GazeboNodeInstancesHolder instance;
            return instance;
        }

        /*!
         * @brief Assign a partition ID to threadm if this was not already done
         * @param thread_id The thread ID to associate to the partition ID
         * @return The partition ID
         */
        [[maybe_unused]] const std::string& assign_partition(const int thread_id) {
            std::lock_guard lock(_mtx);
            if (!_gz_thread_to_partition.contains(thread_id)) {
                const std::string new_partition_name = "robot" + std::to_string(_gz_thread_to_partition.size());
                _gz_thread_to_partition.emplace(thread_id, new_partition_name);
            }
            return _gz_thread_to_partition.at(thread_id);
        }

        // Create an object for a specific ID if it doesn't exist
        void start_new_sim(const int thread_id) {
            std::lock_guard lock(_mtx);
            const std::string& gz_partition = _gz_thread_to_partition.at(thread_id);
            auto req_ptr = std::make_shared<sim_handling_interfaces::srv::StartGazebo::Request>();
            req_ptr->gz_partition = gz_partition;
            for (size_t attempt = 0u; attempt < 3U; attempt++) {
                auto future_srv_start = _start_client_ptr->async_send_request(req_ptr);
                if (rclcpp::spin_until_future_complete(_node_ptr, future_srv_start, 15s) == rclcpp::FutureReturnCode::SUCCESS) {
                    const auto gz_params = *future_srv_start.get();
                    if (gz_params.success) {
                        _start_client_ptr->prune_pending_requests();
                        return;
                    }
                    std::cerr << "Error: Gazebo simulation start service returned fail on partition " << gz_partition << std::endl << std::flush;
                    continue;
                }
                std::cerr << "Error: Gazebo simulation start service timed out on partition " << gz_partition << std::endl << std::flush;
            }
        throw std::runtime_error("Error: cannot start the Gazebo simulation on partition " + gz_partition);
        }

        void stop_sim(const int thread_id) {
            std::lock_guard lock(_mtx);
            // Assuming this always exists: if not, it is expected to kill the process!
            const std::string& gz_partition = _gz_thread_to_partition.at(thread_id);
            auto req_ptr = std::make_shared<sim_handling_interfaces::srv::StopGazebo::Request>();
            req_ptr->gz_partition = gz_partition;
            for (size_t attempt = 0u; attempt < 3U; attempt++) {
                auto stop_res = _stop_client_ptr->async_send_request(req_ptr);
                if (rclcpp::spin_until_future_complete(_node_ptr, stop_res, 15s) == rclcpp::FutureReturnCode::SUCCESS) {
                    const bool success = stop_res.get()->success;
                    if (!success) {
                        throw std::runtime_error("Unexpected result from service stop request.");
                    }
                    _stop_client_ptr->prune_pending_requests();
                    return;
                }
                std::cerr << "Error: Gazebo simulation stop service req. timed out on partition " << gz_partition << std::endl << std::flush;
            }
            throw std::runtime_error("Error: cannot stop the Gazebo simulation on partition " + gz_partition);
        }

        const std::string& getGzPartition(const int thread_id) const {
            return _gz_thread_to_partition.at(thread_id);
        }

        bool has_partition(const int thread_id) const {
            return _gz_thread_to_partition.contains(thread_id);
        }

    private:
        GazeboNodeInstancesHolder() {
            rclcpp::init(0, nullptr);
            _node_ptr = std::make_shared<rclcpp::Node>("gazebo_plugin_controller");
            _start_client_ptr = _node_ptr->create_client<sim_handling_interfaces::srv::StartGazebo>("/start_simulation");
            _stop_client_ptr = _node_ptr->create_client<sim_handling_interfaces::srv::StopGazebo>("/stop_simulation");
            if (!_start_client_ptr->wait_for_service(5s)) {
                throw std::runtime_error("No response from service '/start_simulation'");
            }
            if (!_stop_client_ptr->wait_for_service(5s)) {
                throw std::runtime_error("No response from service '/stop_simulation'");
            }
        }

        ~GazeboNodeInstancesHolder() {
            _start_client_ptr.reset();
            _stop_client_ptr.reset();
            _node_ptr.reset();
            rclcpp::shutdown();
        }

        GazeboNodeInstancesHolder(const GazeboNodeInstancesHolder&) = delete;
        GazeboNodeInstancesHolder& operator=(const GazeboNodeInstancesHolder&) = delete;
        
        rclcpp::Node::SharedPtr _node_ptr;
        rclcpp::Client<sim_handling_interfaces::srv::StartGazebo>::SharedPtr _start_client_ptr;
        rclcpp::Client<sim_handling_interfaces::srv::StopGazebo>::SharedPtr _stop_client_ptr;
        std::map<int, std::string> _gz_thread_to_partition;
        // A mutex to ensure we do not create / delete instances at once
        std::mutex _mtx;
};