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
#include <ros_gz_interfaces/srv/control_world.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2/utils.hpp>

#include <smc_verifiable_plugins/smc_plugin_base.hpp>
#include <smc_verifiable_plugins/utils.hpp>
#include <sim_handling_interfaces/gazebo_node_instances_holder.hpp>
#include <memory>
#include <utility>
#include <stdexcept>
#include <cstdlib>

using smc_verifiable_plugins::DataExchange;

namespace gazebo_smc_plugins
{

inline bool sameTime(const double& t1, const double& t2) {
    return std::abs(t1 - t2) < 1e-6;
}

inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    const auto tf_q = tf2::Quaternion(q.x, q.y, q.z, q.w);
    return tf2::getYaw(tf_q);
}

/*!
 * @brief A plugin that instantiates a new Gazebo simulation on reset and steps it forward on demand.
 */
class GazeboStepSmcPlugin : public smc_verifiable_plugins::SmcPluginBase {
  public:
    GazeboStepSmcPlugin() {
        _n_resets = 0u;
    }

    ~GazeboStepSmcPlugin() {
        if (_node_ptr) {
            auto& gz_holder = GazeboNodeInstancesHolder::get_holder();
            gz_holder.stop_sim(_thread_id);
            resetROS();
        }
    }
    
    std::string getPluginName() const override {
        return "gazebo_step_smc_plugin";
    }
    
    private:
    inline void throwError(const std::string& msg) {
        std::string partition = _node_ptr->get_namespace();
        throw std::runtime_error("Plugin [" + getPluginName() + "]: part. " + partition + " attempt " + std::to_string(_n_resets) + " at t. " + std::to_string(_sim_time) + ": " + msg);
    }

    inline void printError(const std::string& msg) {
        std::string partition = _node_ptr->get_namespace();
        std::cerr << "Plugin [" << getPluginName() << "]: part. " << partition << " attempt " << _n_resets << " at t. " << _sim_time << ": " << msg << std::endl << std::flush;
    }

    /*!
     * @brief Reset the ROS node and related subscribers.
     */
    void resetROS() {
        _poses_sub.reset();
        _step_client.reset();
        _executor_ptr->remove_node(_node_ptr);
        _executor_ptr.reset();
        _node_ptr.reset();
    }

    inline void generateNode() {
        auto& gz_holder = GazeboNodeInstancesHolder::get_holder();
        const std::string& partition = gz_holder.getGzPartition(_thread_id);
        _node_ptr = std::make_shared<rclcpp::Node>(getPluginName(), partition);
        _poses_sub = _node_ptr->create_subscription<geometry_msgs::msg::PoseArray>("robot_pose", 1u, std::bind(&GazeboStepSmcPlugin::posesMsgCb, this, std::placeholders::_1));
        _distances_sub = _node_ptr->create_subscription<geometry_msgs::msg::Vector3Stamped>("closest_points", 1u, std::bind(&GazeboStepSmcPlugin::closestObstablesCb, this, std::placeholders::_1));
        _step_client = _node_ptr->create_client<ros_gz_interfaces::srv::ControlWorld>("world/default/control");
        _executor_ptr = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        _executor_ptr->add_node(_node_ptr);
    }

    inline DataExchange generateJaniState() const {
        return DataExchange({
            {"sim_time", _sim_time},
            {"robot_pose_x", _robot_pose.position.x},
            {"robot_pose_y", _robot_pose.position.y},
            {"robot_pose_theta", yawFromQuaternion(_robot_pose.orientation)},
            {"distance_front_left", _distances.front_left},
            {"distance_front", _distances.front},
            {"distance_front_right", _distances.front_right}
        });
    }

    void processInitParameters(const DataExchange& config) override {
        _thread_id = gettid();
        auto& gz_holder = GazeboNodeInstancesHolder::get_holder();
        gz_holder.assign_partition(_thread_id);
        // This needs to match with the step_time in the world SDF file.
        const auto step_time_it = config.find("step_time");
        if (step_time_it == config.end()) {
            throwError("expected config param 'step_time'.");
        }
        _step_time = std::get<double>(step_time_it->second);
        if (_step_time > 0.01 + 1e-4) {
            throwError("configured 'step_time' " + std::to_string(_step_time) + " > 0.01");
        }
        const auto n_steps_it = config.find("n_steps");
        if (n_steps_it == config.end()) {
            _n_steps = 1;
        } else {
            _n_steps = std::get<int64_t>(n_steps_it->second);
        }
        _total_time_step = static_cast<double>(_n_steps) * _step_time;
    }

    std::optional<DataExchange> processReset() override {
        _n_resets++;
        auto& gz_holder = GazeboNodeInstancesHolder::get_holder();
        const bool first_run = (_node_ptr == nullptr);
        if (!first_run) {
            gz_holder.stop_sim(_thread_id);
            // resetROS();
        }
        _sim_time = -1.0;
        _model_time = -1.0;
        gz_holder.start_new_sim(_thread_id);
        if (first_run) {
            generateNode();
        }
        _executor_ptr->spin_all(UPDATE_TIMEOUT);
        if (!_step_client->wait_for_service(UPDATE_TIMEOUT * N_ATTEMPTS)) {
            printError("Failed latching to service.");
            return std::nullopt;
        }
        if (!waitUpdate(0.0)) {
            return std::nullopt;
        }
        return generateJaniState();
    }

    std::optional<DataExchange> processInputParameters(const DataExchange&) override {
        bool success = stepFwd();
        if (!success) {
            return std::nullopt;
        }
        const double expected_new_time = _sim_time + _total_time_step;
        success = waitUpdate(expected_new_time);
        if (!success) {
            return std::nullopt;
        }
        return generateJaniState();
    }

    /*!
     * @brief Request the simulation to step forward by a n. of time steps.
     * @return True if the request succeeded, false otherwise.
     */
    bool stepFwd() {
        auto req_ptr = std::make_shared<ros_gz_interfaces::srv::ControlWorld::Request>();
        req_ptr->world_control.pause = true;
        req_ptr->world_control.step = false;
        req_ptr->world_control.multi_step = _n_steps;
        bool executed = false;
        auto future_srv_res = _step_client->async_send_request(req_ptr);
        for (size_t attempt = 0u; !executed && attempt < N_ATTEMPTS; attempt++) {
            if (_executor_ptr->spin_until_future_complete(future_srv_res, UPDATE_TIMEOUT) == rclcpp::FutureReturnCode::SUCCESS) {
                const bool success = future_srv_res.get()->success;
                if (!success) {
                    throwError("Step service reported a failure.");
                }
                _step_client->prune_pending_requests();
                return true;
            }
            // printError("Step srv request failed. Retry n. " + std::to_string(attempt));
        }
        _step_client->prune_pending_requests();
        printError("Failed trace due to failed srv request.");
        return false;
    }

    /*!
     * @brief Wait for a specific time update.
     * @param expected_new_time The time update we are waiting for.
     * @return True if we could receive the required time, false if that wasn't possible.
     */
    bool waitUpdate(const double expected_new_time) {
        _expected_time = expected_new_time;
        for (size_t attempt = 0u; attempt < N_ATTEMPTS; attempt++) {
            _updates_promise_ptr = std::make_unique<std::promise<void>>();
            auto future = _updates_promise_ptr->get_future();
            const auto ret_code = _executor_ptr->spin_until_future_complete(future, UPDATE_TIMEOUT);
            _updates_promise_ptr.reset();
            if (ret_code == rclcpp::FutureReturnCode::SUCCESS) {
                _sim_time = _model_time;
                return true;
            }
            // printError("Did not receive complete updates yet. Retry n. " + std::to_string(attempt) + ". Time: " + std::to_string(_model_time));
        }
        printError("Failed trace due to no time update at " + std::to_string(_sim_time));
        return false;
    }

    void posesMsgCb(geometry_msgs::msg::PoseArray::UniquePtr msg) {
        if (_updates_promise_ptr) {
            _model_time = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
            _robot_pose = msg->poses.at(0u);
            updateMsgPromise();
        }
    }

    void closestObstablesCb(geometry_msgs::msg::Vector3Stamped::UniquePtr msg) {
        if (_updates_promise_ptr) {
            const double msg_time = static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
            _distances = {msg_time, msg->vector.x, msg->vector.y, msg->vector.z};
            updateMsgPromise();
        }
    }

    /*!
     * @brief Makes sure that the desired callbacks have been executed and the system is at the desired state, before setting the promise as satisfied
     */
    void updateMsgPromise() {
        if (sameTime(_expected_time, _model_time) && sameTime(_expected_time, _distances.time)) {
            _updates_promise_ptr->set_value();
        }
    }

    // ==== Operation variables ====
    const size_t N_ATTEMPTS = 5u;
    const std::chrono::seconds UPDATE_TIMEOUT = 1s;

    // ==== Operation variables ====
    int _thread_id;
    // ROS-Communication related
    rclcpp::Node::SharedPtr _node_ptr;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr _poses_sub;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr _distances_sub;
    rclcpp::Client<ros_gz_interfaces::srv::ControlWorld>::SharedPtr _step_client;
    rclcpp::executors::SingleThreadedExecutor::UniquePtr _executor_ptr;
    // Track the sim time from Gazebo
    double _expected_time = -1.0;
    double _sim_time = -1.0;
    double _model_time = -1.0;
    
    struct {
        double time = -1.0;
        double front_left = -1.0;
        double front = -1.0;
        double front_right = -1.0;
    } _distances;

    // Keep the robot pose
    geometry_msgs::msg::Pose _robot_pose;
    // Shared promise, for spinning the node as long as needed
    std::unique_ptr<std::promise<void>> _updates_promise_ptr;
    // ==== Plugin-related ====
    // Config the Gazebo (how much time forward from a single step)
    double _step_time;
    // Config for Gazebo (how many steps to execute each time we advance sim)
    int64_t _n_steps;
    // Time step
    double _total_time_step;
    // Better error messages
    uint64_t _n_resets = 0u;
};
} // namespace gazebo_smc_plugins

namespace smc_verifiable_plugins {
GENERATE_PLUGIN_LOADER(gazebo_smc_plugins::GazeboStepSmcPlugin);
}  // namespace smc_verifiable_plugins