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
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/bool.hpp>
#include <roamer_pkg/action/roamer.hpp>
#include <tf2/utils.hpp>
#include <smc_verifiable_plugins/smc_plugin_base.hpp>
#include <smc_verifiable_plugins/utils.hpp>
#include <sim_handling_interfaces/gazebo_node_instances_holder.hpp>
#include <memory>
#include <utility>
#include <stdexcept>
#include <cstdlib>
#include <chrono>
#include <random>
#include <optional>


using smc_verifiable_plugins::DataExchange;
using namespace std::chrono_literals;

namespace ros_smc_plugins
{

using RoamerAction = roamer_pkg::action::Roamer;
using GoalHandle   = rclcpp_action::ClientGoalHandle<RoamerAction>;

inline double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    const auto tf_q = tf2::Quaternion(q.x, q.y, q.z, q.w);
    return tf2::getYaw(tf_q);
}

class RosSmcPlugin : public smc_verifiable_plugins::SmcPluginBase {
  public:

    RosSmcPlugin() {
        _n_resets = 0u;
    }

    ~RosSmcPlugin() {
        if (_node_ptr) {
            resetROS();
        }
    }
    
    std::string getPluginName() const override {
        return "roamer_smc_plugin";
    }

    void setRandomSeed(const uint32_t seed) override {
        rng_.seed(seed);
    }
    
  private:

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
        // 1) Stop spinning and join the spinner thread if you have one.
        if (_executor_ptr) {
            _executor_ptr->cancel();       // break out of spin()
        }

        // 2) Detach node from executor.
        if (_executor_ptr && _node_ptr) {
            _executor_ptr->remove_node(_node_ptr);
        }

        // 3) Destroy comms (reverse order of creation).
        _roamer_client.reset();
        // ... reset pubs/subs/services/timers ...

        // 4) Destroy node, then executor.
        _node_ptr.reset();
        _executor_ptr.reset();

        // Optional but robust if you gave the node its own context:
        // if (_ctx) { _ctx->shutdown("reset"); _ctx.reset(); }
    }

    void resetState() {
        _robot_pose = geometry_msgs::msg::Pose();
        _bumped.data = false;
        _finished.data = false;
    }

    inline void generateNode() {
        auto& gz_holder = GazeboNodeInstancesHolder::get_holder();
        const std::string& partition = gz_holder.getGzPartition(_thread_id);
        _node_ptr = std::make_shared<rclcpp::Node>(getPluginName(), partition);

        // Put subscriptions, actions, and services definition here, i.e.
        // _sub = _node->create_subscription<MsgT>("topic", rclcpp::SensorDataQoS(),
        //     [this](MsgT::UniquePtr msg){ /* update local state */ });
        _roamer_client = rclcpp_action::create_client<RoamerAction>(_node_ptr, "roamer");

        // Initialize executor
        _executor_ptr = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
        _executor_ptr->add_node(_node_ptr);
        if (_executor_ptr) {
            _executor_ptr->spin_some();  // pump callbacks
        }
    }

    inline DataExchange generateJaniState() const {
        return DataExchange({
            // Populate with relevant state variables
            {"robot_pose_x", _robot_pose.position.x},
            {"robot_pose_y", _robot_pose.position.y},
            {"robot_pose_theta", yawFromQuaternion(_robot_pose.orientation)},
            {"bumped", _bumped.data}
        });
    }

    void processInitParameters([[maybe_unused]] const DataExchange&) override {
        _thread_id = gettid();
        auto& simulation_holder = GazeboNodeInstancesHolder::get_holder();
        simulation_holder.assign_partition(_thread_id);
    }

    std::optional<DataExchange> processReset() override {
        _n_resets++;
        const bool first_run = (_node_ptr == nullptr);
        _sim_time = -1.0;
        if (!first_run) {
            resetROS();
        }
        resetState();
        generateNode();
        _executor_ptr->spin_some();
        clear_roamer_goal_state();
        return generateJaniState();
    }

    std::optional<DataExchange> processInputParameters([[maybe_unused]] const DataExchange&) override {
        // Generate random motion
        constexpr double kPI = 3.14159265358979323846;
        double ang_min = -kPI;
        double ang_max = kPI;   // [-pi, pi]
        double d_min   = 0.5;
        double d_max   = 2.0;   // [0.5, 2.0]
        std::uniform_real_distribution<double> A(ang_min, ang_max);
        std::uniform_real_distribution<double> D(d_min,   d_max);
        const double angle = A(rng_);
        const double dist  = D(rng_);
        // Request the action
        if (!_roamer_client || !(_roamer_client->action_server_is_ready() || _roamer_client->wait_for_action_server(200ms))) {
            RCLCPP_ERROR(_node_ptr->get_logger(), "Roamer action server not ready!");
            return std::nullopt;
        }
        const bool goal_success = send_roamer_goal_async(angle, dist);
        if (!goal_success) {
            RCLCPP_ERROR(_node_ptr->get_logger(), "The action failed to execute!");
            return std::nullopt;
        }
        // Get result
        auto snap = get_roamer_snapshot();
        if (snap.has_result && !(snap.code == rclcpp_action::ResultCode::UNKNOWN)) {
            // Use the RESULT
            const auto& r = *snap.result;
            _robot_pose    = r.pose;
            _bumped.data   = r.bumped;
            _finished.data = r.finished;
            auto returning_value = generateJaniState();

            // Allow sending a new random goal next tick
            clear_roamer_goal_state();
            return returning_value;
        }
        // We should never reach this point
        RCLCPP_ERROR(_node_ptr->get_logger(), "The action succeeded, but the result isn't available!");
        return std::nullopt;
    }

    bool send_roamer_goal_async(double angle_rad, double distance_m)
    {
        _roamer_client->async_cancel_all_goals();
        RoamerAction::Goal g; g.angle_rad = angle_rad; g.distance_m = distance_m;

        rclcpp_action::Client<RoamerAction>::SendGoalOptions opts;

        // Goal response: void(GoalHandle::SharedPtr)
        opts.goal_response_callback = [this](GoalHandle::SharedPtr gh){
            std::lock_guard<std::mutex> lk(ag_mtx_);
            ag_sent_   = true;
            ag_handle_ = gh;             // nullptr => rejected
            // RCLCPP_INFO(_node_ptr->get_logger(),
            //   gh ? "Server ACCEPTED goal" : "Server REJECTED goal");
        };

        // Feedback: void(GoalHandle::SharedPtr, std::shared_ptr<const RoamerAction::Feedback>)
        opts.feedback_callback = [this](GoalHandle::SharedPtr /*gh*/,
            const std::shared_ptr<const RoamerAction::Feedback> fb){
            std::lock_guard<std::mutex> lk(ag_mtx_);
            ag_last_fb_ = *fb;
        };

        // Result: void(const GoalHandle::WrappedResult &)
        opts.result_callback = [this](const GoalHandle::WrappedResult & wr){
            std::lock_guard<std::mutex> lk(ag_mtx_);
            ag_code_ = wr.code;
            ag_last_result_.reset();
            if (wr.result) ag_last_result_ = *wr.result;
        };

        const auto per_attempt_timeout = 5s;     // wait this long for the accept/reject
        const auto overall_timeout     = 30s;
        typename GoalHandle::SharedPtr gh;  // <— declare before loop
       
        auto gh_future = _roamer_client->async_send_goal(g, opts);
        if (_executor_ptr->spin_until_future_complete(gh_future, per_attempt_timeout)
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_WARN(_node_ptr->get_logger(), "No accept/reject in time; retrying…");
            return false;
        }
        gh = gh_future.get();
        if (!gh) {
            RCLCPP_ERROR(_node_ptr->get_logger(), "Goal was rejected");
            return false;
        }

        // result
        auto res_future = _roamer_client->async_get_result(gh);
        if (_executor_ptr->spin_until_future_complete(res_future, overall_timeout)
            != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_ERROR(_node_ptr->get_logger(), "Result timed out");
            return false;
        }
        auto wrapped = res_future.get();
        return true;
    }

    void clear_roamer_goal_state([[maybe_unused]] bool cancel_active = false) {
        std::lock_guard<std::mutex> lk(ag_mtx_);
        ag_handle_.reset();                         // no active goal handle
        ag_sent_ = false;                           // <- makes snap.sent false next time
        ag_last_fb_.reset();                        // drop last feedback
        ag_last_result_.reset();                    // drop last result
        ag_code_ = rclcpp_action::ResultCode::UNKNOWN;
    }

    struct RoamerSnapshot {
        bool active = false;     // goal accepted and not finished
        bool sent   = false;     // a send was attempted
        bool has_result = false;
        rclcpp_action::ResultCode code{rclcpp_action::ResultCode::UNKNOWN};
        std::optional<RoamerAction::Feedback> last_fb;
        std::optional<RoamerAction::Result>   result;
    };

    RoamerSnapshot get_roamer_snapshot() {
        std::lock_guard<std::mutex> lk(ag_mtx_);
        RoamerSnapshot s;
        s.sent = ag_sent_;
        s.active = ag_handle_ && !ag_last_result_.has_value();
        s.has_result = ag_last_result_.has_value();
        s.code = ag_code_;
        s.last_fb = ag_last_fb_;
        s.result  = ag_last_result_;
        return s;
    }

    // ==== Operation variables ====
    int _thread_id;
    
    // Track the sim time from Gazebo
    double _sim_time = -1.0;
   
    rclcpp::Node::SharedPtr _node_ptr;
    rclcpp::executors::SingleThreadedExecutor::UniquePtr _executor_ptr;
    
    // ROS-Communication related
    // For action client handling
    std::mt19937 rng_;  // seed once
    mutable std::mutex ag_mtx_;
    GoalHandle::SharedPtr                         ag_handle_;        // non-null when accepted
    bool                                           ag_sent_{false};
    std::optional<RoamerAction::Feedback>         ag_last_fb_;
    std::optional<RoamerAction::Result>           ag_last_result_;
    rclcpp_action::ResultCode                      ag_code_{rclcpp_action::ResultCode::UNKNOWN};
    std::string _action_name;                       // e.g., "roamer"
    rclcpp_action::Client<RoamerAction>::SharedPtr _roamer_client;         // action client handle

    geometry_msgs::msg::Pose _robot_pose;
    std_msgs::msg::Bool _bumped;
    std_msgs::msg::Bool _finished;


    // Better error messages
    uint64_t _n_resets = 0u;
};
} // namespace ros_smc_plugins

namespace smc_verifiable_plugins {
GENERATE_PLUGIN_LOADER(ros_smc_plugins::RosSmcPlugin);
}  // namespace smc_verifiable_plugins
