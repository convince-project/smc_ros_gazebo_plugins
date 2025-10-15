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
#include <stdexcept>
#include <Eigen/Geometry>

using smc_verifiable_plugins::DataExchange;

namespace gazebo_smc_plugins
{
/*!
 * @brief A plugin that instantiates a new Gazebo simulation on reset and steps it forward on demand.
 */
class VelToGoalSmcPlugin : public smc_verifiable_plugins::SmcPluginBase {
  public:
    VelToGoalSmcPlugin() {
    }

    ~VelToGoalSmcPlugin() {}

    std::string getPluginName() const override {
        return "vel_to_goal_smc_plugin";
    }
    
    private:
    inline void throwError(const std::string& msg) {
        throw std::runtime_error("Plugin [" + getPluginName() + "]: " + msg);
    }
    

    void processInitParameters([[maybe_unused]] const DataExchange& conf) override {
        _max_lin_vel = std::get<double>(conf.at("max_lin_vel"));
        _max_ang_vel = std::get<double>(conf.at("max_ang_vel"));
    }

    std::optional<DataExchange> processReset() override {
        return DataExchange({
            {"lin_vel", 0.0},
            {"ang_vel", 0.0}
        });
    }

    std::optional<DataExchange> processInputParameters(const DataExchange& input) override {
        const double robot_x = std::get<double>(input.at("robot_pose_x"));
        const double robot_y = std::get<double>(input.at("robot_pose_y"));
        const double robot_theta = std::get<double>(input.at("robot_pose_theta"));
        const double goal_x = std::get<double>(input.at("goal_pose_x"));
        const double goal_y = std::get<double>(input.at("goal_pose_y"));
        Eigen::Isometry2d trf_world_robot = Eigen::Isometry2d::Identity();
        trf_world_robot.translate(Eigen::Vector2d(robot_x, robot_y));
        trf_world_robot.rotate(Eigen::Rotation2Dd(robot_theta));
        const Eigen::Vector2d p_world = Eigen::Vector2d(goal_x, goal_y);
        const Eigen::Vector2d p_robot = trf_world_robot.inverse() * p_world;
        const double lin_vel = std::min(_max_lin_vel, std::max(0.0, p_robot.x()));
        const double angle_displacement = atan2(p_robot.y(), p_robot.x());
        const double ang_abs_vel = std::min(_max_ang_vel, std::abs(angle_displacement));
        return DataExchange({
            {"lin_vel", lin_vel},
            {"ang_vel", copysign(ang_abs_vel, angle_displacement)}
        });
    }

    double _max_lin_vel = 0.0;
    double _max_ang_vel = 0.0;
};
} // namespace gazebo_smc_plugins

namespace smc_verifiable_plugins {
GENERATE_PLUGIN_LOADER(gazebo_smc_plugins::VelToGoalSmcPlugin);
}  // namespace smc_verifiable_plugins