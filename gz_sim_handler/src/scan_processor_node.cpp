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
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace gz_sim_handler
{
    class ScanProcessorNode : public rclcpp::Node {
    public:
        ScanProcessorNode(const rclcpp::NodeOptions & options)
        : Node("scan_processor_node", options)
        {
            using namespace std::placeholders;
            _scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", 10, std::bind(&ScanProcessorNode::scanCallback, this, _1));
            _vec_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
                "closest_points", 10);
        }

    private:
        void scanCallback(const sensor_msgs::msg::LaserScan::UniquePtr msg)
        {
            // Define three bins: left (-30 to -10 deg), center (-10 to +10 deg), right (+10 to +30 deg)
            struct Bin {
                double min_angle;
                double max_angle;
                double min_range;
                int min_index;
            };

            // Add overlapping bins, that can be used to prevent flickering during the recovery
            std::vector<Bin> bins = {
                { -70.0 * M_PI / 180.0, -20.0 * M_PI / 180.0, std::numeric_limits<double>::infinity(), -1 },
                { -40.0 * M_PI / 180.0,  40.0 * M_PI / 180.0, std::numeric_limits<double>::infinity(), -1 },
                {  20.0 * M_PI / 180.0,  70.0 * M_PI / 180.0, std::numeric_limits<double>::infinity(), -1 }
            };

            for (size_t i = 0; i < msg->ranges.size(); ++i) {
                double angle = msg->angle_min + i * msg->angle_increment;
                if (angle > M_PI) {
                    angle -= 2 * M_PI;
                }
                float range = msg->ranges[i];
                if (!std::isfinite(range)) continue;
                for (auto& bin : bins) {
                    if (angle >= bin.min_angle && angle < bin.max_angle) {
                        if (range < bin.min_range) {
                            bin.min_range = range;
                            bin.min_index = static_cast<int>(i);
                        }
                    }
                }
            }

            geometry_msgs::msg::Vector3Stamped out_msg;
            out_msg.header = msg->header;
            // x: left, y: center, z: right
            out_msg.vector.x = (bins[0].min_index >= 0) ? bins[0].min_range : msg->range_max;
            out_msg.vector.y = (bins[1].min_index >= 0) ? bins[1].min_range : msg->range_max;
            out_msg.vector.z = (bins[2].min_index >= 0) ? bins[2].min_range : msg->range_max;

            _vec_pub_->publish(out_msg);
        }

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scan_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr _vec_pub_;
    };
    
} // namespace gz_sim_handler

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(gz_sim_handler::ScanProcessorNode)