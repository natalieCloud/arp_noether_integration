/**
 * @copyright Copyright (c) 2023, The Ohio State University
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


// ROS & Utilities
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.h>

// ROS Messages
// #include <andi_msgs/srv/generate_tool_paths.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

// Noether
#include <noether_tpp/core/types.h>
#include <noether_tpp/tool_path_planners/raster/direction_generators/fixed_direction_generator.h>
#include <noether_tpp/tool_path_planners/raster/direction_generators/principal_axis_direction_generator.h>

#include <noether_tpp/tool_path_planners/raster/origin_generators/aabb_origin_generator.h>
#include <noether_tpp/tool_path_planners/raster/origin_generators/centroid_origin_generator.h>
#include <noether_tpp/tool_path_planners/raster/origin_generators/fixed_origin_generator.h>
#include <noether_tpp/tool_path_planners/raster/origin_generators/offset_origin_generator.h>

#include <noether_tpp/tool_path_planners/raster/plane_slicer_raster_planner.h>

#include <noether_tpp/tool_path_modifiers/snake_organization_modifier.h>
#include <noether_tpp/tool_path_modifiers/raster_organization_modifier.h>

// pcl - Point Cloud Library
#include <pcl/conversions.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <pcl/Vertices.h>

#include <string>

/////////////////////////////////
// ROS 2 Node Parameter Retrieval
/////////////////////////////////
namespace
{
template <typename T>
T declareAndGet(rclcpp::Node* node, const std::string& key)
{
  T val;
  node->declare_parameter(key);
  if (!node->get_parameter(key, val))
  {
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  }
  return val;
}


///////////////////
// TYPE CONVERSIONS
///////////////////

geometry_msgs::msg::PoseArray toMsg(const noether::ToolPathSegment &segment)
{
    geometry_msgs::msg::PoseArray pose_array;
    pose_array.poses.reserve(segment.size());
    for (auto waypoint : segment)
    {
        // Renormalize orientation
        Eigen::Quaterniond q(waypoint.linear());
        q.normalize();
        waypoint.matrix().block<3, 3>(0, 0) = q.toRotationMatrix();

        pose_array.poses.push_back(tf2::toMsg(waypoint));
    }
    return pose_array;
}

// andi_msgs::msg::ToolPath toMsg(const noether::ToolPath &path)
// {
//     andi_msgs::msg::ToolPath path_msg;
//     path_msg.segments.reserve(path.size());
//     for (const auto &segment : path)
//         path_msg.segments.push_back(toMsg(segment));
//     return path_msg;
// }

// andi_msgs::msg::ToolPaths toMsg(const noether::ToolPaths &paths)
// {
//     andi_msgs::msg::ToolPaths paths_msg;
//     paths_msg.paths.reserve(paths.size());
//     for (const auto &path : paths)
//         paths_msg.paths.push_back(toMsg(path));
//     return paths_msg;
// }



}

namespace arp_tpp
{
class TPPNode : public rclcpp::Node
{
    public:
        TPPNode() : Node("arp_tpp_node"),
        line_spacing_(declareAndGet<double>(this, "line_spacing")),
        point_spacing_(declareAndGet<double>(this, "point_spacing")),
        min_segment_size_(declareAndGet<double>(this, "min_segment_size")),
        min_hole_size_(declareAndGet<double>(this, "min_hole_size")),
        search_radius_(declareAndGet<double>(this, "search_radius"))
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Tool path planning node is ready! :)");
            publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("topic", 10);
            pathPlanning();
        }

        void pathPlanning()
        {


            pcl::PolygonMesh pcl_mesh;

            std::string filename = "src/arp_tpp_noether/include/wavy_mesh_with_hole.ply";
            if (pcl::io::loadPolygonFile(filename, pcl_mesh) == 0)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error opening file! :(");
            }
            else
            {
                // Create a planner
            
                // PlaneSlicerRaster Planner Options:
                //  1 - std::make_unique<noether::PrincipalAxisDirectionGenerator>()
                //  1 - std::make_unique<noether::FixedDirectionGenerator>()
                //  2 - std::make_unique<noether::FixedOriginGenerator>()
                //  2 - std::make_unique<noether::CentroidOriginGenerator>()
                //  2 - std::make_unique<noether::AABBOriginGenerator>()
                noether::PlaneSlicerRasterPlanner planner(std::make_unique<noether::PrincipalAxisDirectionGenerator>(),
                                                          std::make_unique<noether::FixedOriginGenerator>());

                // Configure the planner
                planner.setLineSpacing(line_spacing_);
                planner.setMinHoleSize(min_hole_size_);
                planner.setMinSegmentSize(min_segment_size_);
                planner.setPointSpacing(point_spacing_);
                planner.setSearchRadius(search_radius_);

                // Create a modifier to organize the tool path in a snake pattern
                noether::SnakeOrganizationModifier mod;

                // Call the planner
                noether::ToolPaths paths = mod.modify(planner.plan(pcl_mesh));

                //geometry_msgs::msg::PoseArray pose_array = toMsg(paths);

                if (paths.empty())
                {
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TPP Server returning failed. Path generation failed :(");
                }

                for (const auto &path : paths) {
                    for (const auto &segment : path) {
                        geometry_msgs::msg::PoseArray poses = toMsg(segment);
                        poses.header.frame_id = "world"
                        RCLCPP_INFO(this->get_logger(), "Publishing Poses:\n");
                        for (const auto &pose : poses.poses) {
                            RCLCPP_INFO(this->get_logger(), "Pose: pos- '%f' '%f' '%f' ori- '%f' '%f' '%f' '%f' ", 
                            pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
                        } 
                        publisher_->publish(poses);
                    }
                }
            }

            // Report node is COMPLETE
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Called path planning node!!! :)");

            return;
        }

        const double line_spacing_;
        const double point_spacing_;
        const double min_segment_size_;
        const double min_hole_size_;
        const double search_radius_;

        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
};
} // namespace arp_tpp

/////////
// Main
/////////

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "READY TO INSTANTIATE NODE?!?");

    // Instantiate node
    std::shared_ptr<rclcpp::Node> tpp_node = std::make_shared<arp_tpp::TPPNode>();


    // Spin to accept service calls until ROS shuts down.
    rclcpp::spin(tpp_node);
    rclcpp::shutdown();
    return 0;
}


