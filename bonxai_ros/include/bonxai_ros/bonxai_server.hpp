#pragma once

#include <memory>
#include <string>
#include <chrono>
#include <cstdint>
#include <map>
#include <set>
#include <tuple>
#include <vector>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Eigen
#include <eigen3/Eigen/Geometry>

// Bonxai
#include "bonxai_map/occupancy_map.hpp"

// Custom services and messages
#include "bonxai_msgs/srv/get_occupied_voxels.hpp"
#include "bonxai_msgs/srv/get_free_voxels.hpp"
#include "bonxai_msgs/msg/occupancy_map_stats.hpp"
#include "surf_multirobot_msgs/msg/voxel_delta.hpp"

namespace Bonxai
{

struct BonxaiParams
{
  double static_resolution{0.0};
  double dynamic_resolution{0.0};
  std::string frame_id;
  std::string base_frame_id;
  std::string topic_in;
  std::string delta_topic_in;

  double occupancy_min_z{0.0};
  double occupancy_max_z{0.0};
  double occupancy_threshold{0.50};

  double sensor_max_range{0.0};
  double sensor_hit{0.0};
  double sensor_miss{0.0};
  double sensor_min{0.0};
  double sensor_max{0.0};
  
  // Cleanup
  double cleanup_interval_sec{300.0};  // 5 minutes
  
  // Stats
  bool enable_stats{true};
  bool quick_stats{true};
  bool publish_occupied_voxels{true};
  double stats_publish_rate{1.0};  // Hz
  double static_voxel_publish_rate{1.0};  // Hz
  double dynamic_voxel_publish_rate{5.0};  // Hz

  // Dynamic obstacles
  bool dynamic_obstacles_enabled{true};
  double dynamic_obstacle_decay_time_sec{2.0};
  double dynamic_obstacle_decay_interval_sec{0.25};
  double dynamic_obstacle_static_demotion_time_sec{20.0};
  int dynamic_obstacle_static_stability_hits{3};
  double dynamic_obstacle_static_stability_time_sec{1.0};
  double dynamic_obstacle_min_probability{0.05};

  // Static map persistence
  std::string static_map_path;
  bool static_map_load_on_startup{false};
  bool static_map_save_on_shutdown{true};
  
};

struct DynamicCellState
{
  uint32_t consecutive_hits{0};
  std::chrono::steady_clock::time_point last_seen{std::chrono::steady_clock::now()};
  bool promoted_to_static{false};
  Bonxai::CoordT promoted_static_coord{};
};

struct RemoteSourceLayer
{
  uint64_t map_epoch{0U};
  uint64_t last_version{0U};
  bool awaiting_full_refresh{true};
  std::unique_ptr<Bonxai::OccupancyMap> occupancy;
  std::set<Bonxai::CoordT> dynamic_voxels;
};

class BonxaiServer : public rclcpp::Node
{
public:
  explicit BonxaiServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~BonxaiServer() override;

private:
  // Initialization methods
  void load_parameters();
  void init_tf();
  void init_publishers();
  void init_subscribers();
  void init_services();
  void init_timers();

  // Callbacks
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void voxel_delta_callback(
    const surf_multirobot_msgs::msg::VoxelDelta::SharedPtr msg);
  void reset_remote_source(RemoteSourceLayer & source, uint64_t map_epoch);
  void get_fused_occupied_voxels(
    std::vector<Bonxai::CoordT> & coords, bool include_static, bool include_dynamic) const;
  void get_fused_free_voxels(std::vector<Bonxai::CoordT> & coords) const;
  
  // Service handlers
  void handle_get_occupied_voxels(
    const std::shared_ptr<bonxai_msgs::srv::GetOccupiedVoxels::Request> request,
    std::shared_ptr<bonxai_msgs::srv::GetOccupiedVoxels::Response> response);
    
  void handle_get_free_voxels(
    const std::shared_ptr<bonxai_msgs::srv::GetFreeVoxels::Request> request,
    std::shared_ptr<bonxai_msgs::srv::GetFreeVoxels::Response> response);

  void handle_save_static_map(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handle_load_static_map(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool save_static_map(std::string& error_message) const;
  bool load_static_map(std::string& error_message);
  
  // Timer callbacks
  void cleanup_timer_callback();
  void stats_timer_callback();
  void static_voxel_timer_callback();
  void dynamic_voxel_timer_callback();

  // Helper: Fill VoxelGrid message from coordinate vector
  void fill_voxel_grid_msg(
    const std::vector<Bonxai::CoordT>& coords,
    bonxai_msgs::msg::VoxelGrid& msg);

  void update_dynamic_obstacle_layer(
    const std::vector<Eigen::Vector3d>& map_points,
    const Eigen::Vector3d& sensor_origin);

  void get_dynamic_obstacle_voxels(std::vector<Bonxai::CoordT>& coords) const;
  Bonxai::CoordT dynamic_to_static_coord(const Bonxai::CoordT& coord) const;
  bool dynamic_voxel_overlaps_static(const Bonxai::CoordT& coord) const;
  void reconcile_dynamic_with_static_map();

  void decay_dynamic_obstacles();

  void fill_pcl_msg(
    const std::vector<Bonxai::CoordT>& coords,
    sensor_msgs::msg::PointCloud2& msg,
    const Bonxai::OccupancyMap& coordinate_map);
    
  // Helper: Fill OccupancyMapStats message
  void fill_stats_msg(bonxai_msgs::msg::OccupancyMapStats& msg);

  // Parameters
  BonxaiParams params_;

  // Flags
  bool updated_map_once_{false};

  // Occupancy maps
  std::unique_ptr<Bonxai::OccupancyMap> occupancy_map_;
  std::unique_ptr<Bonxai::OccupancyMap> dynamic_obstacle_map_;
  std::map<std::tuple<int32_t, int32_t, int32_t>, DynamicCellState> dynamic_obstacle_states_;
  std::map<std::string, RemoteSourceLayer> remote_sources_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<surf_multirobot_msgs::msg::VoxelDelta>::SharedPtr delta_sub_;
  
  // Service servers
  rclcpp::Service<bonxai_msgs::srv::GetOccupiedVoxels>::SharedPtr occupied_voxels_service_;
  rclcpp::Service<bonxai_msgs::srv::GetFreeVoxels>::SharedPtr free_voxels_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_static_map_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_static_map_service_;
  
  // Publishers
  rclcpp::Publisher<bonxai_msgs::msg::OccupancyMapStats>::SharedPtr stats_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_voxel_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_voxel_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_voxel_publisher_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr cleanup_timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::TimerBase::SharedPtr static_voxel_timer_;
  rclcpp::TimerBase::SharedPtr dynamic_voxel_timer_;
  rclcpp::TimerBase::SharedPtr dynamic_decay_timer_;
  
  // Statistics
  uint64_t point_clouds_processed_{0};
  uint64_t total_points_processed_{0};
};

}  // namespace Bonxai
