#pragma once

#include <memory>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <visualization_msgs/msg/marker_array.hpp>

#include "rolling_bonxai/rolling_occupancy_map.hpp"

// Logger interfaces
#include "rolling_bonxai/logger.hpp"
#include "rolling_bonxai_ros/ros_logger.hpp"

// Messages
#include "rolling_bonxai_msgs/msg/chunk_stats.hpp"
#include "rolling_bonxai_msgs/msg/quick_occupancy_stats.hpp"
#include "rolling_bonxai_msgs/msg/rolling_state.hpp"

// Services
#include "std_srvs/srv/trigger.hpp"

namespace RollingBonxai
{

struct ServerParams
{
  std::string frame_id;
  std::string base_frame_id;
  std::string topic_in;
  bool qos_reliable{false};  // subscribe to topic_in reliably instead of SensorDataQoS (best effort)

  double cleanup_interval_sec{300.0};

  bool enable_stats{true};
  bool quick_stats{true};
  bool publish_occupied_voxels{true};
  double stats_publish_rate{1.0};
};

class RollingBonxaiServer : public rclcpp::Node
{
public:
  explicit RollingBonxaiServer(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  /**
   * @brief Load all ROS parameters into server and map parameter structs
   */
  void loadParameters();

  /**
   * @brief Initialize framework-agnostic logger backed by rclcpp::Logger
   */
  void initLogger();

  /**
   * @brief Initialize TF2 buffer and listener
   */
  void initTF();

  /**
   * @brief Construct the rolling occupancy map
   */
  void initMap();

  /**
   * @brief Initialize PointCloud2 subscriber
   */
  void initSubscriber();

  /**
   * @brief Initialize PointCloud2 services
   */
  void initService();

  /**
   * @brief Initialize statistics publishers and timer
   */
  void initStatsPublishers();

  /**
   * @brief Initialize chunk visualization publishers and timer
   */
  void initChunkVisualization();

  /**
   * @brief Periodic statistics publishing callback
   *
   * Publishes:
   *  - Occupied voxel PointCloud2 (if enabled)
   *  - ChunkStats
   *  - QuickOccupancyStats (MiB)
   */
  void publishStats();

  /**
   * @brief Service that allows a user to clean memory
   */
  void cleanMemoryServiceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Publish chunk visualization and transition state
   *
   * Chunk visualization:
   *  - Green  → dirty
   *  - Blue   → clean
   *  - Gray   → pending
   *
   * @complexity O(N) where N = active + pending chunks
   */
  void publishChunkVisualization();

  /**
   * @brief Handle incoming sensor point clouds
   */
  void pointcloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  ServerParams server_params_;
  RollingOccupancyMap::AllParameters rolling_params_;

  std::shared_ptr<Logger> logger_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  std::unique_ptr<RollingBonxai::RollingOccupancyMap> occupancy_map_;
  uint64_t point_clouds_processed_{0};

  // Stats publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_pub_;
  rclcpp::Publisher<rolling_bonxai_msgs::msg::ChunkStats>::SharedPtr chunk_stats_pub_;
  rclcpp::Publisher<rolling_bonxai_msgs::msg::QuickOccupancyStats>::SharedPtr quick_occ_stats_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clean_memory_srv_;

  // Chunk visualization
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr chunk_marker_pub_;
  rclcpp::Publisher<rolling_bonxai_msgs::msg::RollingState>::SharedPtr transition_state_pub_;
  rclcpp::TimerBase::SharedPtr chunk_vis_timer_;
};

} // namespace RollingBonxai
