#include "rolling_bonxai_ros/rolling_bonxai_server.hpp"

#include <tf2/exceptions.h>

namespace
{
inline double bytesToMiB(std::size_t bytes)
{
  return static_cast<double>(bytes) / (1024.0 * 1024.0);
}
}

namespace RollingBonxai
{

RollingBonxaiServer::RollingBonxaiServer(
  const rclcpp::NodeOptions& options)
: rclcpp::Node("rolling_bonxai_server", options)
{
  loadParameters();
  initLogger();
  initTF();
  initMap();
  initSubscriber();
  initService();
  initStatsPublishers();
  initChunkVisualization();

  RCLCPP_INFO(get_logger(), "RollingBonxaiServer started");
}

void RollingBonxaiServer::loadParameters()
{
  server_params_.frame_id = declare_parameter<std::string>("frame_id", "map");
  server_params_.base_frame_id = declare_parameter<std::string>("base_frame_id", "base_footprint");
  server_params_.topic_in = declare_parameter<std::string>("topic_in", "/points");
  server_params_.qos_reliable = declare_parameter<bool>("qos_reliable", false);

  server_params_.cleanup_interval_sec =
    declare_parameter<double>("server.cleanup_interval_sec", 300.0);

  server_params_.enable_stats =
    declare_parameter<bool>("stats.enable_stats", true);

  server_params_.quick_stats =
    declare_parameter<bool>("stats.quick_stats", true);

  server_params_.publish_occupied_voxels =
    declare_parameter<bool>("stats.publish_occupied_voxels", true);

  server_params_.stats_publish_rate =
    declare_parameter<double>("stats.publish_rate", 1.0);

  auto& mp = rolling_params_.map_params;

  mp.resolution = declare_parameter<double>("occupancy.resolution", 0.05);
  mp.occupancy_min_z = declare_parameter<double>("occupancy.occupancy_min_z", -100.0);
  mp.occupancy_max_z = declare_parameter<double>("occupancy.occupancy_max_z", 100.0);
  mp.occupancy_threshold = declare_parameter<double>("occupancy.occupancy_threshold", 0.5);

  mp.sensor_max_range = declare_parameter<double>("sensor_model.max_range", 30.0);
  mp.sensor_hit = declare_parameter<double>("sensor_model.hit", 0.7);
  mp.sensor_miss = declare_parameter<double>("sensor_model.miss", 0.4);
  mp.sensor_min = declare_parameter<double>("sensor_model.min", 0.12);
  mp.sensor_max = declare_parameter<double>("sensor_model.max", 0.97);

  rolling_params_.chunk_size =
    declare_parameter<double>("rolling.coordinate_system.chunk_size", 5.0);

  rolling_params_.hysteresis_ratio =
    declare_parameter<double>("rolling.transition_manager.hysteresis_ratio", 0.20);

  rolling_params_.loading_policy_name =
    declare_parameter<std::string>("rolling.loading_policy.name", "neighbourhood");

  rolling_params_.nb_radius =
    declare_parameter<int>("rolling.loading_policy.neighborhood.radius", 2);

  rolling_params_.planar_z_min =
    declare_parameter<int>("rolling.loading_policy.planar.z_min", 1);

  rolling_params_.planar_z_max =
    declare_parameter<int>("rolling.loading_policy.planar.z_max", 3);

  rolling_params_.planar_use_relative =
    declare_parameter<bool>("rolling.loading_policy.planar.use_relative", false);

  rolling_params_.temporal_age_weightage =
    declare_parameter<double>("rolling.loading_policy.temporal.age_weightage", 0.3);

  rolling_params_.temporal_max_age_s =
    declare_parameter<int>("rolling.loading_policy.temporal.max_age_s", 300);

  rolling_params_.full_save_folder_path =
    declare_parameter<std::string>("rolling.storage.full_save_folder_path", "");

  rolling_params_.asyncio.enable_io = declare_parameter<bool>("rolling.asyncio.enable_io",true);
  
  rolling_params_.asyncio.num_load_threads =
    static_cast<size_t>(declare_parameter<int>("rolling.asyncio.num_load_threads", 3));

  rolling_params_.asyncio.num_save_threads =
    static_cast<size_t>(declare_parameter<int>("rolling.asyncio.num_save_threads", 1));
}

void RollingBonxaiServer::initLogger()
{
  logger_ = std::make_shared<RosLogger>(get_logger());
}

void RollingBonxaiServer::initTF()
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void RollingBonxaiServer::initMap()
{
  occupancy_map_ =
    std::make_unique<RollingBonxai::RollingOccupancyMap>(rolling_params_, logger_);
}

void RollingBonxaiServer::initSubscriber()
{
  // Best effort drops large clouds whose fragments are lost, so bag playback uses reliable;
  // a reliable subscriber won't connect to a best-effort publisher.
  const rclcpp::QoS qos = server_params_.qos_reliable ?
    rclcpp::QoS(rclcpp::KeepLast(50)).reliable() : rclcpp::SensorDataQoS();
  pointcloud_sub_ =
    create_subscription<sensor_msgs::msg::PointCloud2>(
      server_params_.topic_in,
      qos,
      std::bind(&RollingBonxaiServer::pointcloudCallback, this, std::placeholders::_1));
}

void RollingBonxaiServer::initService()
{
  clean_memory_srv_ =
    create_service<std_srvs::srv::Trigger>(
      "/rolling_bonxai/clean_memory",
      std::bind(
        &RollingBonxaiServer::cleanMemoryServiceCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
  }

void RollingBonxaiServer::initStatsPublishers()
{
  if (!server_params_.enable_stats) {
    return;
  }

  if (server_params_.publish_occupied_voxels) {
    voxel_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
        "/rolling_bonxai/occupied_voxels",
        rclcpp::QoS(1).transient_local());
  }

  chunk_stats_pub_ =
    create_publisher<rolling_bonxai_msgs::msg::ChunkStats>("/rolling_bonxai/chunk_stats", 10);

  quick_occ_stats_pub_ =
    create_publisher<rolling_bonxai_msgs::msg::QuickOccupancyStats>(
      "/rolling_bonxai/quick_occupancy_stats", 10);

  const auto period =
    std::chrono::duration<double>(1.0 / server_params_.stats_publish_rate);

  stats_timer_ =
    create_wall_timer(period, std::bind(&RollingBonxaiServer::publishStats, this));
}

void RollingBonxaiServer::initChunkVisualization()
{
  chunk_marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("/rolling_bonxai/chunk_markers", 1);

  transition_state_pub_ =
    create_publisher<rolling_bonxai_msgs::msg::RollingState>("/rolling_bonxai/transition_state", 10);

  chunk_vis_timer_ =
    create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&RollingBonxaiServer::publishChunkVisualization, this));
}

void RollingBonxaiServer::cleanMemoryServiceCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!occupancy_map_->isStatsStable()) {
    response->success = false;
    response->message = "Memory cleanup skipped: map not in stats-safe state";
    return;
  }

  occupancy_map_->releaseUnusedMemory();
  occupancy_map_->shrinkToFit();

  response->success = true;
  response->message = "RollingBonxai memory cleanup completed";
}


void RollingBonxaiServer::publishStats()
{
  const auto now_time = now();

  if (voxel_pub_) {
    std::vector<Eigen::Vector3d> points;
    occupancy_map_->getOccupiedVoxels(points);

    if (!points.empty()) {
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      cloud.reserve(points.size());

      for (const auto& p : points) {
        pcl::PointXYZRGB pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = p.z();
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;
        cloud.push_back(pt);
      }

      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(cloud, msg);
      msg.header.frame_id = server_params_.frame_id;
      msg.header.stamp = now_time;
      voxel_pub_->publish(msg);
    }
  }

  rolling_bonxai_msgs::msg::ChunkStats cs;
  cs.header.stamp = now_time;
  cs.header.frame_id = server_params_.frame_id;
  cs.active_chunk_count = occupancy_map_->getActiveChunkCount();
  cs.dirty_chunk_count = occupancy_map_->getDirtyChunkCount();
  cs.clean_chunk_count = occupancy_map_->getCleanChunkCount();
  chunk_stats_pub_->publish(cs);

  const auto qs = occupancy_map_->getStats(false);
  rolling_bonxai_msgs::msg::QuickOccupancyStats qs_msg;
  qs_msg.header.stamp = now_time;
  qs_msg.header.frame_id = server_params_.frame_id;
  qs_msg.total_memory_mib = bytesToMiB(qs.total_memory_bytes);
  qs_msg.grid_memory_mib = bytesToMiB(qs.grid_memory_bytes);
  qs_msg.buffer_memory_mib = bytesToMiB(qs.buffer_memory_bytes);
  qs_msg.total_active_cells = qs.total_active_cells;
  qs_msg.total_insertions = qs.total_insertions;
  qs_msg.hit_points_added = qs.hit_points_added;
  qs_msg.miss_points_added = qs.miss_points_added;
  qs_msg.point_clouds_processed = point_clouds_processed_;
  qs_msg.has_data = qs.has_data;
  quick_occ_stats_pub_->publish(qs_msg);
}

void RollingBonxaiServer::publishChunkVisualization()
{
  const auto now_time = now();

  const auto states = occupancy_map_->getChunkStates();
  const auto pending = occupancy_map_->getPendingChunks();

  visualization_msgs::msg::MarkerArray arr;
  const double size = rolling_params_.chunk_size;
  int id = 0;

  for (const auto& [pos, dirty] : states) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = server_params_.frame_id;
    m.header.stamp = now_time;
    m.ns = "chunks";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = pos.x();
    m.pose.position.y = pos.y();
    m.pose.position.z = pos.z();
    m.pose.orientation.w = 1.0;
    m.scale.x = size;
    m.scale.y = size;
    m.scale.z = size;

    if (dirty) {
      m.color.r = 0.0f;
      m.color.g = 1.0f;
      m.color.b = 0.0f;
    } else {
      m.color.r = 0.0f;
      m.color.g = 0.0f;
      m.color.b = 1.0f;
    }
    m.color.a = 0.15f;
    arr.markers.push_back(m);
  }

  for (const auto& pos : pending) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = server_params_.frame_id;
    m.header.stamp = now_time;
    m.ns = "chunks";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = pos.x();
    m.pose.position.y = pos.y();
    m.pose.position.z = pos.z();
    m.pose.orientation.w = 1.0;
    m.scale.x = size;
    m.scale.y = size;
    m.scale.z = size;
    m.color.r = 0.5f;
    m.color.g = 0.5f;
    m.color.b = 0.5f;
    m.color.a = 0.15f;
    arr.markers.push_back(m);
  }

  chunk_marker_pub_->publish(arr);

  rolling_bonxai_msgs::msg::RollingState ts;
  ts.header.stamp = now_time;
  ts.header.frame_id = server_params_.frame_id;
  ts.state = occupancy_map_->getTransitionState();
  transition_state_pub_->publish(ts);
}

void RollingBonxaiServer::pointcloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(
      server_params_.frame_id,
      msg->header.frame_id,
      msg->header.stamp,
      rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException& ex) {
    RCLCPP_WARN(get_logger(), "TF failed: %s", ex.what());
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);
  if (cloud->empty()) return;

  Eigen::Vector3d origin(
    tf.transform.translation.x,
    tf.transform.translation.y,
    tf.transform.translation.z);

  Eigen::Vector3d linvel(0.0,0.0,0.0);

  Eigen::Quaterniond q(
    tf.transform.rotation.w,
    tf.transform.rotation.x,
    tf.transform.rotation.y,
    tf.transform.rotation.z);

  std::vector<Eigen::Vector3d> pts;
  pts.reserve(cloud->size());

  for (const auto& p : cloud->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;

    Eigen::Vector3d ps(p.x, p.y, p.z);
    Eigen::Vector3d pm = q * ps + origin;

    if (pm.z() < rolling_params_.map_params.occupancy_min_z ||
        pm.z() > rolling_params_.map_params.occupancy_max_z)
      continue;

    pts.push_back(pm);
  }

  if (!pts.empty()) {
    occupancy_map_->insertPointCloud(
      pts, origin, linvel,
      rolling_params_.map_params.sensor_max_range);
    ++point_clouds_processed_;
  }
}

} // namespace RollingBonxai


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(RollingBonxai::RollingBonxaiServer)