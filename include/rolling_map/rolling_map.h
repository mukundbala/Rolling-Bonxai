#ifndef ROLLING_MAP_H
#define ROLLING_MAP_H

#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "message_filters/subscriber.h"

#include "rolling_map/rolling_map_params.h"
#include "rolling_map/map_manager.h"

#include <mutex>
#include <filesystem>
#include <limits>
/*
ROS 2 Node to interface with incoming point cloud and
pose
*/
namespace RM
{
class RollingMapNode : public rclcpp::Node
{
public:
    using PCLPoint = pcl::PointXYZ;
    using PCLPointCloud = pcl::PointCloud<pcl::PointXYZ>;
    using PCLPointCloudSharedPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
    explicit RollingMapNode(rclcpp::NodeOptions options = rclcpp::NodeOptions());
    ~RollingMapNode();

private:

    /**
     * @brief Declare ROS 2 Parameters for this node
     */
    void declareParameters();

    /**
     * @brief Load ROS 2 Parameters for this node
     */
    void loadParameters();
    
    /**
     * @brief Parameter Printing for logging
     */
    void printParameters();

    /**
     * @brief Cloud Callback Function. This will trigger the updates
     * @params sensor_msgs::msg::PointCloud2::ConstSharedPtr msg
     */
    void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);


    void publishMap();

    //Map Manager
    MapManager map_manager_;

    //Parameters
    FrameParams frame_params_;
    MapParams map_params_;
    SensorParams sensor_params_;
    ChunkParams chunk_params_;

    //TF2
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

    //Publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_center_pub_;

    //TODO: Visualization Publisher but can do later

    //Subscriber using Message Filters. Make sure that the arriving pcl is transformable by dovetailing it with TF Listener
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pcl_sub_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> tf_pcl_sub_;

    //PCL Filtering flags
    bool use_sor_filter_ {true};
    
    //Visualization
    bool viz_occupied_ {true};
    bool viz_free_ {false};
    bool viz_chunks_ {false};

    //Profiling for callback
    int num_callback_hits_ {0};
    double elapsed_total_time_s_ {0};
    double elapsed_avg_time_s_ {0};
};
}
#endif //ROLLING_MAP_H