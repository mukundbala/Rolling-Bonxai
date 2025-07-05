#include "rolling_map/rolling_map.h"

namespace RM
{
RollingMapNode::RollingMapNode(rclcpp::NodeOptions options)
:
rclcpp::Node("rolling_map_node",options)
{
    //Declare and Load all the parameters into the parameter objects
    declareParameters();
    loadParameters();

    //Load the map manager
    map_manager_ = MapManager(sensor_params_,map_params_,chunk_params_);
    
    auto qos = rclcpp::QoS{1};
    this->voxel_center_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/voxel_centers", qos);

    //Setup the TF2 objects
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    printParameters();
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    
    /*
    Reference: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Using-Stamped-Datatypes-With-Tf2-Ros-MessageFilter.html
    
    Using Message Filters is a pretty good way to ensure that the incoming cloud topic, which
    is by default in the robot's frame, is transformable to world frame. If a cloud message
    comes in at time t, the message_filter::Subscriber receives the message immediate, but
    does not trigger the callback directly. Instead, it passes the message to the TF MessageFilter.
    The TF MessageFilter check if a transform is available from cloud_message.header.frame_id to map at
    time t. If the transform is available, the callback fires. If its not, it waits up to 5s (parameterized).
    If it arrives during the timeout period, callback fires. If not, the message is dropped.
    This ensures that we don't have to write checks in the callback to see if transform is available and can
    safely transform without exceptions screwing up the callback.
     */
    using std::chrono_literals::operator""s;
    pcl_sub_.subscribe(this,"/cloud_in",rmw_qos_profile_sensor_data);
    tf_pcl_sub_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
                                    pcl_sub_,*tf2_buffer_,frame_params_.map_frame,5,
                                    this->get_node_logging_interface(),
                                    this->get_node_clock_interface(), 5s);
    
    tf_pcl_sub_->registerCallback(std::bind(&RollingMapNode::cloudCallback, this, std::placeholders::_1));
}

RollingMapNode::~RollingMapNode()
{
    //Maybe there are resources to release here? Lets see...
    return;
}

void RollingMapNode::declareParameters()
{
    //Frame Parameters
    this->declare_parameter("map_frame","map");
    this->declare_parameter("sensor_frame","laser_scanner");
    this->declare_parameter("robot_frame","base_link");

    //Map Parameters
    this->declare_parameter("m_resolution",0.1);
    this->declare_parameter("m_occupancy_min_thresh",0.12);
    this->declare_parameter("m_occupancy_max_thresh",0.97);
    
    //Sensor Parameters
    this->declare_parameter("s_max_z",3.0);
    this->declare_parameter("s_min_z",-3.0);
    this->declare_parameter("s_max_range",5.0);
    this->declare_parameter("s_min_range",0.5);
    this->declare_parameter("s_probability_hit",0.7);
    this->declare_parameter("s_probability_miss",0.4);
    
    //Chunk Manager Parameters
    this->declare_parameter("c_chunk_dim",32);
    this->declare_parameter("c_chunk_neiborhood",6);
    this->declare_parameter("c_chunk_cache_capacity",7);
    this->declare_parameter("c_chunk_folder_path","chunks");

    //PCL Filters
    this->declare_parameter("use_sor_filter",true);

    //Visualization Parameters
    this->declare_parameter("viz_occupied",true);
    this->declare_parameter("viz_free",false);
    this->declare_parameter("viz_chunks",false);
}   

void RollingMapNode::loadParameters()
{
    //Load Frame Parameters
    this->frame_params_.map_frame    = this->get_parameter("map_frame").as_string();
    this->frame_params_.sensor_frame = this->get_parameter("sensor_frame").as_string();
    this->frame_params_.robot_frame  = this->get_parameter("robot_frame").as_string();
    
    //Load Map Parameters
    this->map_params_.resolution              = this->get_parameter("m_resolution").as_double();
    this->map_params_.occupancy_min_threshold = this->get_parameter("m_occupancy_min_thresh").as_double();
    this->map_params_.occupancy_max_threshold = this->get_parameter("m_occupancy_max_thresh").as_double();
    
    //Load Sensor Parameters
    this->sensor_params_.max_z            = this->get_parameter("s_max_z").as_double();
    this->sensor_params_.min_z            = this->get_parameter("s_min_z").as_double();
    this->sensor_params_.max_range        = this->get_parameter("s_max_range").as_double();
    this->sensor_params_.min_range        = this->get_parameter("s_min_range").as_double();
    this->sensor_params_.probability_hit  = this->get_parameter("s_probability_hit").as_double();
    this->sensor_params_.probability_miss = this->get_parameter("s_probability_miss").as_double();
    
    //Load Chunk Parameters
    this->chunk_params_.chunk_dim            = this->get_parameter("c_chunk_dim").as_int();
    this->chunk_params_.chunk_neib           = this->get_parameter("c_chunk_neiborhood").as_int();
    this->chunk_params_.chunk_cache_capacity = this->get_parameter("c_chunk_cache_capacity").as_int();
    this->chunk_params_.chunk_folder_path    = this->get_parameter("c_chunk_folder_path").as_string();

    //Load PCL Filter Parameters
    this->use_sor_filter_ = this->get_parameter("use_sor_filter").as_bool();

    //Load Visualization Parameters
    this->viz_occupied_ = this->get_parameter("viz_occupied").as_bool();
    this->viz_free_     = this->get_parameter("viz_free").as_bool();
    this->viz_chunks_   = this->get_parameter("viz_chunks").as_bool();
}

void RollingMapNode::printParameters()
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Parameters loaded:");
    RCLCPP_INFO_STREAM(this->get_logger(), "  FRAME PARAMETERS:");
    RCLCPP_INFO_STREAM(this->get_logger(), "  map frame: " << this->frame_params_.map_frame);
    RCLCPP_INFO_STREAM(this->get_logger(), "  sensor frame: " << this->frame_params_.sensor_frame);
    RCLCPP_INFO_STREAM(this->get_logger(), "  robot frame: " << this->frame_params_.robot_frame);
    
    RCLCPP_INFO_STREAM(this->get_logger(), "  MAP PARAMETERS:");
    RCLCPP_INFO_STREAM(this->get_logger(), "  Map resolution: " << this->map_params_.resolution);
    RCLCPP_INFO_STREAM(this->get_logger(), "  Map min occupancy threshold: " << this->map_params_.occupancy_min_threshold);
    RCLCPP_INFO_STREAM(this->get_logger(), "  Map max occupancy threshold: " << this->map_params_.occupancy_max_threshold);
    
    RCLCPP_INFO_STREAM(this->get_logger(), "  SENSOR PARAMETERS:");
    RCLCPP_INFO_STREAM(this->get_logger(), "  Sensor max z: " << this->sensor_params_.max_z);
    RCLCPP_INFO_STREAM(this->get_logger(), "  Sensor min z: " << this->sensor_params_.min_z);
    RCLCPP_INFO_STREAM(this->get_logger(), "  Sensor max range: " << this->sensor_params_.max_range);
    RCLCPP_INFO_STREAM(this->get_logger(), "  Sensor min range: " << this->sensor_params_.min_range);
    RCLCPP_INFO_STREAM(this->get_logger(), "  Sensor probability hit: " << this->sensor_params_.probability_hit);
    RCLCPP_INFO_STREAM(this->get_logger(), "  Sensor probability miss: " << this->sensor_params_.probability_miss);
    RCLCPP_INFO_STREAM(this->get_logger(), "  CHUNK PARAMETERS:");
    RCLCPP_INFO_STREAM(this->get_logger(), "  Chunk dim: " << this->chunk_params_.chunk_dim);
    RCLCPP_INFO_STREAM(this->get_logger(), "  Chunk neiborhood: " << this->chunk_params_.chunk_neib);
    RCLCPP_INFO_STREAM(this->get_logger(), "  Chunk cache capacity: " << this->chunk_params_.chunk_cache_capacity);
    RCLCPP_INFO_STREAM(this->get_logger(), "  Chunk folder path: " << this->chunk_params_.chunk_folder_path);
}

void RollingMapNode::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
    const double start_time = this->get_clock()->now().seconds();
    PCLPointCloud pc;
    pcl::fromROSMsg(*msg, pc);
    // Filter NaNs and by z value
    size_t filtered_index = 0;
    for (const auto& point : pc.points) 
    {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) 
        {
            if (point.z >= this->sensor_params_.min_z && point.z <= this->sensor_params_.max_z)
            {
                pc.points[filtered_index++] = point;
            }
            
        }
    }
    pc.resize(filtered_index);
    
    // Remove Noisy Points using SOR filter
    if (use_sor_filter_)
    {
        pcl::StatisticalOutlierRemoval<PCLPoint> sorfilter;
        sorfilter.setInputCloud(pc.makeShared());
        sorfilter.setMeanK(30);
        sorfilter.setStddevMulThresh(1.0);
        
        PCLPointCloud filtered_pc;
        sorfilter.filter(filtered_pc);
        pc = std::move(filtered_pc);
    }

    geometry_msgs::msg::TransformStamped sensor_to_world_transform_stamped;
    try
    {
        sensor_to_world_transform_stamped = tf2_buffer_->lookupTransform(
            frame_params_.map_frame,msg->header.frame_id,msg->header.stamp,
            rclcpp::Duration::from_seconds(1.0)
        );
    }
    catch(const tf2::TransformException& ex)
    {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        return;
    }
    

    Eigen::Matrix4f sensor_to_world = tf2::transformToEigen(
                                    sensor_to_world_transform_stamped.transform).matrix().cast<float>();
    
                                    //Transform the pointcloud from the robot frame to the world frame
    PCLPointCloud world_points;
    pcl::transformPointCloud(pc,world_points,sensor_to_world);
    const auto& t = sensor_to_world_transform_stamped.transform.translation;
    PCLPoint global_sensor_position(t.x,t.y,t.z);
    
    //Chunk Updates happen here!
    map_manager_.updateMap(world_points,global_sensor_position);
    
    const double end_time = this->get_clock()->now().seconds();
    double elapsed_s = end_time - start_time;
    
    this->num_callback_hits_++;
    this->elapsed_total_time_s_ += elapsed_s;
    this->elapsed_avg_time_s_ = this->elapsed_total_time_s_/num_callback_hits_;
    RCLCPP_INFO_STREAM(this->get_logger(), "Callback Completion Time: " << elapsed_s           << "s\n"
                                        << "Total Callback Hits: "      << num_callback_hits_  << "\n"
                                        << "Average Completion Time: "  << elapsed_avg_time_s_ << "s\n");
}


void RollingMapNode::publishMap()
{
    return;
}

} // namespace RM