#include "courier_obstacle_plugin/courier_obstacle_layer.hpp"

#include "rclcpp/parameter_events_filter.hpp"


namespace courier_obstacle_plugin
{

    CourierObstacleLayer::CourierObstacleLayer(){
}

void CourierObstacleLayer::onInitialize() {
  auto node = node_.lock(); 
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("point_topics", rclcpp::ParameterValue(std::vector<std::string>{}));
  declareParameter("point_decay", rclcpp::ParameterValue(0.5));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "point_topics", point_topics_);
  node->get_parameter(name_ + "." + "point_decay", obstacle_duration_);
  
  for (const auto & topic : point_topics_) {
      auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        topic,
        rclcpp::SensorDataQoS(),
        std::bind(&CourierObstacleLayer::PointCloudCallback, this, std::placeholders::_1)
      );

      cloud_subscriptions_.push_back(sub);
      RCLCPP_INFO(node->get_logger(), "Subscribed to %s", topic.c_str());
  }

  

  // get costmap frame
  costmap_frame_ = layered_costmap_->getGlobalFrameID();

  current_ = true;
  
  RCLCPP_DEBUG(rclcpp::get_logger("nav2_costmap_2d"), "CourierObstacleLayer initialized!");
}

void CourierObstacleLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (!enabled_) return;

  std::lock_guard<std::mutex> lock(buffer_mutex_);

  for (const auto& pc : pointcloud_buffer_) {
    for (const auto& pt : pc.points) {
      *min_x = std::min(*min_x, pt.x);
      *min_y = std::min(*min_y, pt.y);
      *max_x = std::max(*max_x, pt.x);
      *max_y = std::max(*max_y, pt.y);
    }
  }
}


void CourierObstacleLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) {
  (void)min_i; (void)min_j; (void)max_i; (void)max_j;
  if (!enabled_) return;

  std::lock_guard<std::mutex> lock(buffer_mutex_);
  auto node = node_.lock();

  rclcpp::Time now = node->now();

  // Remove expired clouds
  pointcloud_buffer_.erase(
    std::remove_if(pointcloud_buffer_.begin(), pointcloud_buffer_.end(),
      [&](const TimedPointCloud &pc) {
        return (now - pc.stamp).seconds() > obstacle_duration_;
      }),
    pointcloud_buffer_.end());

  for (const auto &pc : pointcloud_buffer_) {
    for (const auto &pt : pc.points) {
      unsigned int mx, my;
      if (master_grid.worldToMap(pt.x, pt.y, mx, my)) {
        master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
}


  
void CourierObstacleLayer::PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    TimedPointCloud pc;
    pc.stamp = msg->header.stamp;

    sensor_msgs::msg::PointCloud2 transformed_cloud = transformToFrame(msg, costmap_frame_);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(transformed_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(transformed_cloud, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      geometry_msgs::msg::Point pt;
      pt.x = *iter_x;
      pt.y = *iter_y;
      pt.z = *iter_z;
      pc.points.push_back(pt);
    }

    pointcloud_buffer_.push_back(pc);
    
}

sensor_msgs::msg::PointCloud2 CourierObstacleLayer::transformToFrame(const sensor_msgs::msg::PointCloud2::SharedPtr msg, std::string frame) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    try {
        transform_stamped = tf_->lookupTransform(frame, msg->header.frame_id, tf2::TimePointZero);

    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(rclcpp::get_logger("nav2_costmap_2d"), "Transform failed: %s", ex.what());
        return transformed_cloud; // returns empty cloud
    }
    
    pcl_ros::transformPointCloud(costmap_frame_, transform_stamped, *msg, transformed_cloud);
    return transformed_cloud;
}


}  // namespace courier_obstacle_plugin



#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(courier_obstacle_plugin::CourierObstacleLayer, nav2_costmap_2d::Layer)