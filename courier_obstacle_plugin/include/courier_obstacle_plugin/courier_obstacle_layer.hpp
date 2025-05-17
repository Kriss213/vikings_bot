#ifndef COURIER_OBSTACLE_LAYER_HPP_
#define COURIER_OBSTACLE_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include <iostream>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/transforms.hpp>


namespace courier_obstacle_plugin
{

class CourierObstacleLayer : public nav2_costmap_2d::Layer
{
public:
    CourierObstacleLayer();

	virtual void onInitialize();
	virtual void updateBounds(
		double robot_x, double robot_y, double robot_yaw, double * min_x,
		double * min_y,
		double * max_x,
		double * max_y);
	virtual void updateCosts(
		nav2_costmap_2d::Costmap2D & master_grid,
		int min_i, int min_j, int max_i, int max_j);

	virtual void reset() { return; }

	virtual bool isClearable() {return true;}

	/**
	* @brief  Get current system time in seconds.
   	* @return Current time in seconds as int.
   	*/

	/**
	* @brief Transform sensor_msgs PointCloud2 to different frame.
	* @param msg PointCloud2 message.
	* @param frame Frame to which point cloud needs to be transformed.
   	* @return PointCloud2 message transformed to new frame.
   	*/
	sensor_msgs::msg::PointCloud2 transformToFrame(const sensor_msgs::msg::PointCloud2::SharedPtr msg, std::string frame);

	std::vector<std::string> point_topics_; // a vector of PointCloud2 topics for points to add/clear
	double obstacle_duration_; // how long a point should be kept before clearing

private:
	void PointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
	
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_subscription_;
    std::mutex mutex_;

	std::string costmap_frame_;
	std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> cloud_subscriptions_;

	struct TimedPointCloud {
		rclcpp::Time stamp;
		std::vector<geometry_msgs::msg::Point> points;
		};
	std::vector<TimedPointCloud> pointcloud_buffer_;
	std::mutex buffer_mutex_;

};

}

#endif  // COURIER_OBSTACLE_LAYER_HPP_