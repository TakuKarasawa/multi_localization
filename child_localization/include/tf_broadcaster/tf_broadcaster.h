#ifndef TF_BROADCASTER_H_
#define TF_BROADCASTER_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

struct Coordinate
{
	double x;
	double y;
	double z;
	double roll;
	double pitch;
	double yaw;
};

class TFBroadcaster
{
public:
	TFBroadcaster();
	~TFBroadcaster();
	void process();

private:
	void odom_callback(const nav_msgs::OdometryConstPtr& msg);
	geometry_msgs::TransformStamped create_transform_stamped_msg(std::string frame_id,std::string child_frame_id,Coordinate coordinate);

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Subscriber odom_sub_;

	std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

	Coordinate lidar_coordinate_;
	Coordinate thetas_coordinate_;
	Coordinate realsense_coordinate_;
	
	geometry_msgs::TransformStamped lidar_transform_stamped_;
	geometry_msgs::TransformStamped thetas_transform_stamped_;
	geometry_msgs::TransformStamped realsense_transform_stamped_;

	bool is_odom_tf_;
	std::string odom_topic_name_;
	std::string odom_frame_id_;
	std::string base_link_frame_id_;
	std::string lidar_frame_id_;
	std::string thetas_frame_id_;
    std::string dynamixel_frame_id_;
    std::string realsense_frame_id_;
};

#endif	// TF_BROADCASTER_H_