#ifndef CHILD_EKF_H_
#define CHILD_EKF_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

#include "object_detector_msgs/ObjectPosition.h"
#include "object_detector_msgs/ObjectPositions.h"

class ChildEKF
{
public:
	ChildEKF();
	~ChildEKF();
	void process();

private:
	void odometry_callbak(const nav_msgs::OdometryConstPtr& msg);
	void object_positions_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg);
	void timer_callback(const ros::TimerEvent& event);



	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Timer timer_;
	ros::Publisher odom_sub_;
	ros::Subscriber obj_sub_;
	
	std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
	std::shared_ptr<tf2_ros::Buffer> buffer_;
	std::shared_ptr<tf2_ros::TransformListener> listener_;

	std::string odom_topic_name_;
	std::string obj_topic_name_;
};

#endif	// CHILD_EKF_H_