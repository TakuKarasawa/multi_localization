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

	void initialize(double x,double y,double yaw);
	void set_pose(double x,double y,double yaw);
	void publish_pose();
	void publish_tf();

	void motion_update(double dt);
	void measurement_update();

	geometry_msgs::Quaternion rpy_to_msg(double roll,double pitch,double yaw);

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Timer timer_;
	ros::Subscriber odom_sub_;
	ros::Subscriber obj_sub_;
	ros::Publisher pose_pub_;	


	std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
	std::shared_ptr<tf2_ros::Buffer> buffer_;
	std::shared_ptr<tf2_ros::TransformListener> listener_;

	nav_msgs::Odometry odom_;
	object_detector_msgs::ObjectPositions objects_;
	ros::Time now_time_;
	ros::Time last_time_;

	bool is_odom_tf_;
	bool is_first_;
	bool has_received_odom_;
	bool has_received_obj_;

	std::string odom_topic_name_;
	std::string obj_topic_name_;
	std::string pose_topic_name_;
	std::string map_frame_id_;
	std::string odom_frame_id_;
	std::string base_link_frame_id_;


	Eigen::Vector3d MU_;
	Eigen::Matrix3d SIGMA_;

	double HZ_;
	double INIT_X_;
	double INIT_Y_;
	double INIT_YAW_;
	double MOTION_NOISE_NN_;
	double MOTION_NOISE_NO_;
	double MOTION_NOISE_ON_;
	double MOTION_NOISE_OO_;
	double DISTANCE_NOISE_RATE_;
	double DIRECTION_NOISE_;

};

#endif	// CHILD_EKF_H_