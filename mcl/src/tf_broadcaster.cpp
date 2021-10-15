#include "tf_broadcaster/tf_broadcaster.h"

TFBroadcaster::TFBroadcaster() : private_nh_("~")
{
	private_nh_.param("odom_topic_name",odom_topic_name_,{"roomba/odometry"});
	private_nh_.param("odom_frame_id",odom_frame_id_,{"odom"});
	private_nh_.param("base_link_frame_id",base_link_frame_id_,{"base_link"});
	private_nh_.param("lidar_frame_id",lidar_frame_id_,{"scan"});
	private_nh_.param("thetas_frame_id",thetas_frame_id_,{"theta_s"});
	private_nh_.param("dynamixel_frame_id",dynamixel_frame_id_,{"dynamixel"});
	private_nh_.param("realsense_frame_id",realsense_frame_id_,{"realsense"});
	
	private_nh_.param("lidar_x",lidar_coordinate_.x,{0.0});
	private_nh_.param("lidar_y",lidar_coordinate_.y,{0.0});
	private_nh_.param("lidar_z",lidar_coordinate_.z,{0.0});
	private_nh_.param("lidar_roll",lidar_coordinate_.roll,{0.0});
	private_nh_.param("lidar_pitch",lidar_coordinate_.pitch,{0.0});
	private_nh_.param("lidar_yaw",lidar_coordinate_.yaw,{0.0});
	private_nh_.param("thetas_x",thetas_coordinate_.x,{0.0});
	private_nh_.param("thetas_y",thetas_coordinate_.y,{0.0});
	private_nh_.param("thetas_z",thetas_coordinate_.z,{0.0});
	private_nh_.param("thetas_roll",thetas_coordinate_.roll,{0.0});
	private_nh_.param("thetas_pitch",thetas_coordinate_.pitch,{0.0});
	private_nh_.param("thetas_yaw",thetas_coordinate_.yaw,{0.0});
	private_nh_.param("realsense_x",realsense_coordinate_.x,{0.0});
	private_nh_.param("realsense_y",realsense_coordinate_.y,{0.0});
	private_nh_.param("realsense_z",realsense_coordinate_.z,{0.0});
	private_nh_.param("realsense_roll",realsense_coordinate_.roll,{0.0});
	private_nh_.param("realsense_pitch",realsense_coordinate_.pitch,{0.0});
	private_nh_.param("realsense_yaw",realsense_coordinate_.yaw,{0.0});

	odom_sub_ = nh_.subscribe(odom_topic_name_,1,&TFBroadcaster::odom_callback,this);

	static_broadcaster_.reset(new tf2_ros::StaticTransformBroadcaster);
	broadcaster_.reset(new tf2_ros::TransformBroadcaster);

	lidar_transform_stamped_ = create_transform_stamped_msg(base_link_frame_id_,lidar_frame_id_,lidar_coordinate_);
	thetas_transform_stamped_ = create_transform_stamped_msg(base_link_frame_id_,thetas_frame_id_,thetas_coordinate_);
	realsense_transform_stamped_ = create_transform_stamped_msg(base_link_frame_id_,realsense_frame_id_,realsense_coordinate_);
}

TFBroadcaster::~TFBroadcaster() { }

void TFBroadcaster::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
	geometry_msgs::TransformStamped odom_transform_stamped;
	odom_transform_stamped.header = msg->header;
	odom_transform_stamped.header.frame_id = odom_frame_id_;
	odom_transform_stamped.child_frame_id = base_link_frame_id_;

	odom_transform_stamped.transform.translation.x = msg->pose.pose.position.x;
	odom_transform_stamped.transform.translation.y = msg->pose.pose.position.y;
	odom_transform_stamped.transform.translation.z = msg->pose.pose.position.z;
	odom_transform_stamped.transform.rotation = msg->pose.pose.orientation;

	broadcaster_->sendTransform(odom_transform_stamped);
}

geometry_msgs::TransformStamped TFBroadcaster::create_transform_stamped_msg(std::string frame_id,std::string child_frame_id,Coordinate coordinate)
{
	geometry_msgs::TransformStamped static_transform_stamped;
	static_transform_stamped.header.stamp = ros::Time::now();
	static_transform_stamped.header.frame_id = frame_id;
	static_transform_stamped.child_frame_id = child_frame_id;

	static_transform_stamped.transform.translation.x = coordinate.x;
	static_transform_stamped.transform.translation.y = coordinate.y;
	static_transform_stamped.transform.translation.z = coordinate.z;

	tf2::Quaternion quaternion;
	quaternion.setRPY(coordinate.roll,coordinate.pitch,coordinate.yaw);
	static_transform_stamped.transform.rotation.x = quaternion.x();
	static_transform_stamped.transform.rotation.y = quaternion.y();
	static_transform_stamped.transform.rotation.z = quaternion.z();
	static_transform_stamped.transform.rotation.w = quaternion.w();	

	return static_transform_stamped;
}

void TFBroadcaster::process()
{ 
	static_broadcaster_->sendTransform({ lidar_transform_stamped_, thetas_transform_stamped_, realsense_transform_stamped_ });
	ros::spin(); 
}