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

#include <sstream>
#include <fstream>

#include "object_detector_msgs/ObjectPosition.h"
#include "object_detector_msgs/ObjectPositions.h"

class ObjectError
{
public:
	ObjectError(int id,double error) :
		id_(id), error_(error) {}

	int id_;
	double error_;

private:
};

class ObjectDistance
{
public:
	ObjectDistance(std::string name,double x,double y,double z) :
		name_(name), x_(x), y_(y), z_(z)
	{
		calc_distance();
		calc_theta();
	}
	
	void push_object_error(ObjectError object_error)
	{
		object_errors_.push_back(object_error);
	}

	void calc_distance()
	{
		distance_ = std::sqrt(x_*x_ + z_*z_);
	}

	void calc_theta()
	{
		theta_ = std::atan2(z_,x_) - 0.5*M_PI;
	}

	void sort_object_error()
	{
		std::sort(object_errors_.begin(),object_errors_.end(),[](const ObjectError& a,const ObjectError& b) { return a.error_ < b.error_; });
	}

	std::string name_;
	double x_;
	double y_;
	double z_;

	double distance_;
	double theta_;
	std::vector<ObjectError> object_errors_;

private:
};

struct MeasurementList
{
	MeasurementList(int id,double distance,double theta) :
		id_(id), distance_(distance), theta_(theta) {}

	int id_;
	double distance_;
	double theta_;
};

struct ObjectNode
{
	ObjectNode(std::string name_,float r_,float g_,float b_) :
	name(name_), r(r_), g(g_), b(b_) {}

	std::string name;
	float r;
	float g;
	float b;
};

class ChildEKF
{
public:
	ChildEKF();
	~ChildEKF();
	void process();

private:
	void odometry_callbak(const nav_msgs::OdometryConstPtr& msg);
	void object_positions_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg);

	void initialize(double x,double y,double yaw);
	void set_pose(double x,double y,double yaw);
	void publish_pose();
	void publish_tf();

	void motion_update(double dt);
	void measurement_update();

	void load_parameter();
	void read_csv();
    std::vector<std::string> split(std::string& input,char delimiter);

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
	object_detector_msgs::ObjectPositions obj_poses_;
	visualization_msgs::MarkerArray markers_;
	ros::Time now_time_;
	ros::Time last_time_;

	XmlRpc::XmlRpcValue object_list_;
	std::vector<ObjectNode> objects_;
	std::vector<MeasurementList> measurement_list_;

	bool is_odom_tf_;
	bool is_first_;
	bool has_received_odom_;
	bool has_received_obj_;

	int count_;

	std::string dir_path_ = "/home/amsl/catkin_ws/src/multi_localization/map_updater/record/";
	std::string file_name_;
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

	double RANGE_TH_;	
};

#endif	// CHILD_EKF_H_