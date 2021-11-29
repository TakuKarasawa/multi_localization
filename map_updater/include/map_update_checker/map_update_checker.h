#ifndef MAP_UPDATE_CHECKER_H_
#define MAP_UPDATE_CHECKER_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>

#include <sstream>
#include <fstream>

#include "object_detector_msgs/ObjectPosition.h"
#include "object_detector_msgs/ObjectPositions.h"

struct ObjectError
{
	ObjectError(int id_,double error_) :
		id(id_), error(error_) {}
	
	std::string name;
	int id;
	double error;
};

struct ObjectDistance
{
	ObjectDistance(std::string name_,double distance_) :
		name(name_), distance(distance_) {}

	void push_object_error(ObjectError object_error)
	{ 
		obejct_errors.push_back(object_error); 
	}

	void sort_object_error()
	{
		std::sort(obejct_errors.begin(),obejct_errors.end(),[](const ObjectError& a,const ObjectError& b) { return a.error < b.error; });
	}

	std::string name;
	double distance;

	std::vector<ObjectError> obejct_errors;
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

class MapUpdateChecker
{
public:
	MapUpdateChecker();
	~MapUpdateChecker();
	void process();

private:
	void pose_callback(const geometry_msgs::PoseStampedConstPtr& msg);
	void obj_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg);

	void load_parameter();
	void read_csv(visualization_msgs::MarkerArray& markers);
    std::vector<std::string> split(std::string& input,char delimiter);

	void check_object_range(visualization_msgs::MarkerArray& markers);

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Publisher markers_pub_;
	ros::Subscriber pose_sub_;
	ros::Subscriber obj_sub_;

	geometry_msgs::PoseStamped pose_;
	object_detector_msgs::ObjectPositions obj_poses_;

	bool is_first_;	
	int count_;
	double first_time_;
	
	bool has_received_pose_;
	bool has_received_obj_;

	std::string marker_frame_id_;
	std::string est_pose_topic_name_;
	std::string obj_topic_name_;

	std::string dir_path_ = "/home/amsl/catkin_ws/src/multi_localization/map_updater/record/";
    std::string file_name_;

	XmlRpc::XmlRpcValue object_list_;
	std::vector<ObjectNode> objects_;

	double RANGE_TH_;
};

#endif	// MAP_UPDATE_CHECKER_H_