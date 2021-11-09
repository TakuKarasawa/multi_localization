#ifndef MARKER_UPDATER_H_
#define MARKER_UPDATER_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>

#include <sstream>
#include <fstream>

struct MarkerColor
{
    float r;
    float g;
    float b;
};

class MarkerUpdater
{
public:
	MarkerUpdater();
	~MarkerUpdater();
	void process();

private:
	void read_csv(visualization_msgs::MarkerArray& markers);
    std::vector<std::string> split(std::string& input,char delimiter);

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;
	ros::Publisher raw_markers_pub_;

	int count_;
	double first_time_;
	bool is_first_;

	std::string dir_path_ = "/home/amsl/catkin_ws/src/multi_localization/parent_localization/record/";
    std::string file_name_;
	std::string raw_marker_frame_id_;
	
	// marker color
    MarkerColor trash_can_;
    MarkerColor fire_hydrant_;
    MarkerColor bench_;
    MarkerColor big_bench_;
    MarkerColor fire_extinguisher_;
    MarkerColor kitchenette_icon_; 
    MarkerColor toilet_icon_;
    MarkerColor chair_; 
};

#endif	// MARKER_UPDATER_H_