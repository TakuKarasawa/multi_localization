#ifndef REPOSITION_MARKER_H_
#define REPOSITION_MARKER_H_

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

struct Object
{
    double x;
    double y;
};

class Objects
{
public:
    Objects(std::string name) :
        name_(name) { }

    ~Objects() { }

    void push_object(Object object) { element_.push_back(object); }

    std::string name_;
    std::vector<Object> element_;
};

class Dateset
{
public:
    Dateset()
    {
        
    }
    ~Dateset();



    std::vector<Objects> list_;

};


class RepositionMarker
{
public:
    RepositionMarker();
    ~RepositionMarker();
    void process();

private:
    void read_csv(visualization_msgs::MarkerArray& markers);
    std::vector<std::string> split(std::string& input,char delimiter);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher markers_pub_;

    int count_;

    std::string dir_path_ = "/home/amsl/catkin_ws/src/multi_localization/parent_localization/record/";
    std::string file_name_;
    std::string marker_frame_id_;
    bool is_first_;
    double first_time_;

    Dateset dataset_;

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

#endif  // REPOSITION_MARKER_H_