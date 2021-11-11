#ifndef REPOSITION_MARKER_H_
#define REPOSITION_MARKER_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

#include <sstream>
#include <fstream>

struct ObjectNode
{
	ObjectNode(std::string name_,float r_,float g_,float b_) :
	name(name_), r(r_), g(g_), b(b_) {}

	std::string name;
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
    
    std::string reference_object_name(){
        return name_;
    }

private:
    std::string name_;
    std::vector<Object> element_;
};

class Dateset
{
public:
    Dateset()
    {
        
    }
    ~Dateset() {}

    void push_objects(Objects objects) { list_.push_back(objects); }

    void push_object(std::string object_name,Object object)
    {
        for(auto &obj : list_){
            if(obj.reference_object_name() == object_name){
                obj.push_object(object);
            }
        }
    }

    std::vector<Objects> list_;

};


class RepositionMarker
{
public:
    RepositionMarker();
    ~RepositionMarker();
    void process();

private:
    void load_parameter();
    void read_csv(visualization_msgs::MarkerArray& markers);
    void make_firsts();
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

    int object_num_;
    std::vector<bool> is_firsts_;

    Dateset dataset_;

    XmlRpc::XmlRpcValue object_list_;
    std::vector<ObjectNode> objects_;
};

#endif  // REPOSITION_MARKER_H_