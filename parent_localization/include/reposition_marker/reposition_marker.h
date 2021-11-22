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
    Object(double x_,double y_) :
    x(x_), y(y_) {}

    double x;
    double y;
};

class Objects
{
public:
    Objects(std::string name) :
        name_(name), is_first_(true) { }

    ~Objects() { }

    void push_object(Object object) { element_.push_back(object); }    
    std::string reference_object_name() { return name_; }

    std::string name_;
    bool is_first_;
    std::vector<Object> element_;

};

class FilteredObjects
{
public:
    FilteredObjects(std::string name) : 
        name_(name),
        average_x_(0.0), average_y_(0.0)
    {
        X_.setZero();
        Sigma_.setZero();
    }
    ~FilteredObjects() {}

    void push_object(Object object) { objects_.push_back(object); }

    void set_X(double x,double y)
    {
        X_(0) = x - average_x_;
        X_(1) = y - average_y_;
    }

    void calc_average_cov()
    {
        double sum_x = 0.0;
        double sum_y = 0.0;
        for(int i = 0; i < (int)objects_.size(); i++){
            sum_x += objects_[i].x;
            sum_y += objects_[i].y;
        }
        average_x_ = sum_x/(double)objects_.size();
        average_y_ = sum_y/(double)objects_.size();

        double rss_x = 0.0;
        double rss_y = 0.0;
        double rss_xy = 0.0;
        for(int i = 0; i < (int)objects_.size(); i++){
            rss_x += std::pow(objects_[i].x - average_x_,2);
            rss_y += std::pow(objects_[i].y - average_y_,2);
            rss_xy += (objects_[i].x - average_x_)*(objects_[i].y - average_y_);
        }
        rss_x /= (double)objects_.size();
        rss_y /= (double)objects_.size();
        rss_xy /= (double)objects_.size();

        Sigma_(0,0) = rss_x;
        Sigma_(0,1) = rss_xy;
        Sigma_(1,0) = rss_xy;
        Sigma_(1,1) = rss_xy;
    }

    void calc_average(double x,double y,double& average_x,double& average_y)
    {
        double sum_x = 0.0;
        double sum_y = 0.0;

        for(int i = 0; i < (int)objects_.size(); i++){
            sum_x += objects_[i].x;
            sum_y += objects_[i].y;
        }

        average_x_ = (sum_x + x)/(double)(objects_.size() + 1);
        average_y_ = (sum_y + y)/(double)(objects_.size() + 1);
    }

    void calc_cov(double x,double y,double average_x,double average_y,Eigen::Matrix2d& sigma)
    {
        double rss_x = 0.0;
        double rss_y = 0.0;
        double rss_xy = 0.0;
        
        for(int i = 0; i < (int)objects_.size(); i++){
            rss_x += std::pow(objects_[i].x - average_x,2);
            rss_y += std::pow(objects_[i].y - average_y,2);
            rss_xy += (objects_[i].x - average_x)*(objects_[i].y - average_y);
        }
        rss_x += std::pow(x - average_x,2);
        rss_y += std::pow(y - average_y,2);
        rss_xy +- (x - average_x)*(y - average_y);

        //std::cout << rss_x/(double)(objects_.size() + 1) << std::endl;
        //std::cout << rss_y << std::endl;
        //std::cout << rss_xy << std::endl;
        
        sigma(0,0) = rss_x/(double)(objects_.size() + 1);
        sigma(0,1) = rss_xy/(double)(objects_.size() + 1);
        sigma(1,0) = rss_xy/(double)(objects_.size() + 1);
        sigma(1,1) = rss_xy/(double)(objects_.size() + 1);

    }

    double calc_mahalanobis(double x,double y)
    {
        //calc_average_cov();
        //set_X(x,y);

        //double similality = X_.transpose()*Sigma_.inverse()*X_;
    
        double ave_x, ave_y;
        Eigen::Matrix2d sigma;
        Eigen::Vector2d state;
        sigma.setZero();
        state.setZero();
        calc_average(x,y,ave_x,ave_y);
        calc_cov(x,y,ave_x,ave_y,sigma);

        state(0) = x - ave_x;
        state(1) = y - ave_y;

        //std::cout << state << std::endl;
        //std::cout << sigma << std::endl;
        
        double similality;
        similality = state.transpose()*sigma.inverse()*state;

        std::cout << "similality: " << similality << std::endl;

        return std::sqrt(similality);
    }

    std::string name_;
    std::vector<Object> objects_;

    double average_x_;
    double average_y_;
    Eigen::Vector2d X_;
    Eigen::Matrix2d Sigma_;

};

class Dateset
{
public:
    Dateset() { }
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

    void push_filtered_objects(FilteredObjects objects) { filtered_list_.push_back(objects); }


    std::vector<Objects> list_;
    std::vector<FilteredObjects> filtered_list_;
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
    void classfy();
    std::vector<std::string> split(std::string& input,char delimiter);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher markers_pub_;

    int count_;
    bool is_first_;
    bool is_debug_;
    double first_time_;

    std::string dir_path_ = "/home/amsl/catkin_ws/src/multi_localization/parent_localization/record/";
    std::string file_name_;
    std::string marker_frame_id_;

    std::vector<bool> is_firsts_;

    Dateset dataset_;

    XmlRpc::XmlRpcValue object_list_;
    std::vector<ObjectNode> objects_;
};

#endif  // REPOSITION_MARKER_H_