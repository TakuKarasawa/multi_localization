#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>

#include <sstream>
#include <fstream>

int count = 0;
double dist_th = 0.5;
bool is_first_trash;
bool is_first_hydrant;
bool is_first_bench;
bool is_first_big_bench;
bool is_first_fire_extinguisher;
bool is_first_kitchenette_icon;
bool is_first_toilet_icon;
bool is_first_chair;

class Object
{
public:
    Object() { }
    ~Object() { }

    void set_obj(double x,double y)
    {
        x_.push_back(x);
        y_.push_back(y);
    }

    void rm_obj()
    {
        x_.pop_back();
        y_.pop_back();

    }

    double get_x_mean()
    {
        double x_sum = 0;
        for(int i = 0; i < x_.size(); i++){
            x_sum += x_.at(i);
        }

        return x_sum/(double)x_.size();
    }

    double get_y_mean()
    {
        double y_sum = 0;
        for(int i = 0; i < y_.size(); i++){
            y_sum += y_.at(i);
        }

        return y_sum/(double)y_.size();
    }

    double get_x_cov()
    {
        double x_cov = 0;
        for(int i = 0; i < x_.size(); i++){
            x_cov += (x_.at(i) - get_x_mean())*(x_.at(i) - get_x_mean());
        }

        return std::sqrt(x_cov/(double)x_.size());
    }

    double get_y_cov()
    {
        double y_cov = 0;
        for(int i = 0; i < y_.size(); i++){
            y_cov += (y_.at(i) - get_y_mean())*(y_.at(i) - get_y_mean());
        }

        return std::sqrt(y_cov/(double)y_.size());
    }

    double get_xy_cov()
    {
        double xy_cov = 0;
        for(int i = 0; i < x_.size(); i++){
            xy_cov += (x_.at(i) - get_x_mean())*(y_.at(i) - get_y_mean());
        }

        return xy_cov/(double)x_.size();
    }

    double output_dis(double x,double y)
    {
        Eigen::Vector2d X;
        X(0) = x;
        X(1) = y;

        Eigen::Vector2d MU;
        MU(0) = get_x_mean();
        MU(1) = get_y_mean();

        Eigen::Matrix2d SIGMA;
        SIGMA(0,0) = get_x_cov();
        SIGMA(0,1) = get_xy_cov();
        SIGMA(1,0) = get_xy_cov();
        SIGMA(1,1) = get_y_cov();

        double d = std::sqrt((X - MU).transpose()*SIGMA.inverse()*(X - MU));
    }

private:

    std::vector<double> x_;
    std::vector<double> y_;

};

class Objects
{
public:
    Objects() { }
    ~Objects() { }

    void push_object(Object object)
    {
        objects_.push_back(object);
    }

    std::vector<Object> objects_;

private:
};

std::vector<std::string> split(std::string& input,char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while(std::getline(stream,field,delimiter)){
        result.push_back(field);
    }

    return result;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"set_marker");
    ros::NodeHandle nh;
    ros::Publisher markers_pub = nh.advertise<visualization_msgs::MarkerArray>("/markers",1);
    std::ifstream ifs_csv_file("/home/amsl/catkin_ws/src/multi_localization/parent_localization/record/record_3.csv");

    visualization_msgs::MarkerArray markers;
    std::string line;
    Objects objects;
    while(std::getline(ifs_csv_file,line)){
        std::vector<std::string> strvec = split(line,',');

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        ros::Time::now();
        marker.ns = strvec.at(1);
        marker.id = count;

        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration();

        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.6;

        marker.pose.position.x = std::stod(strvec.at(2));
        marker.pose.position.y = std::stod(strvec.at(3));
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        if(strvec.at(1) == "trash_can"){
            if(is_first_trash){
                Object trash_can;
                trash_can.set_obj(marker.pose.position.x,marker.pose.position.y);
                objects.push_object(trash_can);
                is_first_trash = false;
            }

            for(const auto&obj : objects.objects_){

            }


            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
        }
        else if(strvec.at(1) == "fire_hydrant"){
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0f;
        }
        else if(strvec.at(1) == "bench"){
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
        }
        else if(strvec.at(1) == "big_bench"){
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
        }
        else if(strvec.at(1) == "fire_extinguisher"){
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0f;
        }
        else if(strvec.at(1) == "kitchenette_icon"){
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0f;
        }
        else if(strvec.at(1) == "toilet_icon"){
            marker.color.r = 0.0f;
            marker.color.g = 0.5f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;
        }
        else if(strvec.at(1) == "chair"){
            marker.color.r = 1.0f;
            marker.color.g = 0.5f;
            marker.color.b = 0.5f;
            marker.color.a = 1.0f;
        }
        else{
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0f;
        }
        markers.markers.push_back(marker);
        count ++;
    }

    ros::Rate rate(1);
    while(ros::ok()){
        markers_pub.publish(markers);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}