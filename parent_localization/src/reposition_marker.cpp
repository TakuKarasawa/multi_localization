#include "reposition_marker/reposition_marker.h"

RepositionMarker::RepositionMarker() :
    private_nh_("~"),
    count_(0)
{
    private_nh_.param("file_name",file_name_,{"record_3.csv"});
    private_nh_.param("marker_frame_id",marker_frame_id_,{"map"});

    // marker color
    private_nh_.param("trash_can_r",trash_can_.r,{0.0});
    private_nh_.param("trash_can_g",trash_can_.g,{1.0});
    private_nh_.param("trash_can_b",trash_can_.b,{0.0});
    private_nh_.param("fire_hydrant_r",fire_hydrant_.r,{0.0});
    private_nh_.param("fire_hydrant_g",fire_hydrant_.g,{0.0});
    private_nh_.param("fire_hydrant_b",fire_hydrant_.b,{1.0});
    private_nh_.param("bench_r",bench_.r,{1.0});
    private_nh_.param("bench_g",bench_.g,{0.0});
    private_nh_.param("bench_b",bench_.b,{0.0});
    private_nh_.param("big_bench_r",big_bench_.r,{1.0});
    private_nh_.param("big_bench_g",big_bench_.g,{1.0});
    private_nh_.param("big_bench_b",big_bench_.b,{0.0});
    private_nh_.param("fire_extinguisher_r",fire_extinguisher_.r,{0.0});
    private_nh_.param("fire_extinguisher_g",fire_extinguisher_.g,{1.0});
    private_nh_.param("fire_extinguisher_b",fire_extinguisher_.b,{1.0});
    private_nh_.param("kitchenette_icon_r",kitchenette_icon_.r,{1.0});
    private_nh_.param("kitchenette_icon_g",kitchenette_icon_.g,{0.0});
    private_nh_.param("kitchenette_icon_b",kitchenette_icon_.b,{1.0});
    private_nh_.param("toilet_icon_r",toilet_icon_.r,{0.0});
    private_nh_.param("toilet_icon_g",toilet_icon_.g,{0.0});
    private_nh_.param("toilet_icon_b",toilet_icon_.b,{0.0});
    private_nh_.param("chair_r",chair_.r,{1.0});
    private_nh_.param("chair_g",chair_.g,{0.5});
    private_nh_.param("chair_b",chair_.b,{0.5});

    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/markers",1);
}

RepositionMarker::~RepositionMarker() { }

std::vector<std::string> RepositionMarker::split(std::string& input,char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while(std::getline(stream,field,delimiter)) result.push_back(field);
    
    return result;
}

void RepositionMarker::read_csv(visualization_msgs::MarkerArray& markers)
{
    std::ifstream ifs_csv_file(dir_path_ + file_name_);
    std::string line;
    while(std::getline(ifs_csv_file,line)){
        std::vector<std::string> strvec = split(line,',');

        visualization_msgs::Marker marker;
        marker.header.frame_id = marker_frame_id_;
        marker.ns = strvec.at(1);
        marker.id = count_;

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
            marker.color.r = trash_can_.r;
            marker.color.g = trash_can_.g;
            marker.color.b = trash_can_.b;
            marker.color.a = 1.0f;
        }
        else if(strvec.at(1) == "fire_hydrant"){
            marker.color.r = fire_hydrant_.r;
            marker.color.g = fire_hydrant_.g;
            marker.color.b = fire_hydrant_.b;
            marker.color.a = 1.0f;
        }
        else if(strvec.at(1) == "bench"){
            marker.color.r = bench_.r;
            marker.color.g = bench_.g;
            marker.color.b = bench_.b;
            marker.color.a = 1.0f;
        }
        else if(strvec.at(1) == "big_bench"){
            marker.color.r = big_bench_.r;
            marker.color.g = big_bench_.g;
            marker.color.b = big_bench_.b;
            marker.color.a = 1.0f;
        }
        else if(strvec.at(1) == "fire_extinguisher"){
            marker.color.r = fire_extinguisher_.r;
            marker.color.g = fire_extinguisher_.g;
            marker.color.b = fire_extinguisher_.b;
            marker.color.a = 1.0f;
        }
        else if(strvec.at(1) == "kitchenette_icon"){
            marker.color.r = kitchenette_icon_.r;
            marker.color.g = kitchenette_icon_.g;
            marker.color.b = kitchenette_icon_.b;
            marker.color.a = 1.0f;
        }
        else if(strvec.at(1) == "toilet_icon"){
            marker.color.r = toilet_icon_.r;
            marker.color.g = toilet_icon_.g;
            marker.color.b = toilet_icon_.b;
            marker.color.a = 1.0f;
        }
        else if(strvec.at(1) == "chair"){
            marker.color.r = chair_.r;
            marker.color.g = chair_.g;
            marker.color.b = chair_.b;
            marker.color.a = 1.0f;
        }
        else{
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0f;
        }
        markers.markers.push_back(marker);
        count_ ++;
    }
}


void RepositionMarker::process()
{
    visualization_msgs::MarkerArray markers;
    read_csv(markers);
    ros::Rate rate(1);
    while(ros::ok()){
        markers_pub_.publish(markers);
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"reposition_marker");
    RepositionMarker reposition_marker;
    reposition_marker.process();
    return 0;
}