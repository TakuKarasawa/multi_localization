#include "marker_updater/marker_updater.h"

MarkerUpdater::MarkerUpdater() :
	private_nh_("~"),
	count_(0), is_first_(true), first_time_(0.0)
{
	private_nh_.param("file_name",file_name_,{"record_3.csv"});
	private_nh_.param("raw_marker_frame_id",raw_marker_frame_id_,{"map"});

	raw_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/raw_markers",1);

    load_parameter();
}

MarkerUpdater::~MarkerUpdater() { }

std::vector<std::string> MarkerUpdater::split(std::string& input,char delimiter)
{
	std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while(std::getline(stream,field,delimiter)) result.push_back(field);
    
    return result;
}

void MarkerUpdater::load_parameter()
{
    if(!private_nh_.getParam("object_list",object_list_)){
        ROS_WARN("Could not load objects list");
        return;
    }
    ROS_ASSERT(object_list_.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i = 0; i < (int)object_list_.size(); i++){
        if(!object_list_[i]["name"].valid() || !object_list_[i]["id"].valid() || !object_list_[i]["r"].valid() || !object_list_[i]["g"].valid() || !object_list_[i]["b"].valid()){
            ROS_WARN("object_list is valid");
            return;
        }
        if(object_list_[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString && object_list_[i]["r"].getType() == XmlRpc::XmlRpcValue::TypeDouble && object_list_[i]["g"].getType() == XmlRpc::XmlRpcValue::TypeDouble && object_list_[i]["b"].getType() == XmlRpc::XmlRpcValue::TypeDouble){
            std::string name = static_cast<std::string>(object_list_[i]["name"]);
            double r = static_cast<double>(object_list_[i]["r"]);
            double g = static_cast<double>(object_list_[i]["g"]);
            double b = static_cast<double>(object_list_[i]["b"]);
            ObjectNode object_node(name,(float)r,(float)g,(float)b);
            objects_.push_back(object_node);
        }
    }
}

void MarkerUpdater::read_csv(visualization_msgs::MarkerArray& markers)
{
	std::ifstream ifs_csv_file(dir_path_ + file_name_);
    std::string line;
    while(std::getline(ifs_csv_file,line)){
        std::vector<std::string> strvec = split(line,',');

        if(is_first_){
            first_time_ = std::stod(strvec.at(0));
            is_first_ = false;
        }

        visualization_msgs::Marker marker;
        marker.header.frame_id = raw_marker_frame_id_;
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

        bool is_color = false;
        for(const auto&object : objects_){
            if(strvec.at(1) == object.name){
                marker.color.r = object.r;
                marker.color.g = object.g;
                marker.color.b = object.b;
                marker.color.a = 1.0f;
                is_color = true;
            }
            if(is_color) break;
        }

        if(!is_color){
            marker.color.r = 1.0f;
            marker.color.g = 1.0f;
            marker.color.b = 1.0f;
            marker.color.a = 1.0f;
        }
        
        markers.markers.push_back(marker);
        count_ ++;
    }
}

void MarkerUpdater::process()
{
	visualization_msgs::MarkerArray markers;
    read_csv(markers);
    ros::Rate rate(1);
    while(ros::ok()){
        raw_markers_pub_.publish(markers);
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"marker_updater");
	MarkerUpdater marker_updater;
	marker_updater.process();
	return 0;
}