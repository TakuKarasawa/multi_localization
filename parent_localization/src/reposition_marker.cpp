#include "reposition_marker/reposition_marker.h"

RepositionMarker::RepositionMarker() :
    private_nh_("~"),
    count_(0), is_first_(true), first_time_(0.0)
{
    private_nh_.param("file_name",file_name_,{"record_3.csv"});
    private_nh_.param("marker_frame_id",marker_frame_id_,{"map"});
    private_nh_.param("is_debug",is_debug_,{false});

    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/markers",1);

    load_parameter();
    make_firsts();
}

RepositionMarker::~RepositionMarker() { }

void RepositionMarker::load_parameter()
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

void RepositionMarker::make_firsts()
{
    for(int i = 0; i < (int)object_list_.size(); i++) is_firsts_.push_back(true);
}

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

        if(is_first_){
            first_time_ = std::stod(strvec.at(0));
            is_first_ = false;
        }

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

        marker.pose.position.x = std::stod(strvec[2]);
        marker.pose.position.y = std::stod(strvec[3]);
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        bool is_color = false;
        for(int i = 0; i < (int)object_list_.size(); i++){
            if(strvec[1] == objects_[i].name){
                marker.color.r = objects_[i].r;
                marker.color.g = objects_[i].g;
                marker.color.b = objects_[i].b;
                marker.color.a = 1.0f;
                is_color = true;

                // push dataset
                Object object(std::stod(strvec[2]),std::stod(strvec[3]));
                if(is_firsts_[i]){
                    Objects objects(strvec[1]);
                    objects.push_object(object);
                    dataset_.push_objects(objects);
                    is_firsts_[i] = false;
                }
                else{
                    dataset_.push_object(strvec[1],object);
                }
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

void RepositionMarker::classfy()
{
    /*
    for(auto&l : dataset_.list_){
        if(l.is_first_){
            FilteredObjects filterd_object(l.reference_object_name());
            filterd_object.push_object(l.element_[0]);
            l.is_first_ = false;
        }
        else{
            for(auto&fo : dataset_.filtered_list_){
                if(l.reference_object_name() == fo.name_){
                    for(int i = 1; i < (int)l.element_.size(); i++){
                        double th = fo.calc_mahalanobis(l.element_[i].x,l.element_[i].y);
                        if(th <= 0.5) fo.push_object(l.element_[i]);
                        else{
                            FilteredObjects filtered_object(l.reference_object_name());
                            filtered_object.push_object(l.element_[i]);
                            dataset_.push_filtered_objects(filtered_object);
                        }
                    }
                }
            }
        }
    }
    */

   /*
    for(auto&l : dataset_.list_){
        for(int i = 0; i < (int)l.element_.size(); i++){
            if(l.is_first_){
                FilteredObjects filtered_object(l.name_);
                filtered_object.push_object(l.element_[i]);
                dataset_.push_filtered_objects(filtered_object);
                l.is_first_ = false;
            }
            else{
                for(auto&fo : dataset_.filtered_list_){
                    if(fo.name_ == l.name_){
                        fo.calc_average_cov();
                        std::cout << fo.name_ << std::endl;
                        std::cout << fo.calc_mahalanobis(l.element_[i].x,l.element_[i].y) << std::endl;
                        std::cout << std::endl;
                    }
                    //std::cout << fo.calc_mahalanobis(l.element_[i].x,l.element_[i].y) << std::endl;
                }
            }
        }
    }
    */

    for(int i = 0; i < (int)dataset_.list_.size(); i++){
        for(int j = 0; j < (int)dataset_.list_[i].element_.size(); j++){
            if(j == 0){
                FilteredObjects filtered_objecct(dataset_.list_[i].name_);
                filtered_objecct.push_object(dataset_.list_[i].element_[j]);
                dataset_.push_filtered_objects(filtered_objecct);
            }
            else{
                for(int k = 0; k < (int)dataset_.filtered_list_.size(); k++){
                    if(dataset_.filtered_list_[k].name_ == dataset_.list_[i].name_){
                        std::cout << "name: " << dataset_.filtered_list_[k].name_ << std::endl;
                        double s;
                        s = dataset_.filtered_list_[k].calc_mahalanobis(dataset_.list_[i].element_[j].x,dataset_.list_[i].element_[j].y);
                        if(s <= 1.5){
                            dataset_.filtered_list_[k].push_object(dataset_.list_[i].element_[j]);
                        }
                        else{
                            FilteredObjects filtered_object(dataset_.list_[i].name_);
                            filtered_object.push_object(dataset_.list_[i].element_[j]);
                            dataset_.push_filtered_objects(filtered_object);
                        }

                    }
                }
            }
        };
    }


    /*
    for(auto&l :dataset_.list_){
        std::cout << l.name_ << std::endl;
        std::cout << (int)l.element_.size() << std::endl;
        std::cout << std::endl;
    }
    */
    
    /*
    for(auto&fl : dataset_.filtered_list_){
        std::cout << fl.name_ << std::endl;
        for(auto&o : fl.objects_){
            std::cout << o.x << "," << o.y << std::endl;
        }
    }
    */
}

void RepositionMarker::process()
{
    visualization_msgs::MarkerArray markers;
    read_csv(markers);
    classfy();
    
    if(is_debug_){
        std::cout << "========== debug ==========" << std::endl;
        for(auto&l : dataset_.list_){
            std::cout << "object name: " << l.reference_object_name() << std::endl;
            for(auto&e : l.element_){
                std::cout << "(x,y): " << "(" << e.x << "," << e.y << ")" << std::endl;
            }
        }
        std::cout << std::endl;

        std::cout << "========== filter ==========" << std::endl;
        for(auto&fl : dataset_.filtered_list_){
            std::cout << "object name: " << fl.name_ << std::endl;
            for(auto&o : fl.objects_){
                std::cout << "(x,y): " << "(" << o.x << "," << o.y << ")" << std::endl;
            }
        }   
        std::cout << std::endl;

    }



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