#include "child_mcl/child_mcl.h"

ChildMCL::ChildMCL() :
    private_nh_("~"),
    engine_(seed_()),
    has_received_map_(false), has_received_obj_poses_(false), is_update_(false),
    count_(0), max_index_(0),
    x_cov_(0.5), y_cov_(0.5), yaw_cov_(0.5),
    likelihood_fast_(0.0), likelihood_slow_(0.0),
    likelihood_average_(0.0), likelihood_sum_(0.0), likelihood_max_(0.0),
    distance_sum_(0.0), angle_sum_(0.0)
{
    // parameter
    private_nh_.param("file_name",file_name_,{"arrange_1.csv"});
    private_nh_.param("obj_poses_topic_name",obj_poses_topic_name_,{"/object_positions"});
    private_nh_.param("estimated_pose_topic_name",estimated_pose_topic_name_,{"/pose_out"});
    private_nh_.param("estimated_poses_topic_name",estimated_poses_topic_name_,{"/poses_out"});
    private_nh_.param("map_topic_name",map_topic_name_,{"/map"});
    private_nh_.param("map_frame_id",map_frame_id_,{"map"});
    private_nh_.param("odom_frame_id",odom_frame_id_,{"odom"});
    private_nh_.param("base_link_frame_id",base_link_frame_id_,{"base_link"});

    // load_parameter
    private_nh_.param("HZ",HZ_,{10});
    private_nh_.param("NUM_OF_PARTICLES",NUM_OF_PARTICLES_,{2000});
    private_nh_.param("INIT_X",INIT_X_,{7.0});
    private_nh_.param("INIT_Y",INIT_Y_,{0.0});
    private_nh_.param("INIT_YAW",INIT_YAW_,{1.5708});
    private_nh_.param("INIT_X_COV",INIT_X_COV_,{0.5});
    private_nh_.param("INIT_Y_COV",INIT_Y_COV_,{0.5});
    private_nh_.param("INIT_YAW_COV",INIT_YAW_COV_,{0.5});
    private_nh_.param("MAX_RANGE",MAX_RANGE_,{25});
    private_nh_.param("RANGE_STEP",RANGE_STEP_,{5});
    private_nh_.param("X_COV_TH",X_COV_TH_,{0.15});
    private_nh_.param("Y_COV_TH",Y_COV_TH_,{0.15});
    private_nh_.param("YAW_COV_TH",YAW_COV_TH_,{0.10});
    private_nh_.param("ALPHA_1",ALPHA_1_,{0.3});
    private_nh_.param("ALPHA_2",ALPHA_2_,{0.3});
    private_nh_.param("ALPHA_3",ALPHA_3_,{0.1});
    private_nh_.param("ALPHA_4",ALPHA_4_,{0.1});
    private_nh_.param("ALPHA_SLOW",ALPHA_SLOW_,{0.001});
    private_nh_.param("ALPHA_FAST",ALPHA_FAST_,{0.1});
    private_nh_.param("HIT_COV",HIT_COV_,{1.0});
    private_nh_.param("LAMBDA_SHORT",LAMBDA_SHORT_,{0.1});
    private_nh_.param("Z_HIT",Z_HIT_,{0.95});
    private_nh_.param("Z_SHORT",Z_SHORT_,{0.1});
    private_nh_.param("Z_MAX",Z_MAX_,{0.05});
    private_nh_.param("Z_RAND",Z_RAND_,{0.05});
    private_nh_.param("DISTANCE_TH",DISTANCE_TH_,{0.15});
    private_nh_.param("ANGLE_TH",ANGLE_TH_,{0.15});
    private_nh_.param("SELECTION_RATE",SELECTION_RATE_,{0.2});

    private_nh_.param("DISTANCE_DEV_RATE",DISTANCE_DEV_RATE_,{0.14});
    private_nh_.param("DIRECTION_DEV",DIRECTION_DEV_,{0.05});

    map_sub_ = nh_.subscribe(map_topic_name_,HZ_,&ChildMCL::map_callback,this);
    obj_poses_sub_ = nh_.subscribe(obj_poses_topic_name_,HZ_,&ChildMCL::obj_poses_callback,this);
    estimated_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(estimated_pose_topic_name_,HZ_);
    estimated_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>(estimated_poses_topic_name_,HZ_);

    // tf
    broadcaster_.reset(new tf2_ros::TransformBroadcaster);
    buffer_.reset(new tf2_ros::Buffer);
    listener_.reset(new tf2_ros::TransformListener(*buffer_));

    // initialize
    initialize();

    // object parameter
    //load_parameter();
    //read_csv();
}

ChildMCL::~ChildMCL() { }

ChildMCL::Particle::Particle(ChildMCL* child_mcl) : child_mcl_(child_mcl)
{
    pose_.header.frame_id = child_mcl_->map_frame_id_;
    child_mcl_->set_pose(pose_,0.0,0.0,0.0);
    likelihood_ = 1.0/(double)child_mcl_->NUM_OF_PARTICLES_;
}

ChildMCL::Particle::~Particle() { }

void ChildMCL::Particle::init(double x,double y,double yaw,double x_cov,double y_cov,double yaw_cov)
{
    do{
        std::normal_distribution<> dist_x(x_cov);
        std::normal_distribution<> dist_y(y,y_cov);
        std::normal_distribution<> dist_yaw(yaw,yaw_cov);
        child_mcl_->set_pose(pose_,dist_x(child_mcl_->engine_),dist_y(child_mcl_->engine_),dist_yaw(child_mcl_->engine_));
    }
    while(child_mcl_->map_grid_data(pose_.pose.position.x,pose_.pose.position.y) != 0);
}

void ChildMCL::Particle::motion_update()
{
    double dx = child_mcl_->current_pose_.pose.position.x - child_mcl_->previous_pose_.pose.position.x;
    double dy = child_mcl_->current_pose_.pose.position.y - child_mcl_->previous_pose_.pose.position.y;
    double dyaw = child_mcl_->calc_angle_diff(child_mcl_->calc_yaw_from_msg(child_mcl_->current_pose_.pose.orientation),
                                              child_mcl_->calc_yaw_from_msg(child_mcl_->previous_pose_.pose.orientation));

    child_mcl_->distance_sum_ += std::sqrt(dx*dx + dy*dy);
    child_mcl_->angle_sum_ += std::fabs(dyaw);

    if(child_mcl_->distance_sum_ > child_mcl_->DISTANCE_TH_ || child_mcl_->angle_sum_ > child_mcl_->ANGLE_TH_){
        child_mcl_->distance_sum_ = 0.0;
        child_mcl_->angle_sum_ = 0.0;
        child_mcl_->is_update_ = true;
    }

    state_transition(dx,dy,dyaw);
}

void ChildMCL::Particle::state_transition(double dx,double dy,double dyaw)
{
    double yaw = child_mcl_->calc_yaw_from_msg(pose_.pose.orientation);

    double delta_trans = std::sqrt(dx*dx + dy*dy);
    double delta_rot1;
    if(delta_trans < 1e-2) delta_rot1 = 0.0;
    else delta_rot1 = dyaw;

    double delta_rot2  = child_mcl_->calc_angle_diff(dyaw,delta_rot1);

    double delta_rot1_noise = std::min(std::fabs(child_mcl_->calc_angle_diff(delta_rot1,0.0)),
                                       std::fabs(child_mcl_->calc_angle_diff(delta_rot1,M_PI)));
    double delta_rot2_noise = std::min(std::fabs(child_mcl_->calc_angle_diff(delta_rot2,0.0)),
                                       std::fabs(child_mcl_->calc_angle_diff(delta_rot2,M_PI)));

    std::normal_distribution<> dist_rot1(0.0,(child_mcl_->ALPHA_1_*delta_rot1_noise*delta_rot1_noise - child_mcl_->ALPHA_2_*delta_trans*delta_trans));
    std::normal_distribution<> dist_rot2(0.0,(child_mcl_->ALPHA_1_*delta_rot2_noise*delta_rot2_noise - child_mcl_->ALPHA_2_*delta_trans*delta_trans));
    std::normal_distribution<> dist_trans(0.0,(child_mcl_->ALPHA_3_*delta_trans*delta_trans + child_mcl_->ALPHA_4_*delta_rot1_noise*delta_rot1_noise + child_mcl_->ALPHA_4_*delta_rot2_noise*delta_rot2_noise));

    double delta_rot1_hat  = child_mcl_->calc_angle_diff(delta_rot1,dist_rot1(child_mcl_->engine_));
    double delta_rot2_hat  = child_mcl_->calc_angle_diff(delta_rot2,dist_rot2(child_mcl_->engine_));
    double delta_trans_hat = delta_trans - dist_trans(child_mcl_->engine_);

    child_mcl_->set_pose(pose_,
                         pose_.pose.position.x + delta_trans_hat * std::cos(yaw + delta_rot1_hat),
                         pose_.pose.position.y + delta_trans_hat * std::sin(yaw + delta_rot1_hat),
                         yaw + delta_rot1_hat + delta_rot2_hat);
}

void ChildMCL::Particle::measurement_update()
{
    std::vector<ObjectDistance> distances;
    for(auto&o : child_mcl_->obj_poses_.object_position){
        if(o.Class == "roomba") continue;
        if(o.Class == "elevator") continue;
        if(o.Class == "table") continue;

        double distance = std::sqrt(o.x*o.x + o.z*o.z);
        double theta = std::atan2(o.z,o.x) - 0.5*M_PI;
        ObjectDistance object_distance(o.Class,distance,theta);
        distances.push_back(object_distance);
    }

    if(distances.size() == 0) return;

    for(auto&d : distances){
        for(auto&m : child_mcl_->markers_.markers){
            if(d.name == m.ns){
                double distance = std::sqrt(std::pow(m.pose.position.x - pose_.pose.position.x,2) + std::pow(m.pose.position.y - pose_.pose.position.y,2));
                double error = std::sqrt(std::pow(distance - d.distance,2));
                ObjectError object_error(m.id,error);
                d.push_object_error(object_error);
            }
        }
        d.sort_object_error();
    }

    for(auto&d : distances){
        for(auto&m : child_mcl_->markers_.markers){
            if(d.obejct_errors[0].id == m.id  && d.obejct_errors[0].error < 2 ){
                double distance = std::sqrt(std::pow(m.pose.position.x - pose_.pose.position.x,2) + std::pow(m.pose.position.y - pose_.pose.position.y,2));
                double direction = std::atan2(m.pose.position.y - pose_.pose.position.y,m.pose.position.x - pose_.pose.position.x) - 0.5*M_PI;
                Eigen::Vector2d mean;
                mean(0) = distance;
                mean(1) = direction;
                Eigen::MatrixX2d cov;
                cov.setZero();
                cov(0,0) = std::pow(distance*child_mcl_->DISTANCE_DEV_RATE_,2);
                cov(1,1) = std::pow(child_mcl_->DIRECTION_DEV_,2);
                MultivariateNormal multivariate_normal(mean,cov);
                Eigen::Vector2d x;
                x(0) = d.distance;
                x(1) = d.theta;
                likelihood_ *= multivariate_normal.pdf(x);
            }
        }
    }
}

std::vector<std::string> ChildMCL::split(std::string& input,char delimiter)
{
	std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while(std::getline(stream,field,delimiter)) result.push_back(field);
    
    return result;
}

void ChildMCL::load_parameter()
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

void ChildMCL::read_csv()
{
    markers_.markers.clear();
	std::ifstream ifs_csv_file(dir_path_ + file_name_);
    std::string line;
    while(std::getline(ifs_csv_file,line)){
        std::vector<std::string> strvec = split(line,',');

        visualization_msgs::Marker marker;
        marker.header.frame_id = map_frame_id_;
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
                marker.color.a = 0.5f;
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
        
        markers_.markers.push_back(marker);
        count_ ++;
    }
}

void ChildMCL::map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    map_ = *msg;
    std::cout << "2" << std::endl;
    if(!has_received_map_){
        std::vector<Particle> init_particles;
        for(int i = 0; NUM_OF_PARTICLES_; i++){
            Particle particle(this);
            particle.init(INIT_X_,INIT_Y_,INIT_YAW_,INIT_X_COV_,INIT_Y_COV_,INIT_YAW_COV_);
            estimated_poses_.poses.push_back(particle.pose_.pose);
            init_particles.push_back(particle);
        }
        particles_ = init_particles;
    }
    std::cout << "a" << std::endl;
    has_received_map_ = true;
}

void ChildMCL::obj_poses_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg)
{
    obj_poses_ = *msg;
    has_received_obj_poses_ = true;
}

void ChildMCL::initialize()
{
    estimated_pose_.header.frame_id = map_frame_id_;
    estimated_poses_.header.frame_id = map_frame_id_;
    estimated_poses_.poses.clear();
    current_pose_.header.frame_id = map_frame_id_;
    set_pose(estimated_pose_,INIT_X_,INIT_Y_,INIT_YAW_);
    set_pose(current_pose_,0.0,0.0,0.0);
    previous_pose_ = current_pose_;
}

void ChildMCL::load_odometry(geometry_msgs::PoseStamped& odometry)
{
    geometry_msgs::TransformStamped transform_stamped;
    try{
        transform_stamped = buffer_->lookupTransform(odom_frame_id_,base_link_frame_id_,ros::Time(0));
    }
    catch(tf2::TransformException& ex){
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;

    tf2::doTransform(pose,odometry,transform_stamped);
}

void ChildMCL::set_pose(geometry_msgs::PoseStamped& pose,double x,double y,double yaw)
{
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;

    geometry_msgs::Quaternion msg_q;
    tf2::Quaternion tf_q;
    tf_q.setRPY(0.0,0.0,yaw);
    msg_q.x = (double)tf_q.x();
    msg_q.y = (double)tf_q.y();
    msg_q.z = (double)tf_q.z();
    msg_q.w = (double)tf_q.w();
    pose.pose.orientation = msg_q;
}

void ChildMCL::spread_particles()
{
    std::vector<Particle> reset_particles;
    for(int i = 0; i < NUM_OF_PARTICLES_; i++){
        Particle particle(this);
        particle.init(estimated_pose_.pose.position.x,
                      estimated_pose_.pose.position.y,
                      calc_yaw_from_msg(estimated_pose_.pose.orientation),
                      x_cov_,y_cov_,yaw_cov_);
        reset_particles.push_back(particle);
    }
    particles_ = reset_particles;
}

void ChildMCL::motion_update()
{
    for(int i = 0; i < NUM_OF_PARTICLES_; i++) particles_[i].motion_update();
}

void ChildMCL::measurement_update()
{
    likelihood_sum_ = 0.0;
    likelihood_average_ = 0.0;
    for(int i = 0; i < NUM_OF_PARTICLES_; i++){
        particles_[i].measurement_update();
        likelihood_sum_ += particles_[i].likelihood_;
    }
    likelihood_average_ = likelihood_sum_/(double)NUM_OF_PARTICLES_;
}

void ChildMCL::calc_likelihood()
{
    max_index_ = 0;
    for(int i = 0; i < NUM_OF_PARTICLES_; i++){
        if(particles_[i].likelihood_ > particles_[max_index_].likelihood_) max_index_ = i;
        particles_[i].likelihood_ /= likelihood_sum_;
    }
    
    if(likelihood_average_ ==  0 || std::isnan(likelihood_average_)){
        likelihood_average_ = 1.0/(double)(NUM_OF_PARTICLES_);
        likelihood_slow_ = likelihood_average_;
        likelihood_fast_ = likelihood_average_;
    }
    
    if(likelihood_slow_ == 0.0) likelihood_slow_ = likelihood_average_;
    else likelihood_slow_ += ALPHA_SLOW_*(likelihood_average_ - likelihood_slow_);
    
    if(likelihood_fast_ == 0.0) likelihood_fast_ = likelihood_average_;
    else likelihood_fast_ += ALPHA_FAST_*(likelihood_average_ - likelihood_fast_);
}

void ChildMCL::resampling()
{
    std::uniform_real_distribution<> dist(0.0,1.0);
    int index = (int)(dist(engine_)*NUM_OF_PARTICLES_);
    double beta = 0.0;
    double mv   = particles_[max_index_].likelihood_;
        
    std::vector<Particle> new_particles;
    double l;
    if((1 - likelihood_fast_/likelihood_slow_) > 0) l = 1 - likelihood_fast_/likelihood_slow_;
    else l = 0.0;

    for(int i = 0; i < NUM_OF_PARTICLES_; i++){
        if(l < dist(engine_)){
            beta += dist(engine_) * 2.0 * mv;
            while(beta > particles_[index].likelihood_){
                beta -= particles_[index].likelihood_;
                index = (index + 1)%NUM_OF_PARTICLES_;
            }
            new_particles.push_back(particles_[index]);
        }
        else{
            Particle particle(this);
            particle.init(estimated_pose_.pose.position.x,
                          estimated_pose_.pose.position.y,
                          calc_yaw_from_msg(estimated_pose_.pose.orientation),
                          INIT_X_COV_,INIT_Y_COV_,INIT_YAW_COV_);
            new_particles.push_back(particle);
        }
    }
    std::sort(new_particles.begin(),new_particles.end(),[](const Particle& a,const Particle& b) { return a.likelihood_ > b.likelihood_; });
    particles_ = new_particles;
}

void ChildMCL::calc_estimated_pose()
{
    double estimated_x = 0.0;
    double estimated_y = 0.0;
    double estimated_yaw = 0.0;

    int selected_N = (int)(SELECTION_RATE_*NUM_OF_PARTICLES_);
    for(int i = 0; i < selected_N; i++){
        estimated_x += particles_[i].pose_.pose.position.x;
        estimated_y += particles_[i].pose_.pose.position.y;
    }
    estimated_yaw = calc_yaw_from_msg(particles_[0].pose_.pose.orientation);

    set_pose(estimated_pose_,estimated_x/(double)selected_N,estimated_y/(double)selected_N,estimated_yaw);
}

void ChildMCL::calc_cov()
{
    double average_x = 0.0;
    double average_y = 0.0;
    double average_yaw = 0.0;
    for(int i = 0; i < NUM_OF_PARTICLES_; i++){
        estimated_poses_.poses[i] = particles_[i].pose_.pose;
        average_x += particles_[i].pose_.pose.position.x;
        average_y += particles_[i].pose_.pose.position.y;
        average_yaw += calc_yaw_from_msg(particles_[i].pose_.pose.orientation);
    }
    
    average_x /= (double)NUM_OF_PARTICLES_;
    average_y /= (double)NUM_OF_PARTICLES_;
    average_yaw /= (double)NUM_OF_PARTICLES_;

    double dev_x_sum = 0.0;
    double dev_y_sum = 0.0;
    double dev_yaw_sum = 0.0;
    for(int i = 0; i < NUM_OF_PARTICLES_; i++){
        dev_x_sum += std::pow((particles_[i].pose_.pose.position.x - average_x),2);
        dev_y_sum += std::pow((particles_[i].pose_.pose.position.y - average_y),2);
        dev_yaw_sum += std::pow((calc_yaw_from_msg(particles_[i].pose_.pose.orientation) - average_yaw),2);
    }
    
    x_cov_ = std::sqrt(dev_x_sum/(double)NUM_OF_PARTICLES_);
    y_cov_ = std::sqrt(dev_y_sum/(double)NUM_OF_PARTICLES_);
    yaw_cov_ = std::sqrt(dev_yaw_sum/(double)NUM_OF_PARTICLES_);
}

void ChildMCL::publish_estimated_pose()
{
    std::cout << "ESTIMATED_POSE" << std::endl;
    std::cout << "ESTIMATED_POSE.X   : " << estimated_pose_.pose.position.x << std::endl;
    std::cout << "ESTIMATED_POSE.Y   : " << estimated_pose_.pose.position.y << std::endl;
    std::cout << "ESTIMATED_POSE.YAW : " << calc_yaw_from_msg(estimated_pose_.pose.orientation) << std::endl;

    estimated_pose_pub_.publish(estimated_pose_);
    estimated_poses_pub_.publish(estimated_poses_);
}

void ChildMCL::publish_tf()
{
    try{
        tf2::Quaternion q;
        q.setRPY(0.0,0.0,calc_yaw_from_msg(estimated_pose_.pose.orientation));
        tf2::Transform map_transform(q,tf2::Vector3(estimated_pose_.pose.position.x,estimated_pose_.pose.position.y,0.0));
        geometry_msgs::PoseStamped tf_stamped;
        tf_stamped.header.frame_id = base_link_frame_id_;
        tf_stamped.header.stamp = obj_poses_.header.stamp;
        tf2::toMsg(map_transform.inverse(),tf_stamped.pose);
        geometry_msgs::PoseStamped odom_to_map;
        buffer_->transform(tf_stamped,odom_to_map,odom_frame_id_);
        tf2::Transform latest_tf;
        tf2::convert(odom_to_map.pose,latest_tf);
        geometry_msgs::TransformStamped tmp_tf_stamped;
        tmp_tf_stamped.header.stamp = obj_poses_.header.stamp;
        tmp_tf_stamped.header.frame_id = map_frame_id_;
        tmp_tf_stamped.child_frame_id = odom_frame_id_;
        tf2::convert(latest_tf.inverse(),tmp_tf_stamped.transform);
        broadcaster_->sendTransform(tmp_tf_stamped);
    }
    catch(tf2::TransformException& ex){
        ROS_WARN("%s", ex.what());
        return;
    }
}

int ChildMCL::map_grid_data(double x,double y)
{
    int index_x = floor((x - map_.info.origin.position.x)/map_.info.resolution);
    int index_y = floor((y - map_.info.origin.position.y)/map_.info.resolution);

    return (map_.data[(int)(index_x + index_y*map_.info.width)]);
}

double ChildMCL::calc_yaw_from_msg(geometry_msgs::Quaternion q)
{
    double roll, pitch, yaw;
    tf2::Quaternion tf_q(q.x,q.y,q.z,q.w);
    tf2::Matrix3x3(tf_q).getRPY(roll,pitch,yaw);
    
    return yaw;
}

double ChildMCL::calc_angle_diff(double a,double b)
{
    double a_angle = std::atan2(std::sin(a),std::cos(a));
    double b_angle = std::atan2(std::sin(b),std::cos(b));

    double d1 = a_angle - b_angle;
    double d2 = 2*M_PI - std::fabs(d1);

    if(d1 > 0) d2 *= -1;
    if(std::fabs(d1) < std::fabs(d2)) return d1;
    else return d2;
}

void ChildMCL::process()
{
    std::cout << "1" << std::endl;
    ros::Rate rate(HZ_);
    while(ros::ok()){
        if(has_received_map_ && has_received_obj_poses_){
            load_odometry(current_pose_);
            if(x_cov_ < X_COV_TH_ || y_cov_ < Y_COV_TH_ || yaw_cov_ < YAW_COV_TH_){
                x_cov_ = 0.2;
                y_cov_ = 0.2;
                yaw_cov_ = 0.2;
                spread_particles();
            }

            motion_update();
            if(has_received_obj_poses_){
                measurement_update();
                has_received_obj_poses_ = false;
            }
            calc_likelihood();

            if(is_update_){
                resampling();
                calc_estimated_pose();
                calc_cov();
                is_update_ = false;
            }
            
            publish_estimated_pose();
            publish_tf();
        }
        previous_pose_ = current_pose_;
        ros::spinOnce();
        rate.sleep();
    }

}