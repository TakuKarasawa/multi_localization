#include "mcl/mcl.h"

std::random_device seed;
std::mt19937 engine(seed());

MCL::MCL() : 
    private_nh_("~"),
    has_received_map_(false), is_update_(false),
    x_cov_(0.5), y_cov_(0.5), yaw_cov_(0.5),
    likelihood_fast_(0.0), likelihood_slow_(0.0),
    likelihood_average_(0.0), likelihood_sum_(0.0), likelihood_max_(0.0),
    distance_sum_(0.0), angle_sum_(0.0), max_index_(0) 
{
    private_nh_.param("scan_topic_name",scan_topic_name_,{"/scan"});
    private_nh_.param("map_topic_name",map_topic_name_,{"/map"});
    private_nh_.param("est_pose_topic_name",est_pose_topic_name_,{"est_pose"});
    private_nh_.param("est_poses_topic_name",est_poses_topic_name_,{"est_poses"});
    private_nh_.param("map_frame_id",map_frame_id_,{"map"});
    private_nh_.param("odom_frame_id",odom_frame_id_,{"odom"});
    private_nh_.param("base_link_frame_id",base_link_frame_id_,{"base_link"});

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

    map_sub_ = nh_.subscribe(map_topic_name_,1,&MCL::map_callback,this);
    scan_sub_ = nh_.subscribe(scan_topic_name_,1,&MCL::scan_callback,this);
    timer_ = nh_.createTimer(ros::Duration(1),&MCL::timer_callback,this);

    est_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(est_pose_topic_name_,1);
    est_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>(est_poses_topic_name_,1);

    // initialize
    est_pose_.header.frame_id = map_frame_id_;
    est_poses_.header.frame_id = map_frame_id_;
    current_pose_.header.frame_id = map_frame_id_;
    set_pose(est_pose_,INIT_X_,INIT_Y_,INIT_YAW_);
    set_pose(current_pose_,0.0,0.0,0.0);
    previous_pose_ = current_pose_;

    broadcaster_.reset(new tf2_ros::TransformBroadcaster);
    buffer_.reset(new tf2_ros::Buffer);
    listener_.reset(new tf2_ros::TransformListener(*buffer_));
}

MCL::~MCL() {}

MCL::Particle::Particle(MCL* mcl) : m_(mcl)
{
    pose_.header.frame_id = m_->map_frame_id_;
    m_->set_pose(pose_,0.0,0.0,0.0);
    likelihood_ = 1.0/(double)m_->NUM_OF_PARTICLES_;
}

MCL::Particle::~Particle() { }

void MCL::Particle::init(double x,double y,double yaw,double x_cov,double y_cov,double yaw_cov)
{
    do{
        std::normal_distribution<> dist_x(x,x_cov);
        std::normal_distribution<> dist_y(y,y_cov);
        std::normal_distribution<> dist_yaw(yaw,yaw_cov);
        m_->set_pose(pose_,dist_x(engine),dist_y(engine),dist_yaw(engine));
    }while(m_->map_grid_data(pose_.pose.position.x,pose_.pose.position.y) != 0);   
}

void MCL::Particle::motion_update()
{
    double dx = m_->current_pose_.pose.position.x - m_->previous_pose_.pose.position.x;
    double dy = m_->current_pose_.pose.position.y - m_->previous_pose_.pose.position.y;
    double dyaw = m_->calc_angle_diff(m_->calc_yaw_from_quat(m_->current_pose_.pose.orientation),m_->calc_yaw_from_quat(m_->previous_pose_.pose.orientation));

    m_->distance_sum_ += std::sqrt(dx*dx + dy*dy);
    m_->angle_sum_ += std::fabs(dyaw);

    if(m_->distance_sum_ > m_->DISTANCE_TH_ || m_->angle_sum_ > m_->ANGLE_TH_){
        m_->distance_sum_ = 0.0;
        m_->angle_sum_ = 0.0;
        m_->is_update_ = true;
    }

    move(dx,dy,dyaw);
}

void MCL::Particle::move(double dx,double dy,double dyaw)
{
    double delta_rot1;
    double yaw = m_->calc_yaw_from_quat(pose_.pose.orientation);

    double delta_trans = std::sqrt(dx*dx + dy*dy);
    if(delta_trans < 1e-2) delta_rot1 = 0.0;
    else delta_rot1 = dyaw;

    double delta_rot2  = m_->calc_angle_diff(dyaw,delta_rot1);

    double delta_rot1_noise = std::min(std::fabs(m_->calc_angle_diff(delta_rot1,0.0)),std::fabs(m_->calc_angle_diff(delta_rot1,M_PI)));
    double delta_rot2_noise = std::min(std::fabs(m_->calc_angle_diff(delta_rot2,0.0)),std::fabs(m_->calc_angle_diff(delta_rot2,M_PI)));

    std::normal_distribution<> dist_rot1(0.0,(m_->ALPHA_1_*delta_rot1_noise*delta_rot1_noise - m_->ALPHA_2_*delta_trans*delta_trans));
    std::normal_distribution<> dist_rot2(0.0,(m_->ALPHA_1_*delta_rot2_noise*delta_rot2_noise - m_->ALPHA_2_*delta_trans*delta_trans));
    std::normal_distribution<> dist_trans(0.0,(m_->ALPHA_3_*delta_trans*delta_trans + m_->ALPHA_4_*delta_rot1_noise*delta_rot1_noise + m_->ALPHA_4_*delta_rot2_noise*delta_rot2_noise));

    double delta_rot1_hat  = m_->calc_angle_diff(delta_rot1,dist_rot1(engine));
    double delta_rot2_hat  = m_->calc_angle_diff(delta_rot2,dist_rot2(engine));
    double delta_trans_hat = delta_trans - dist_trans(engine);

    m_->set_pose(pose_,
                 pose_.pose.position.x + delta_trans_hat * std::cos(yaw + delta_rot1_hat),
                 pose_.pose.position.y + delta_trans_hat * std::sin(yaw + delta_rot1_hat),
                 yaw + delta_rot1_hat + delta_rot2_hat);

    /*
    pose_.pose.position.x += delta_trans_hat * cos(yaw + delta_rot1_hat);
    pose_.pose.position.y += delta_trans_hat * sin(yaw + delta_rot1_hat);
    quaternionTFToMsg(tf::createQuaternionFromYaw(yaw + delta_rot1_hat + delta_rot2_hat),pose_.pose.orientation);
    */
}

void MCL::Particle::measurement_update()
{
    double angle;
    double map_range;
    double range_diff = 0.0;
    double p_w = 0.0;

    for(int i = 0; i < (int)(m_->scan_.ranges.size()); i += m_->RANGE_STEP_){
        angle = m_->scan_.angle_min + i*m_->scan_.angle_increment;
        map_range = m_->calc_range(pose_.pose.position.x,pose_.pose.position.y,m_->calc_yaw_from_quat(pose_.pose.orientation)+ angle);
        range_diff = m_->scan_.ranges[i] - map_range;

        if(m_->scan_.ranges[i] < m_->MAX_RANGE_) p_w += m_->Z_HIT_ * std::exp(-range_diff*range_diff)/(2*m_->HIT_COV_*m_->HIT_COV_);
        if(range_diff < 0) p_w += m_->Z_SHORT_ * m_->LAMBDA_SHORT_ * std::exp(-m_->LAMBDA_SHORT_*m_->scan_.ranges[i]);
        if(m_->scan_.ranges[i] >= m_->MAX_RANGE_) p_w += m_->Z_MAX_;
        if(m_->scan_.ranges[i] < m_->MAX_RANGE_) p_w += m_->Z_RAND_/m_->MAX_RANGE_;
        }
        likelihood_ = p_w;
}

void MCL::map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    map_ = *msg;
    if(!has_received_map_){
        std::cout << "Has received map" << std::endl;
        std::vector<Particle> init_particles;
        for(int i = 0; i < NUM_OF_PARTICLES_; i++){
            Particle p(this);
            p.init(INIT_X_,INIT_Y_,INIT_YAW_,INIT_X_COV_,INIT_Y_COV_,INIT_YAW_COV_);
            est_poses_.poses.push_back(p.pose_.pose);
            init_particles.push_back(p);
        }
        particles_ = init_particles;
    }
    has_received_map_ = true;
}

void MCL::scan_callback(const sensor_msgs::LaserScanConstPtr& msg) { scan_ = *msg; }

void MCL::timer_callback(const ros::TimerEvent& event)
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

    tf2::doTransform(pose,current_pose_,transform_stamped);
}

void MCL::set_pose(geometry_msgs::PoseStamped& pose,double x,double y,double yaw)
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

void MCL::spread_particles()
{
    std::vector<Particle> reset_particles;
    for(int i = 0; i < NUM_OF_PARTICLES_; i++){
        Particle p(this);
        p.init(est_pose_.pose.position.x,est_pose_.pose.position.y,calc_yaw_from_quat(est_pose_.pose.orientation),x_cov_,y_cov_,yaw_cov_);
        reset_particles.push_back(p);
    }
    particles_ = reset_particles;
}

void MCL::motion_process()
{
    for(int i = 0; i < NUM_OF_PARTICLES_; i++){
        particles_[i].motion_update();
    }
}

void MCL::measurement_process()
{
    likelihood_sum_ = 0.0;
    likelihood_average_ = 0.0;
    for(int i = 0; i < NUM_OF_PARTICLES_; i++){
        particles_[i].measurement_update();
        likelihood_sum_ += particles_[i].likelihood_;
    }
    likelihood_average_ = likelihood_sum_/(double)NUM_OF_PARTICLES_;
}

void MCL::calc_likelihood()
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

void MCL::resampling()
{
    std::uniform_real_distribution<> dist(0.0,1.0);
    int index = (int)(dist(engine)*NUM_OF_PARTICLES_);
    double beta = 0.0;
    double mv   = particles_[max_index_].likelihood_;
        
    std::vector<Particle> new_particles;
    double l;
    if((1 - likelihood_fast_/likelihood_slow_) > 0) l = 1 - likelihood_fast_/likelihood_slow_;
    else l = 0.0;

    for(int i = 0; i < NUM_OF_PARTICLES_; i++){
        if(l < dist(engine)){
            beta += dist(engine) * 2.0 * mv;
            while(beta > particles_[index].likelihood_){
                beta -= particles_[index].likelihood_;
                index = (index + 1)%NUM_OF_PARTICLES_;
            }
            new_particles.push_back(particles_[index]);
        }
        else{
            Particle p(this);
            p.init(est_pose_.pose.position.x,est_pose_.pose.position.y,calc_yaw_from_quat(est_pose_.pose.orientation),INIT_X_COV_,INIT_Y_COV_,INIT_YAW_COV_);
            new_particles.push_back(p);
        }
    }
    sort(new_particles.begin(),new_particles.end(),[](const Particle& a,const Particle& b) { return a.likelihood_ > b.likelihood_; });
    particles_ = new_particles;
}

void MCL::calc_est_pose()
{
    double est_pose_x = 0.0;
    double est_pose_y = 0.0;
    double est_pose_yaw = 0.0;

    int selected_N = (int)(SELECTION_RATE_*NUM_OF_PARTICLES_);
    for(int i = 0; i < selected_N; i++){
        est_pose_x += particles_[i].pose_.pose.position.x;
        est_pose_y += particles_[i].pose_.pose.position.y;
    }
    est_pose_yaw = calc_yaw_from_quat(particles_[0].pose_.pose.orientation);

    set_pose(est_pose_,est_pose_x/(double)selected_N,est_pose_y/(double)selected_N,est_pose_yaw);
}

void MCL::calc_cov()
{
    double ave_x = 0.0;
    double ave_y = 0.0;
    double ave_yaw = 0.0;
    for(int i = 0; i < NUM_OF_PARTICLES_; i++){
        est_poses_.poses[i] = particles_[i].pose_.pose;
        ave_x += particles_[i].pose_.pose.position.x;
        ave_y += particles_[i].pose_.pose.position.y;
        ave_yaw += calc_yaw_from_quat(particles_[i].pose_.pose.orientation);
    }
    
    ave_x /= (double)NUM_OF_PARTICLES_;
    ave_y /= (double)NUM_OF_PARTICLES_;
    ave_yaw /= (double)NUM_OF_PARTICLES_;

    double dev_x_sum = 0.0;
    double dev_y_sum = 0.0;
    double dev_yaw_sum = 0.0;
    for(int i = 0; i < NUM_OF_PARTICLES_; i++){
        dev_x_sum += std::pow((particles_[i].pose_.pose.position.x - ave_x),2);
        dev_y_sum += std::pow((particles_[i].pose_.pose.position.y - ave_y),2);
        dev_yaw_sum += std::pow((calc_yaw_from_quat(particles_[i].pose_.pose.orientation) - ave_yaw),2);
    }
    
    x_cov_ = std::sqrt(dev_x_sum/(double)NUM_OF_PARTICLES_);
    y_cov_ = std::sqrt(dev_y_sum/(double)NUM_OF_PARTICLES_);
    yaw_cov_ = std::sqrt(dev_yaw_sum/(double)NUM_OF_PARTICLES_);
}

void MCL::publish_est_pose()
{
    std::cout << "ESTIMATED_POSE" << std::endl;
    std::cout << "ESTIMATED_POSE.X   : " << est_pose_.pose.position.x << std::endl;
    std::cout << "ESTIMATED_POSE.Y   : " << est_pose_.pose.position.y << std::endl;
    std::cout << "ESTIMATED_POSE.YAW : " << calc_yaw_from_quat(est_pose_.pose.orientation) << std::endl;

    est_pose_pub_.publish(est_pose_);
    est_poses_pub_.publish(est_poses_);
}

void MCL::publish_tf()
{
    try{
        tf2::Quaternion q;
        q.setRPY(0.0,0.0,calc_yaw_from_quat(est_pose_.pose.orientation));
        tf2::Transform map_transform(q,tf2::Vector3(est_pose_.pose.position.x,est_pose_.pose.position.y,0.0));
        geometry_msgs::PoseStamped tf_stamped;
        tf_stamped.header.frame_id = base_link_frame_id_;
        tf_stamped.header.stamp = scan_.header.stamp;
        tf2::toMsg(map_transform.inverse(),tf_stamped.pose);
        geometry_msgs::PoseStamped odom_to_map;
        buffer_->transform(tf_stamped,odom_to_map,odom_frame_id_);
        tf2::Transform latest_tf;
        tf2::convert(odom_to_map.pose,latest_tf);
        geometry_msgs::TransformStamped tmp_tf_stamped;
        tmp_tf_stamped.header.stamp = scan_.header.stamp;
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


int MCL::map_grid_data(double x,double y)
{
    int index_x = floor((x - map_.info.origin.position.x)/map_.info.resolution);
    int index_y = floor((y - map_.info.origin.position.y)/map_.info.resolution);

    return (map_.data[(int)(index_x + index_y*map_.info.width)]);
}

double MCL::calc_yaw_from_quat(geometry_msgs::Quaternion q)
{
    double roll, pitch, yaw;
    tf2::Quaternion tf_q(q.x,q.y,q.z,q.w);
    tf2::Matrix3x3(tf_q).getRPY(roll,pitch,yaw);
    
    return yaw;
}

double MCL::calc_angle_diff(double a,double b)
{
    double a_angle = std::atan2(std::sin(a),std::cos(a));
    double b_angle = std::atan2(std::sin(b),std::cos(b));

    double d1 = a_angle - b_angle;
    double d2 = 2*M_PI - std::fabs(d1);

    if(d1 > 0) d2 *= -1;
    if(std::fabs(d1) < std::fabs(d2)) return d1;
    else return d2;
}

double MCL::calc_range(double x,double y,double yaw)
{
    int cx_0 = (x - map_.info.origin.position.x)/map_.info.resolution;
    int cy_0 = (y - map_.info.origin.position.y)/map_.info.resolution;
    int cx_1 = (x + MAX_RANGE_*std::cos(yaw) - map_.info.origin.position.x)/map_.info.resolution;
    int cy_1 = (y + MAX_RANGE_*std::sin(yaw) - map_.info.origin.position.y)/map_.info.resolution;

    bool judge = false;
    if(std::fabs(cx_1 - cx_0) < std::fabs(cy_1 - cy_0)) judge = true;

    if(judge){
        int tmp;
        tmp  = cx_1;
        cx_1 = cy_1;
        cy_1 = tmp;

        tmp  = cx_0;
        cx_0 = cy_0;
        cy_0 = tmp;
    }

    int dx = std::fabs(cx_1 - cx_0);
    int dy = std::fabs(cy_1 - cy_0);

    int cx = cx_0;
    int cy = cy_0;
    int error = 0;

    int xstep, ystep;
    if(cx_1 > cx_0) xstep = 1;
    else xstep = -1;

    if(cy_1 > cy_0) ystep = 1;
    else ystep = -1;

    if(judge){
        if(cy < 0 || cy > (int)map_.info.width || cx < 0 || cx > (int)map_.info.height || map_.data[cx*(int)map_.info.width + cy] != 0){
            return std::sqrt(std::pow((cx - cx_0),2) + std::pow((cy - cy_0),2)) * map_.info.resolution;
        }
    }
    else{
        if(cx < 0 || cx > (int)map_.info.width || cy < 0 || cy > (int)map_.info.height || map_.data[cy*(int)map_.info.width + cx] != 0){
            return std::sqrt(std::pow((cx - cx_0),2) + std::pow((cy - cy_0),2)) * map_.info.resolution;
        }
    }

    while(cx != (cx_1 + xstep)){
        cx += xstep;
        error += dy;
        if(2*error >= dx){
            cy += ystep;
            error -= dx;
        }
        
        if(judge){
            if(cy < 0 || cy > (int)map_.info.width || cx < 0 || cx > (int)map_.info.height || map_.data[cx*(int)map_.info.width + cy] != 0){
                return std::sqrt(std::pow((cx - cx_0),2) + std::pow((cy - cy_0),2)) * map_.info.resolution;
            }
        }
        else{
            if(cx < 0 || cx > (int)map_.info.width || cy < 0 || cy > (int)map_.info.height || map_.data[cy*(int)map_.info.width + cx] != 0){
                return std::sqrt(std::pow((cx - cx_0),2) + std::pow((cy - cy_0),2)) * map_.info.resolution;
            }
        }
    }
    
    return MAX_RANGE_;
}

void MCL::process()
{
    ros::Rate rate(1);
    while(ros::ok()){
        if(has_received_map_ && !scan_.ranges.empty()){
            if(x_cov_ < X_COV_TH_ || y_cov_ < Y_COV_TH_ || yaw_cov_ < YAW_COV_TH_){
                x_cov_ = 0.3;
                y_cov_ = 0.3;
                yaw_cov_ = 0.3;
                spread_particles();
            }
            
            motion_process();
            measurement_process();
            calc_likelihood();

            if(is_update_){
                resampling();
                calc_est_pose();
                calc_cov();
            }
            is_update_ = false;
            
            publish_est_pose();
            publish_tf();
        }
        previous_pose_ = current_pose_;
        ros::spinOnce();
        rate.sleep();
    }
}