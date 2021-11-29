#include "child_ekf/child_ekf.h"

ChildEKF::ChildEKF() :
    private_nh_("~"),
    has_received_odom_(false), has_received_obj_(false),
    is_first_(true)
{
    private_nh_.param("odom_topic_name",odom_topic_name_,{"/roombaodom"});
    private_nh_.param("obj_topic_name",obj_topic_name_,{"/object/positions"});
    private_nh_.param("pose_topic_name",pose_topic_name_,{"/pose_out"});
    private_nh_.param("map_frame_id",map_frame_id_,{"map"});
    private_nh_.param("odom_frame_id",odom_frame_id_,{"odom"});
    private_nh_.param("base_link_frame_id",base_link_frame_id_,{"base_link"});
    private_nh_.param("is_odom_tf",is_odom_tf_,{false});

    private_nh_.param("HZ",HZ_,{10});
    private_nh_.param("INIT_X",INIT_X_,{0.0});
    private_nh_.param("INIT_Y",INIT_Y_,{0.0});
    private_nh_.param("INIT_YAW",INIT_YAW_,{0.0});
    private_nh_.param("MOTION_NOISE_NN",MOTION_NOISE_NN_,{0.0});
    private_nh_.param("MOTION_NOISE_NO",MOTION_NOISE_NO_,{0.0});
    private_nh_.param("MOTION_NOISE_ON",MOTION_NOISE_ON_,{0.0});
    private_nh_.param("MOTION_NOISE_OO",MOTION_NOISE_OO_,{0.0});
    private_nh_.param("DISTANCE_NOISE_RATE",DISTANCE_NOISE_RATE_,{0.0});
    private_nh_.param("DIRECTION_NOISE",DIRECTION_NOISE_,{0.0});
    
    broadcaster_.reset(new tf2_ros::TransformBroadcaster);
    buffer_.reset(new tf2_ros::Buffer);
    listener_.reset(new tf2_ros::TransformListener(*buffer_));

    odom_sub_ = nh_.subscribe(odom_topic_name_,1,&ChildEKF::odometry_callbak,this);
    obj_sub_ = nh_.subscribe(obj_topic_name_,1,&ChildEKF::object_positions_callback,this);

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pose_topic_name_,1);

    initialize(INIT_X_,INIT_Y_,INIT_YAW_);
}

ChildEKF::~ChildEKF() { }

void ChildEKF::odometry_callbak(const nav_msgs::OdometryConstPtr& msg)
{
    odom_ = *msg;

    if(is_odom_tf_){
        geometry_msgs::TransformStamped transform;
        transform.header = msg->header;
        transform.child_frame_id = msg->child_frame_id;

        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;
        transform.transform.rotation = msg->pose.pose.orientation;

        broadcaster_->sendTransform(transform);
    }

    has_received_odom_ = true;
}

void ChildEKF::object_positions_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg)
{
    objects_ = *msg;
    has_received_obj_ = true;
}

void ChildEKF::timer_callback(const ros::TimerEvent& event)
{

}

void ChildEKF::initialize(double x,double y,double yaw)
{
    MU_.setZero();
    set_pose(x,y,yaw);
    SIGMA_.setZero();
    SIGMA_ = 1e-10*Eigen::Matrix3d::Identity();
}

void ChildEKF::set_pose(double x,double y,double yaw)
{
    MU_(0) = x;
    MU_(1) = y;
    MU_(2) = yaw;
}

void ChildEKF::publish_pose()
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = map_frame_id_;
    pose.pose.position.x = MU_(0);
    pose.pose.position.y = MU_(1);
    pose.pose.position.z = 0.0;
    pose.pose.orientation = rpy_to_msg(0.0,0.0,MU_(2));

    std::cout << "POSE: " << std::endl;
    std::cout << " X  : " << MU_(0) << std::endl;
    std::cout << " Y  : " << MU_(1) << std::endl;
    std::cout << "YAW : " << MU_(2) << std::endl;
    std::cout << std::endl;

    pose_pub_.publish(pose);
}

void ChildEKF::motion_update(double dt)
{
    double nu = odom_.twist.twist.linear.x;
    double omega = odom_.twist.twist.angular.z;

    if(std::fabs(omega) < 1e-5) omega = 1e-10;

    // M
    Eigen::Matrix2d M;
    M.setZero();
    M(0,0) = std::pow(MOTION_NOISE_NN_,2)*std::fabs(nu)/dt + std::pow(MOTION_NOISE_NO_,2)*std::fabs(omega)/dt;
    M(1,1) = std::pow(MOTION_NOISE_ON_,2)*std::fabs(nu)/dt + std::pow(MOTION_NOISE_OO_,2)*std::fabs(omega)/dt;

    // A
    Eigen::Matrix<double,3,2> A;
    A.setZero();
    A(0,0) = (std::sin(MU_(2) + omega*dt) - std::sin(MU_(2)))/omega;
	A(0,1) = -nu/std::pow(omega,2)*(std::sin(MU_(2) + omega*dt) - std::sin(MU_(2))) + nu/omega*dt*std::cos(MU_(2) + omega*dt);
	A(1,0) = (-std::cos(MU_(2) + omega*dt) + std::cos(MU_(2)))/omega;
	A(1,1) = -nu/std::pow(omega,2)*(-std::cos(MU_(2) + omega*dt) + std::cos(MU_(2))) + nu/omega*dt*std::sin(MU_(2) + omega*dt);
	A(2,0) = 0.0;
	A(2,1) = dt;

    // G
    Eigen::Matrix3d G;
    G.setIdentity();
    G(0,2) = nu/omega*(std::cos(MU_(2) + omega*dt) - std::cos(MU_(2)));
	G(1,2) = nu/omega*(std::sin(MU_(2) + omega*dt) - std::sin(MU_(2)));

    // state transition
    if(std::fabs(omega) < 1e-2){
		MU_(0) += nu*std::cos(MU_(2))*dt;
		MU_(1) += nu*std::sin(MU_(2))*dt;
		MU_(2) += omega*dt;
	}
	else{
		MU_(0) += nu/omega*(std::sin(MU_(2) + omega*dt) - std::sin(MU_(2)));
		MU_(1) += nu/omega*(-std::cos(MU_(2) + omega*dt) + std::cos(MU_(2)));
		MU_(2) += omega*dt;
	}

    SIGMA_ = G*SIGMA_*G.transpose() + A*M*A.transpose();
}

void ChildEKF::measurement_update()
{
    // Q
    Eigen::Matrix2d Q;
    Q.setZero(0,0);

    Eigen::Vector3d Z;
    Z.setZero();


}

void ChildEKF::publish_tf()
{
    try{
        tf2::Quaternion q;
        q.setRPY(0.0,0.0,MU_(2));
        tf2::Transform map_transform(q,tf2::Vector3(MU_(0),MU_(1),0.0));

        geometry_msgs::PoseStamped tf_stamped;
        tf_stamped.header.frame_id = base_link_frame_id_;
        tf_stamped.header.stamp = odom_.header.stamp;
        tf2::toMsg(map_transform.inverse(),tf_stamped.pose);
        geometry_msgs::PoseStamped odom_to_map;
        buffer_->transform(tf_stamped,odom_to_map,odom_frame_id_);

        tf2::Transform latest_tf;
        tf2::convert(odom_to_map.pose,latest_tf);
        geometry_msgs::TransformStamped tmp_tf_stamped;
        tmp_tf_stamped.header.stamp = odom_.header.stamp;
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

geometry_msgs::Quaternion ChildEKF::rpy_to_msg(double roll,double pitch,double yaw)
{
    geometry_msgs::Quaternion msg;
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll,pitch,yaw);
    msg.x = quaternion.x();
    msg.y = quaternion.y();
    msg.z = quaternion.z();
    msg.w = quaternion.w();

    return msg;
}


void ChildEKF::process()
{
    ros::Rate rate(HZ_);
    double dt;
    now_time_ = ros::Time::now();
    while(ros::ok()){
        if(has_received_odom_){
            now_time_ = ros::Time::now();
            if(is_first_){
                dt = 1.0/(double)HZ_;
                is_first_ = false;
            }
            else dt = now_time_.toSec() - last_time_.toSec();

            motion_update(dt);
            if(has_received_obj_) measurement_update();
            publish_pose();
            publish_tf();

            has_received_obj_ = false;
        }
        else std::cout << "waiting msg" << std::endl;
        last_time_ = now_time_;
        ros::spinOnce();
        rate.sleep();
    }   
}