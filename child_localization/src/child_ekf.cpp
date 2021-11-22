#include "child_ekf/child_ekf.h"

ChildEKF::ChildEKF() :
    private_nh_("~")
{
    private_nh_.param("odom_topic_name",odom_topic_name_,{"/odom"});
}

ChildEKF::~ChildEKF() { }

void ChildEKF::odometry_callbak(const nav_msgs::OdometryConstPtr& msg)
{

}

void ChildEKF::object_positions_callback(const object_detector_msgs::ObjectPositionsConstPtr& msg)
{

}

void ChildEKF::timer_callback(const ros::TimerEvent& event)
{


}


void ChildEKF::process()
{

}