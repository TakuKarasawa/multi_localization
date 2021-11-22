#include "child_ekf/child_ekf.h"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"child_ekf");
	ChildEKF child_ekf;
	child_ekf.process();
    return 0;
}