#include "mcl/mcl.h"

int main(int argc,char** argv)
{
	ros::init(argc,argv,"mcl");
	MCL mcl;
	mcl.process();
	return 0;
}