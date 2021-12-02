#include "child_mcl/child_mcl.h"

int main(int argc,char** argv)
{
	ros::init(argc,argv,"child_mcl");
	ChildMCL child_mcl;
	child_mcl.process();
	return 0;
}