#include "tf_broadcaster/tf_broadcaster.h"

int main(int argc,char** argv)
{
	ros::init(argc,argv,"tf_broadcaster");
	TFBroadcaster tf_broadcaster;
	tf_broadcaster.process();
	return 0;
}