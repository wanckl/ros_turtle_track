#include <ros/ros.h>
#include <turtlesim/Spawn.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "little_turtle");
	ros::NodeHandle node;
	ros::service::waitForService("/spawn");
	ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("/spawn");

	turtlesim::Spawn head;
	head.request.x = 2.0;
	head.request.y = 2.0;
	head.request.name = "turtle2";

	ROS_INFO("Calling to service \"/spawn\" with [x:%.2f, y:%.2f, naem:%s]", \
		head.request.x, head.request.y, head.request.name.c_str());
	add_turtle.call(head);
	ROS_INFO("call \"/spawn\" returned 200:%s", head.response.name.c_str());

	return 0;
}
