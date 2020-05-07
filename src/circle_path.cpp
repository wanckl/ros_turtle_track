#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "circle_path");
	ros::NodeHandle nd;

    ros::Publisher circle_pub = nd.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    ros::Rate loop_rate(4);
    float count = 0.5;
    
    geometry_msgs::Twist path_point;
    
    while(ros::ok())
    {
        path_point.linear.x = 2;
        path_point.angular.z = 2*sin(count);
        count += 0.5;
        
        circle_pub.publish(path_point);
        ROS_INFO("Published : [%.2f m/s, %.2f rad/s]", \
                    path_point.linear.x, path_point.angular.z);
        
        loop_rate.sleep();
    }
    
    return 0;
}
