#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>

void tracker_rcv(const turtlesim::Pose::ConstPtr& data);
void self_pose_rcv(const turtlesim::Pose::ConstPtr& data);

static const float half_pi = 1.5707963;
turtlesim::Pose turtle1_pose, turtle2_pose;
bool pos1_ok = 0, pos2_ok = 0;

int main(int argc, char **argv)
{
    float angle;
    
    ros::init(argc, argv, "track_path");
    ros::NodeHandle node;
    
    ros::Subscriber track_sour = node.subscribe("/turtle1/pose", 10, tracker_rcv);
    ros::Subscriber self_pos = node.subscribe("/turtle2/pose", 10, self_pose_rcv);
    ros::Publisher track_pub = node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
    
    ros::Rate loop_rate(10);
    geometry_msgs::Twist aim;
    
    while(ros::ok())
    {
        ros::spinOnce();
        if(pos1_ok)
        {
            if(pos2_ok)
            {
                pos1_ok = 0;
                pos2_ok = 0;
                if((turtle1_pose.y-turtle2_pose.y)>0)   //turtle1 on the 1st or 2nd graph area of turtle2
                {
                    if((turtle1_pose.x-turtle2_pose.x)>0)
                    {
                        //turtle1 on the 1st graph area of turtle2
                        angle = 0*half_pi + atan((turtle1_pose.y-turtle2_pose.y)/(turtle1_pose.x-turtle2_pose.x));
                    }
                    else
                    {
                        //turtle1 on the 2nd graph area of turtle2
                        angle = 1*half_pi + atan((turtle2_pose.x-turtle1_pose.x)/(turtle1_pose.y-turtle2_pose.y));
                    }
                }
                else                                    //turtle1 on the 3rd or 4th graph area of turtle2
                {
                    if((turtle1_pose.x-turtle2_pose.x)<0)
                    {
                        //turtle1 on the 3rd graph area of turtle2
                        angle = 2*half_pi + atan((turtle2_pose.y-turtle1_pose.y)/(turtle2_pose.x-turtle1_pose.x));
                    }
                    else
                    {
                        //turtle1 on the 4th graph area of turtle2
                        angle = 3*half_pi + atan((turtle1_pose.x-turtle2_pose.x)/(turtle2_pose.y-turtle1_pose.y));
                    }
                }
                aim.linear.x = sqrt(pow(turtle1_pose.x-turtle2_pose.x, 2) + pow(turtle1_pose.y-turtle2_pose.y, 2));
                aim.angular.z = angle - turtle2_pose.theta;
                if(2*half_pi<aim.angular.z)
                    aim.angular.z = aim.angular.z - 4*half_pi;
                else if(-2*half_pi>aim.angular.z)
                    aim.angular.z = 4*half_pi + aim.angular.z;
                if(aim.linear.x<0.1)
                    aim.linear.x = 0;
                track_pub.publish(aim);
                ROS_INFO("calc:%.2f\tpath:%.2f", aim.angular.z, aim.linear.x);
                loop_rate.sleep();
            }
        }
    }
    
    return 0;
}


void tracker_rcv(const turtlesim::Pose::ConstPtr& data)
{
    pos1_ok = 1;
    turtle1_pose.x = data->x;
    turtle1_pose.y = data->y;
    turtle1_pose.theta = data->theta;
//    ROS_INFO("turtle1_pose.x=%.2f, turtle1_pose.y=%.2f", data->x, data->y);
}

void self_pose_rcv(const turtlesim::Pose::ConstPtr& data)
{
    pos2_ok = 1;
    turtle2_pose.x = data->x;
    turtle2_pose.y = data->y;
    turtle2_pose.theta = data->theta;
//    ROS_INFO("turtle2_pose.x=%.2f, turtle2_pose.y=%.2f", data->x, data->y);
}
