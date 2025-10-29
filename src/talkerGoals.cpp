#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <iostream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talkerGoals");
	ros::NodeHandle node_obj;
	ros::NodeHandle nh("~");
	double t_goal;
	double x_goal;
	double y_goal;

	nh.param<double>("t_goal", t_goal, 3);
	nh.param<double>("x_goal", x_goal, 5);
	nh.param<double>("y_goal", y_goal, 6);

	ROS_INFO("x=%lf, y=%lf", x_goal, y_goal);

	double frequency = 1 / t_goal;

	ros::Rate loop_rate(frequency);
	ros::Publisher number_publisher = node_obj.advertise<geometry_msgs::Point>("myGoals",10);

	while (ros::ok())
	{
		geometry_msgs::Point point;
		point.x = x_goal;
		point.y = y_goal;
		number_publisher.publish(point);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
