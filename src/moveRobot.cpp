#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "stdio.h"
#include "assert.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_datatypes.h>

#include "nav_msgs/Odometry.h"


#include <string>
#include <sstream>
#include <iostream>
#include <cmath>
// We know that the laser takes measurements from -180 to 180 degrees
// using 0.1 angles between measurements. Thus, we can specify a parameter
// to specify how wide our "no obstacles path" is. This will be used
// in computing the simple avoidance algorithm
#define ANGLE_STEPS_OFFSET 10

using namespace std;
using std::ostringstream;


ros::Publisher cmd_vel_pub; // publisher for movement commands
ros::Time start;
double vMaxDes, vMaxRot, kRotMin, kRotMax, oriError, critDist, dObj;

double currentX, currentY, currentOrientation;
double targetX, targetY;

double computeAngleDifference() { 
	double dx = targetX - currentX;
	double dy = targetY - currentY;

	double thetaObjective = atan2(dy, dx);

	double angleDifference = thetaObjective - currentOrientation;

	// Calculate the shortest rotation angle
	angleDifference = std::fmod(angleDifference + M_PI, 2.0 * M_PI) - M_PI;

	ROS_INFO("angleDiff=%lf, oriError=%lf", angleDifference, oriError);

	return angleDifference;
}


double computeRotation(double angleDifference) {
	double rotVel = 0;
	if (angleDifference < oriError) {
		rotVel = kRotMin * vMaxRot * angleDifference;
	} else {
		rotVel = kRotMax * vMaxRot * angleDifference;
	}

	ROS_INFO("rotVel=%lf", rotVel);
	return rotVel;
}

bool isObjectAhead(const sensor_msgs::LaserScan& most_intense) {
	int middle = most_intense.ranges.size() / 2;

	for (size_t i = middle - ANGLE_STEPS_OFFSET; i <= middle + ANGLE_STEPS_OFFSET; i++) {
		if (most_intense.ranges[i] < critDist)
			return true;
	}

	return false;
}

void callbackLaser(const sensor_msgs::LaserScan& most_intense) {	
	int length = most_intense.ranges.size(); 	
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

	bool goingToCollide = isObjectAhead(most_intense);

	if (goingToCollide) {
		cmd_vel.angular.z = 1.0;

		cmd_vel_pub.publish(cmd_vel);
		ros::spinOnce(); 
		return;
	}

	double angleDifference = computeAngleDifference();
	
	double rotation = computeRotation(angleDifference);
	cmd_vel.angular.z = rotation;

	if (std::abs(angleDifference) < oriError) {
		cmd_vel.linear.x = vMaxDes;
	}

	/*
    for(int i=0; i<length; i++) {
		bool left_side = i < (length / 2);

    	if(most_intense.ranges[i] < CRIT_DIST) {
			cmd_vel.linear.x = 0.0;
			if (left_side) {
				cmd_vel.angular.z = 1.0;
			} else {
				cmd_vel.angular.z = -1.0;
			}
			break;
    	}
    }
	*/
    cmd_vel_pub.publish(cmd_vel);
    ros::spinOnce(); 
}


/***
Print the odometry location.
***/
void callbackOdom(const nav_msgs::Odometry odom) {
	currentX = odom.pose.pose.position.x;
	currentY = odom.pose.pose.position.y;
	double current_z=odom.pose.pose.orientation.z;
    double current_w=odom.pose.pose.orientation.w;
	currentOrientation = 2.0 * atan2(current_z, current_w);
	ROS_INFO("Odom X=%lf, Y=%lf, ori=%lf\n", currentX, currentY, currentOrientation);
}

void callbackGoals(const geometry_msgs::Point& point) {
	targetX = point.x;
	targetY = point.x;
	ROS_INFO("x=%lf, y=%lf", point.x, point.y);
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "moveRobot");
	ros::Time::init();

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
  
	nh_private.param<double>("crit_dist", critDist, 0.8);
	nh_private.param<double>("d_obj", dObj, 0.2);
	nh_private.param<double>("v_max_des", vMaxDes, 0.8);
	nh_private.param<double>("v_max_rot", vMaxRot, 0.8);
	nh_private.param<double>("k_rot_min", kRotMin, 0.05);
	nh_private.param<double>("k_rot_max", kRotMax, 0.2);
	nh_private.param<double>("ori_error", oriError, 0.3);

	ros::Subscriber goals_sub = nh.subscribe("myGoals", 10, callbackGoals);


	//Build a string with the odom topic
    string odom_topic_name = "robot_";
	odom_topic_name += "0";
	odom_topic_name += "/odom";

	// subscribe to robot's odom topic "robot_X/base_scan"
	ros::Subscriber odom_sub = nh.subscribe(odom_topic_name, 10, callbackOdom);


     // subscribe to robot's laser scan topic "robot_X/base_scan"
	string sonar_scan_topic_name = "robot_";
	sonar_scan_topic_name += "0";
	sonar_scan_topic_name += "/base_scan_1";
	ros::Subscriber sub = nh.subscribe(sonar_scan_topic_name, 1, callbackLaser);

    string cmd_vel_topic_name = "robot_";
	cmd_vel_topic_name += "0";
	cmd_vel_topic_name += "/cmd_vel";
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);
    
	start = ros::Time::now();
	ros::spin();
}

