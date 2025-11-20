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
#include "project1/robot.hpp"


#include <string>
#include <sstream>
#include <iostream>
#include <cmath>

using namespace std;
using std::ostringstream;

ros::Publisher cmd_vel_pub; 

MUSIRobot::Robot robot;

double tObs;

void callbackLaser(const sensor_msgs::LaserScan& most_intense) {	
	geometry_msgs::Twist result = robot.run(most_intense);
	cmd_vel_pub.publish(result);
	ros::spinOnce();
}

/***
Print the odometry location.
***/
void callbackOdom(const nav_msgs::Odometry odom) {
	MUSIRobot::Odom newRobotOdom;
	newRobotOdom.currentX = odom.pose.pose.position.x;
	newRobotOdom.currentY = odom.pose.pose.position.y;

	double current_z = odom.pose.pose.orientation.z;
    double current_w = odom.pose.pose.orientation.w;
	newRobotOdom.currentOrientation = 2.0 * atan2(current_z, current_w);

	robot.setOdom(newRobotOdom);
	//ROS_INFO("Odom X=%lf, Y=%lf, ori=%lf\n", currentX, currentY, currentOrientation);
}

void callbackGoals(const geometry_msgs::Point& point) {
	robot.setTarget(point.x, point.y);
}

void callbackLeader(const nav_msgs::Odometry odom) {
	robot.setTarget(odom.pose.pose.position.x, odom.pose.pose.position.y);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "moveRobot");
	ros::Time::init();

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	int idRobot, chosenAlgorithm;
	double critDist, dObj, vMaxDes, vMaxRot, kRotMin, kRotMax, oriError, tAvoidObs;

	double w1, w2;
	int timeToWait;

	int role, idLeader;
	double distLeader;
  
	nh_private.param<int>("id_robot", idRobot, 0);
	nh_private.param<int>("algor", chosenAlgorithm, 1);
	nh_private.param<double>("crit_dist", critDist, 0.8);
	nh_private.param<double>("d_obj", dObj, 0.2);
	nh_private.param<double>("v_max_des", vMaxDes, 0.8);
	nh_private.param<double>("v_max_rot", vMaxRot, 0.8);
	nh_private.param<double>("k_rot_min", kRotMin, 0.05);
	nh_private.param<double>("k_rot_max", kRotMax, 0.2);
	nh_private.param<double>("ori_error", oriError, 0.3);
	nh_private.param<double>("t_avoid_obs", tAvoidObs, 1);

	nh_private.param<double>("w_1", w1, 1);
	nh_private.param<double>("w_2", w2, 2.5);
	nh_private.param<int>("t_wait", timeToWait, 100);

	nh_private.param<double>("dist_leader", distLeader, 1.5);
	nh_private.param<int>("robot_rol", role, 0);
	nh_private.param<int>("id_leader", idLeader, 0);

	MUSIRobot::Params params;
	params.criticalDistance = critDist;
	params.distanceToObjective = dObj;
	params.permittedOrientationError = oriError;
	params.kMinimumRotation = kRotMin;
	params.kMaximumRotation = kRotMax;
	params.vMaximumRotation = vMaxRot;
	params.vMaximumDisplacement = vMaxDes;
	params.timeToAvoidObstacle = tAvoidObs;

	params.w1 = w1;
	params.w2 = w2;
	params.timeToWait = timeToWait;

	params.distLeader = distLeader;

	MUSIRobot::Algorithm selectedAlgorithm = MUSIRobot::Algorithm::SimpleAvoidance;
	if (chosenAlgorithm == 2) {
		selectedAlgorithm = MUSIRobot::Algorithm::PotentialFields;
	}

	MUSIRobot::Role selectedRole = MUSIRobot::Role::Leader;
	if (role != 0) {
		selectedRole = MUSIRobot::Role::Follower;
		ROS_INFO("Setting role Follower to robot id %d", idRobot);
	}

	tf::TransformListener listener(ros::Duration(10.0));
	robot.init(idRobot, selectedAlgorithm, selectedRole, params, &listener);

	string robotName = "robot_" + std::to_string(idRobot);

	ROS_INFO("I'm robot with name %s", robotName.c_str());

	//Build a string with the odom topic
    string odom_topic_name = robotName + "/odom";

	ROS_INFO("Subscribing to odom %s", odom_topic_name.c_str());

	ros::Subscriber odom_sub = nh.subscribe(odom_topic_name, 10, callbackOdom);

	ros::Subscriber goals_sub;

	if (selectedRole == MUSIRobot::Role::Leader) {
		string goalTopic = "/myGoals";
		goals_sub = nh.subscribe(goalTopic, 10, callbackGoals);
		ROS_INFO("Subscribing to target %s", goalTopic.c_str());
	} else if (selectedRole == MUSIRobot::Role::Follower) {
		string leaderName = "robot_" + std::to_string(idLeader);
		string goalTopic = leaderName + "/odom";
		goals_sub = nh.subscribe(goalTopic, 10, callbackLeader);
		ROS_INFO("Subscribing to target %s", goalTopic.c_str());
	}


     // subscribe to robot's laser scan topic "robot_X/base_scan"
	string sonar_scan_topic_name = robotName + "/base_scan_1";
	ros::Subscriber sub = nh.subscribe(sonar_scan_topic_name, 1, callbackLaser);
	ROS_INFO("Subscribing to base_scan %s", sonar_scan_topic_name.c_str());


    string cmd_vel_topic_name = robotName + "/cmd_vel";
	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);
	ROS_INFO("Advertising to %s", cmd_vel_topic_name.c_str());
    
	ros::spin();
}
