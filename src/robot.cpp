#include <string>
#include "stdio.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <project1/robot.hpp>

namespace MUSIRobot {
Robot::Robot() {}

void Robot::init(int id, Algorithm algorithm, Role role, Params params, tf::TransformListener *listener) {
    this->id = id;
    this->selectedAlgorithm = algorithm;
    this->currentState = Normal;
    this->params = params;
	this->listener = listener;
	this->role = role;
}

geometry_msgs::Twist Robot::run(const sensor_msgs::LaserScan& laserData) {
    if (this->selectedAlgorithm == SimpleAvoidance) {
        return runSimpleAvoidance(laserData);
    }

    return runPotentialFields(laserData);
}

bool Robot::isObjectAhead(const sensor_msgs::LaserScan& laserData) {
	int middle = laserData.ranges.size() / 2;

	for (size_t i = middle - ANGLE_STEPS_OFFSET; i <= middle + ANGLE_STEPS_OFFSET; i++) {
		if (laserData.ranges[i] < params.criticalDistance)
			return true;
	}

	return false;
}

double Robot::computeAngleDifference() { 
	double dx = targetX - currentOdom.currentX;
	double dy = targetY - currentOdom.currentY;

	double thetaObjective = atan2(dy, dx);

	double angleDifference = thetaObjective - currentOdom.currentOrientation;

	// Calculate the shortest rotation angle
	angleDifference = std::fmod(angleDifference + M_PI, 2.0 * M_PI) - M_PI;

	//ROS_INFO("angleDiff=%lf, oriError=%lf", angleDifference, oriError);

	return angleDifference;
}

double Robot::computeRotation(double angleDifference) {
	double rotVel = 0;
	if (angleDifference < params.permittedOrientationError) {
		rotVel = params.kMinimumRotation * params.vMaximumRotation * angleDifference;
	} else {
		rotVel = params.kMaximumRotation * params.vMaximumRotation * angleDifference;
	}

	return rotVel;
}

bool Robot::reachedObjective() {
	double dx = targetX - currentOdom.currentX;
	double dy = targetY - currentOdom.currentY;
	double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

	if (role == Follower && distance <= params.distLeader) {
		ROS_INFO("id: %d (Follower), I've reached objective", id);
		return true;
	}

	if (role == Leader && distance <= params.distanceToObjective) {
		ROS_INFO("id: %d (Leader), I've reached objective", id);
		return true;
	}

	return false;
}

geometry_msgs::Twist Robot::runSimpleAvoidance(const sensor_msgs::LaserScan& laserData) {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

	bool objectAhead = isObjectAhead(laserData);

	if (currentState == Normal && objectAhead) {
		currentState = RotatingToAvoidObstacle;

		return cmd_vel;
	} 
	
	if (currentState == Normal && !objectAhead)  {
		double dx = targetX - currentOdom.currentX;
		double dy = targetY - currentOdom.currentY;
		double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));

		// Reached objective
		if (reachedObjective()) {
			return cmd_vel;
		}

		double angleDifference = computeAngleDifference();
		
		double rotation = computeRotation(angleDifference);
		cmd_vel.angular.z = rotation;

		if (std::abs(angleDifference) < params.permittedOrientationError) {
			cmd_vel.linear.x = params.vMaximumDisplacement;
		}

		return cmd_vel;
	} 
	
	if (currentState == RotatingToAvoidObstacle) {
		cmd_vel.angular.z = 1.0;

		if (!objectAhead) {
			cmd_vel.angular.z = 0.0;
			currentState = AvoidingObstacle;

			lastAvoidTimestamp = ros::Time::now().toSec();
		}

		return cmd_vel;
	} 
	
	if (currentState == AvoidingObstacle) {
		cmd_vel.linear.x = params.vMaximumDisplacement;
		double timeAvoiding = ros::Time::now().toSec() - lastAvoidTimestamp;

		// ROS_INFO("tAvoidObs=%lf",tAvoidObs);

		if (timeAvoiding >= params.timeToAvoidObstacle) {
			currentState = Normal;
		}

		return cmd_vel;
	}
}

geometry_msgs::Twist Robot::runPotentialFields(const sensor_msgs::LaserScan& laserData) {
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x = 0;
	cmd_vel.linear.y = 0;
	cmd_vel.linear.z = 0;
	cmd_vel.angular.x = 0;
	cmd_vel.angular.y = 0;
	cmd_vel.angular.z = 0;

	if (reachedObjective()) {
		return cmd_vel;
	}

	if (currentState == ExecutingPotentialFieldsDecision) {
		double tWait = params.timeToWait / 1000.0;

		double now = ros::Time::now().toSec();

		if ((now - lastPotentialFieldsDecisionTimestamp) < tWait) {
			return lastPotentialFieldsDecision;
		}

		currentState = Normal;
	}

	tf::Vector3 vObj = generateObjectiveVector();
	tf::Vector3 vObs = generateObstacleVectors(laserData);

	tf::Vector3 result = params.w1 * vObj + params.w2 * vObs;

	double angleToTarget = std::atan2(result.y(), result.x());
	angleToTarget = angleToTarget - currentOdom.currentOrientation;

	angleToTarget = std::fmod(angleToTarget + M_PI, 2.0 * M_PI) - M_PI;

	double magnitude = std::sqrt(
		std::pow(result.x(), 2) +  std::pow(result.y(), 2)
	);


	double rotationVelocity = computeRotation(angleToTarget);
	cmd_vel.angular.z = rotationVelocity;


	if (std::abs(angleToTarget) < params.permittedOrientationError) {
		cmd_vel.linear.x = std::min(magnitude, params.vMaximumDisplacement);
	} else {
		cmd_vel.linear.x = 0;
	}

	lastPotentialFieldsDecision = cmd_vel;
	currentState = ExecutingPotentialFieldsDecision;
	lastAvoidTimestamp = ros::Time::now().toSec();

	return cmd_vel;
}

tf::Vector3 Robot::generateObjectiveVector() {
	double x = targetX - currentOdom.currentX;
	double y = targetY - currentOdom.currentY;

	double magnitude = std::sqrt(std::pow(x, 2) + std::pow(y, 2));

	x /= magnitude;
	y /= magnitude;

	tf::Vector3 vector(x, y, 0);

	return vector;
}

tf::Vector3 Robot::generateObstacleVectors(const sensor_msgs::LaserScan& laserData) {
	std::vector<tf::Vector3> vectors;
	double angleIncrement = laserData.angle_increment;

	for (size_t i = 0; i < laserData.ranges.size(); i++) {
		double reading = laserData.ranges[i];

		if (reading > laserData.range_max) continue;
		if (reading < 0) continue;
		if (reading >= params.criticalDistance) continue;

		tf::Vector3 vector = generateObstacleVector(i, angleIncrement,laserData.angle_min, reading);
		vectors.push_back(vector);
	}

	tf::Vector3 result;
	for (const auto& vec : vectors) {
		result += vec;
	}

	return transformToOdomFrame(result);
}

tf::Vector3 Robot::generateObstacleVector(size_t i, double angleIncrement, double angleMin, double reading) {
	if (reading > params.criticalDistance) {
		return tf::Vector3(0, 0, 0);
	}

	double theta = angleMin + (i * angleIncrement);

	double newMagnitude = (params.criticalDistance - reading) / params.criticalDistance;

	double x;
	double y;
	if (newMagnitude > 0) {
		double ux = std::cos(theta);
		double uy = std::sin(theta);

		x = -ux * newMagnitude;
		y = -uy * newMagnitude;
	} else {
		x = 0;
		y = 0;
	}


	tf::Vector3 robotVector(x, y, 0);

	return robotVector;
}

tf::Vector3 Robot::transformToOdomFrame(tf::Vector3 robotVector) {
	try {

		std::string robotString = "robot_" + std::to_string(id);
		std::string source_frame = robotString + "/base_link";
		std::string target_frame = robotString + "/odom";

		geometry_msgs::Vector3Stamped base_vector;
	
		tf::StampedTransform transform;
		listener->lookupTransform(
			target_frame,
			source_frame,
			ros::Time(0),
			transform
		);

		tf::Matrix3x3 R(transform.getRotation());
		tf::Vector3 v_world = R * robotVector;

		return v_world;
	} catch (tf::TransformException &ex) {
		ROS_WARN("%s", ex.what());

		return tf::Vector3(0, 0, 0);
	}

}

void Robot::setOdom(Odom odom) {
    this->currentOdom = odom;
}

void Robot::setTarget(double targetX, double targetY) {
    this->targetX = targetX;
    this->targetY = targetY;
}
}