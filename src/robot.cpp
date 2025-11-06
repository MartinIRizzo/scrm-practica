#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "project1/robot.hpp"

namespace MUSIRobot {
Robot::Robot() {}

void Robot::init(int id, Algorithm algorithm, Params params) {
    this->id = id;
    this->selectedAlgorithm = algorithm;
    this->currentState = Normal;
    this->params = params;
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
		if (distance <= params.distanceToObjective) {
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
	geometry_msgs::Vector3 vObj = generateObjectiveVector();
	std::vector<geometry_msgs::Vector3> vObs = generateObstacleVectors(laserData);
}

geometry_msgs::Vector3 Robot::generateObjectiveVector() {
	double x = targetX - currentOdom.currentX;
	double y = targetY - currentOdom.currentY;

	double magnitude = std::sqrt(std::pow(x, 2) + std::pow(y, 2));

	x /= magnitude;
	y /= magnitude;

	geometry_msgs::Vector3 vector;
	vector.x = x;
	vector.y = y;

	return vector;
}

std::vector<geometry_msgs::Vector3> Robot::generateObstacleVectors(const sensor_msgs::LaserScan& laserData) {
	std::vector<geometry_msgs::Vector3> vectors;
	double angleIncrement = laserData.angle_increment;

	for (size_t i = 0; i < laserData.ranges.size(); i++) {
		double reading = laserData.ranges[i];

		geometry_msgs::Vector3 vector = generateObstacleVector(i, angleIncrement, reading);

	}
}

geometry_msgs::Vector3 Robot::generateObstacleVector(size_t i, double angleIncrement, double reading) {
	double theta = -M_PI + (i * angleIncrement);

	double x = reading * std::sin(theta);
	double y = reading * std::cos(theta);

	double magnitude = std::sqrt(std::pow(x, 2) + std::pow(y, 2));

	double newMagnitude = 0;
	if (reading <= params.criticalDistance) { 
		magnitude = (params.criticalDistance - reading) / params.criticalDistance;
	}

	x = (x / magnitude) * newMagnitude;
	y = (y / magnitude) * newMagnitude;

	// TODO: transform vector from robot relative to world's relative

}

void Robot::setOdom(Odom odom) {
    this->currentOdom = odom;
}

void Robot::setTarget(double targetX, double targetY) {
    this->targetX = targetX;
    this->targetY = targetY;
}
}